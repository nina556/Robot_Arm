#!/usr/bin/env python3
"""
EtherCAT Calibration Web UI — fast background polling
http://192.168.10.200:8080
"""
import http.server, json, math, os, subprocess, sys, tempfile, threading, time, yaml

PORT = 8080
YAML_PATH = "/home/niic/Light_test/src/light_test/config/master2_joints.yaml"
LIMITS_PATH = "/home/niic/Light_test/src/light_test/config/joint_limits_calibrated.yaml"
ROS2_SOURCE = "source /opt/ros/humble/setup.bash"
ROS_DOMAIN = "25"
NODE_NAME = "master2_encoder_publisher"

# ── thread-safe cache ─────────────────────────────────
cache_lock = threading.Lock()
file_lock = threading.Lock()
calibration_lock = threading.Lock()
CACHE = {
    "time": "---",
    "status": "stopped",
    "counts": None,
    "positions": None,
    "config": {"zero_offsets":[0]*7, "running_offsets":None, "directions":[1]*7,
               "counts_per_rev":[8388608.0]*7, "gear_ratios":[1.0]*7},
    "limits": {"lower":[None]*7, "upper":[None]*7, "lower_deg":[None]*7, "upper_deg":[None]*7},
}

# ── blocking I/O (only called from bg thread) ──────────
def _read_yaml():
    with open(YAML_PATH) as f:
        p = yaml.safe_load(f)["master2_encoder_publisher"]["ros__parameters"]
    return {"zero_offsets": p["zero_offsets"], "directions": p["directions"],
            "counts_per_rev": p["encoder_counts_per_rev"], "gear_ratios": p["gear_ratios"]}

def _read_limits():
    if not os.path.exists(LIMITS_PATH): return {"lower":[None]*7,"upper":[None]*7}
    d = yaml.safe_load(open(LIMITS_PATH))
    # ensure 7 elements
    for k in ("lower","upper"):
        if k not in d: d[k] = [None]*7
        while len(d[k]) < 7: d[k].append(None)
    return d

def _pub_status():
    try: subprocess.check_output("pgrep -f master2_encoder_publisher | grep -v grep | head -1", shell=True); return "running"
    except: return "stopped"

def _get_running_offsets():
    try:
        out = subprocess.check_output(f"bash -c '{ROS2_SOURCE} && export ROS_DOMAIN_ID={ROS_DOMAIN} && ros2 param get /{NODE_NAME} zero_offsets 2>/dev/null'", shell=True, timeout=2).decode()
        if "[" in out and "]" in out:
            arr = out[out.index("["):out.index("]")+1]
            v = [int(x.strip()) for x in arr.strip("[]").split(",") if x.strip()]
            if len(v) == 7: return v
    except: pass
    return None

# ── persistent topic subscribers (100Hz, no restart per poll) ──
_latest_counts = None
_latest_counts_at = 0.0
_latest_positions = None
_counts_lock = threading.Lock()
_positions_lock = threading.Lock()

def _reader_counts():
    """Persistent subprocess reader for /master2/encoder_counts"""
    global _latest_counts, _latest_counts_at
    while True:
        try:
            proc = subprocess.Popen(
                ["bash", "-c", f"{ROS2_SOURCE} && export ROS_DOMAIN_ID={ROS_DOMAIN} && ros2 topic echo /master2/encoder_counts 2>/dev/null"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
            buf = []
            for line in proc.stdout:
                s = line.strip()
                if s.startswith("- "):
                    buf.append(int(s[2:]))
                    if len(buf) == 7:
                        with _counts_lock:
                            _latest_counts = buf[:]
                            _latest_counts_at = time.monotonic()
                        buf = []
        except: time.sleep(1)

def _reader_joints():
    """Persistent subprocess reader for /master2/joint_states"""
    global _latest_positions
    while True:
        try:
            proc = subprocess.Popen(
                ["bash", "-c", f"{ROS2_SOURCE} && export ROS_DOMAIN_ID={ROS_DOMAIN} && ros2 topic echo /master2/joint_states 2>/dev/null"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
            pos = []; in_pos = False
            for line in proc.stdout:
                s = line.strip()
                if s == "position:":
                    in_pos = True; pos = []
                elif in_pos and s.startswith("- "):
                    pos.append(float(s[2:]))
                    if len(pos) == 7:
                        with _positions_lock: _latest_positions = pos[:]
                        in_pos = False
                elif in_pos and not s.startswith("- "):
                    in_pos = False
        except: time.sleep(1)

def poller():
    """Periodically read YAML/limits and update cache with latest topic data"""
    global CACHE
    while True:
        try:
            with _counts_lock: c = _latest_counts[:] if _latest_counts else None
            with _positions_lock: p = _latest_positions[:] if _latest_positions else None

            cfg = _read_yaml()
            lim = _read_limits()
            st = _pub_status()
            running = _get_running_offsets()

            with cache_lock:
                CACHE["time"] = time.strftime("%H:%M:%S")
                CACHE["status"] = st
                CACHE["counts"] = c
                CACHE["positions"] = p
                CACHE["config"] = {"zero_offsets": cfg["zero_offsets"], "running_offsets": running,
                    "directions": cfg["directions"], "counts_per_rev": cfg["counts_per_rev"], "gear_ratios": cfg["gear_ratios"]}
                CACHE["limits"] = lim

            time.sleep(0.2)  # update cache every 200ms from 100Hz data
        except Exception:
            time.sleep(1)

# ── actions (called from HTTP thread) ──────────────────
def _write_yaml_atomic(path, data):
    directory = os.path.dirname(path)
    fd, tmp_path = tempfile.mkstemp(prefix=".encoder-cal-", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(data, f, default_flow_style=None, allow_unicode=True, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
    except Exception:
        try:
            os.unlink(tmp_path)
        except FileNotFoundError:
            pass
        raise

def _inject_offsets(offsets):
    data_str = "{" + f"data: [{', '.join(str(int(v)) for v in offsets)}]" + "}"
    try:
        out = subprocess.check_output(
            ["bash", "-c", f"{ROS2_SOURCE} && export ROS_DOMAIN_ID={ROS_DOMAIN} && "
             f"ros2 topic pub -1 /master2/inject_offsets std_msgs/msg/Int64MultiArray '{data_str}' 2>&1"],
            timeout=5, stderr=subprocess.STDOUT).decode()
        return True, out.strip()
    except subprocess.TimeoutExpired:
        return False, "注入超时，YAML 未修改"
    except subprocess.CalledProcessError as e:
        detail = e.output.decode(errors="replace").strip() if e.output else str(e)
        return False, detail[-300:]

def action_calibrate_joint(joint, target_deg):
    """Atomically calibrate one joint from the latest server-side encoder count."""
    if isinstance(joint, bool) or not isinstance(joint, int) or not 0 <= joint < 7:
        raise ValueError("关节编号无效")
    if isinstance(target_deg, bool) or not isinstance(target_deg, (int, float)):
        raise ValueError("目标角度无效")
    target_deg = float(target_deg)
    if not math.isfinite(target_deg) or abs(target_deg) > 720:
        raise ValueError("目标角度必须在 -720° 到 720° 之间")

    with calibration_lock:
        with _counts_lock:
            raw = _latest_counts[joint] if _latest_counts else None
            count_age = time.monotonic() - _latest_counts_at
        if raw is None or count_age > 2.0:
            raise RuntimeError("编码器数据已中断，请恢复连接后重试")

        with file_lock:
            with open(YAML_PATH) as f:
                cfg = yaml.safe_load(f)
            params = cfg["master2_encoder_publisher"]["ros__parameters"]
            offsets = [int(v) for v in params["zero_offsets"]]
            direction = int(params["directions"][joint])
            counts_per_rev = float(params["encoder_counts_per_rev"][joint])
            gear_ratio = float(params["gear_ratios"][joint])
            denominator = counts_per_rev * gear_ratio
            if direction not in (-1, 1) or denominator <= 0:
                raise ValueError("该关节的方向、每圈计数或减速比配置无效")

            old_offset = offsets[joint]
            new_offset = round(raw - direction * target_deg / 360.0 * denominator)
            offsets[joint] = new_offset

            ok, detail = _inject_offsets(offsets)
            if not ok:
                raise RuntimeError(detail)
            params["zero_offsets"] = offsets
            _write_yaml_atomic(YAML_PATH, cfg)

        with cache_lock:
            CACHE["config"]["zero_offsets"] = offsets[:]
            CACHE["config"]["running_offsets"] = offsets[:]

        return {
            "joint": joint,
            "raw": raw,
            "target_deg": target_deg,
            "old_offset": old_offset,
            "new_offset": new_offset,
            "zero_offsets": offsets,
        }

def action_save_limits(lower, upper):
    """Save raw encoder limits — instant, no restart needed"""
    with file_lock:
        lim = {"lower": lower, "upper": upper}
        _write_yaml_atomic(LIMITS_PATH, lim)
    return True

# ── HTML (unchanged from mobile version) ──────────────
HTML = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Cal</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,sans-serif;background:#111119;color:#e0e0e0;touch-action:manipulation;user-select:none}
.header{background:#0a0a14;padding:10px 14px;display:flex;align-items:center;justify-content:space-between;border-bottom:1px solid #222}
.header h1{font-size:16px;color:#00d4ff}
.st{font-size:11px;padding:3px 8px;border-radius:10px}
.st-running{background:#00ff8811;color:#00ff88}
.st-stopped{background:#ff6b6b11;color:#ff6b6b}
.nav{display:flex;gap:3px;padding:6px 6px;overflow-x:auto;background:#0a0a14;border-bottom:1px solid #222}
.nav button{flex:0 0 auto;min-width:40px;height:32px;border-radius:8px;border:1px solid #333;background:#161626;color:#888;font-size:12px;font-weight:600;cursor:pointer}
.nav button.active{background:#00d4ff22;border-color:#00d4ff;color:#00d4ff}
.section{margin:8px 10px;background:#161626;border-radius:12px;padding:14px}
.section .title{font-size:12px;color:#888;margin-bottom:8px}
.big-row{display:flex;justify-content:space-around;text-align:center;margin:10px 0}
.big-val{font-size:26px;font-weight:700;font-family:monospace;color:#00ff88}
.big-label{font-size:10px;color:#666;margin-top:2px}
.big-val.warn{color:#ffaa00}
.btn-act{height:44px;border:none;border-radius:12px;font-size:14px;font-weight:700;cursor:pointer}
.btn-act:active{transform:scale(0.96)}
.btn-primary{width:100%;background:#00d4ff;color:#071014;font-size:16px}
.btn-primary:disabled{background:#333;color:#777}
.btn-zero{width:100%;border:1px solid #00ff88;background:transparent;color:#00ff88}
.btn-lim{flex:1;height:38px;border:1px solid #ffaa00;border-radius:8px;background:transparent;color:#ffaa00;font-size:12px;font-weight:600;cursor:pointer}
.btn-lim:active{background:#ffaa0011}
.limit-row{display:flex;gap:8px;margin:8px 0}
.limit-box{flex:1;background:#0f0f1c;border-radius:8px;padding:8px;text-align:center}
.limit-box .val{font-size:16px;font-weight:700;font-family:monospace;color:#ffaa00}
.limit-box .lbl{font-size:10px;color:#666}
.limit-none{color:#444!important}
.angle-entry{display:flex;align-items:center;gap:8px;margin:10px 0}
.angle-entry input{min-width:0;flex:1;height:54px;border:1px solid #444;border-radius:10px;background:#0f0f1c;color:#fff;font-size:26px;font-weight:700;text-align:center}
.angle-entry span{font-size:20px;color:#888}
.quick{display:flex;gap:6px;margin-bottom:10px}
.quick button{flex:1;height:34px;border:1px solid #3a3a50;border-radius:8px;background:#1a1a30;color:#bbb;font-weight:600}
.hint{font-size:11px;color:#777;line-height:1.5;margin-top:8px}
.saved-offset{font-family:monospace;color:#aaa}
.result{min-height:18px;margin-top:8px;text-align:center;font-size:12px;color:#00ff88}
.toast{position:fixed;top:12px;left:50%;transform:translateX(-50%);padding:8px 20px;border-radius:20px;font-size:13px;font-weight:600;z-index:99;opacity:0;transition:.3s;pointer-events:none}
.toast.show{opacity:1}
.toast-ok{background:#00ff8833;color:#00ff88}
.toast-err{background:#ff6b6b33;color:#ff6b6b}
</style>
</head>
<body>
<div class="header">
  <h1>🦾 <span id="joint-title">J1</span></h1>
  <span class="st st-running" id="status">---</span>
</div>
<div class="nav" id="nav"></div>

<div class="section">
  <div class="title" id="clock">---</div>
  <div class="big-row">
    <div><div class="big-val" id="cur-cts">---</div><div class="big-label">编码器</div></div>
    <div><div class="big-val" id="cur-deg">---</div><div class="big-label">角度 °</div></div>
    <div><div class="big-val" id="cur-diff">---</div><div class="big-label">差额</div></div>
  </div>
</div>

<div class="section">
  <div class="title">当前位置对应多少度？</div>
  <div class="angle-entry">
    <input type="number" id="target-deg" value="0" step="0.1" inputmode="decimal">
    <span>°</span>
  </div>
  <div class="quick">
    <button onclick="setTarget(-90)">-90°</button>
    <button onclick="setTarget(0)">0°</button>
    <button onclick="setTarget(90)">90°</button>
    <button onclick="setTarget(180)">180°</button>
  </div>
  <button class="btn-act btn-primary" id="btn-calibrate" onclick="calibrate()">标定当前关节并立即保存</button>
  <div style="height:8px"></div>
  <button class="btn-act btn-zero" id="btn-zero" onclick="calibrate(0)">快捷：当前位置设为 0°</button>
  <div class="hint">只修改 <span id="joint-hint">当前关节</span>。保存后立即同步 RViz/MoveIt，无需再点注入或重启。</div>
  <div class="hint">当前保存零点：<span class="saved-offset" id="off-val">---</span></div>
  <div class="result" id="cal-result"></div>
</div>

<div class="section">
  <div class="title">限位</div>
  <div class="limit-row">
    <div class="limit-box"><div class="lbl">🔴 下限 cts</div><div class="val" id="lim-lo-val">---</div><div class="lbl" id="lim-lo-deg" style="color:#ffaa00"></div></div>
    <div class="limit-box"><div class="lbl">🟢 上限 cts</div><div class="val" id="lim-hi-val">---</div><div class="lbl" id="lim-hi-deg" style="color:#ffaa00"></div></div>
  </div>
  <div style="display:flex;gap:8px;margin-bottom:8px">
    <button class="btn-lim" onclick="snapLim('lo')">🔴 设下限</button>
    <button class="btn-lim" onclick="snapLim('hi')">🟢 设上限</button>
  </div>
  <button class="btn-act" style="width:100%;border:2px solid #ffaa00;background:transparent;color:#ffaa00;height:40px" onclick="saveLimits()">💾 保存限位 (即时)</button>
</div>

<div id="toast" class="toast"></div>

<script>
let config={}, limits={}, counts=null, positions=null;
let curJoint=0, limDirty=false, calibrating=false;
const LABELS=['J1底座','J2肩部','J3肘部','J4腕1','J5腕2','J6腕3','J7末端'];

function nav(){let h='';for(let i=0;i<7;i++)h+=`<button class="${i===curJoint?'active':''}" onclick="selectJoint(${i})">J${i+1}</button>`;$('nav').innerHTML=h}
function selectJoint(i){curJoint=i;limDirty=false;$('cal-result').textContent='';render();nav()}
function swipe(d){let n=curJoint+d;if(n>=0&&n<7)selectJoint(n)}
function $(id){return document.getElementById(id)}

async function poll(){
  try{
    let r=await fetch('/api/state'),d=await r.json();
    let savedLimLo=(limDirty&&limits.lower)?limits.lower[curJoint]:null;
    let savedLimHi=(limDirty&&limits.upper)?limits.upper[curJoint]:null;
    config=d.config;limits=d.limits;counts=d.counts;positions=d.positions;
    if(d.config.running_offsets&&d.config.running_offsets.length===7)config.zero_offsets=d.config.running_offsets;
    if(limDirty&&savedLimLo!==null)limits.lower[curJoint]=savedLimLo;
    if(limDirty&&savedLimHi!==null)limits.upper[curJoint]=savedLimHi;
    $('status').textContent=d.status;$('status').className='st st-'+d.status;
    $('clock').textContent=d.time;
    render();nav();
  }catch(e){}
}

function render(){
  $('joint-title').textContent=LABELS[curJoint];
  $('joint-hint').textContent=LABELS[curJoint];
  let raw=(counts&&counts[curJoint]!=null)?counts[curJoint]:null;
  let deg=(positions&&positions[curJoint]!=null)?positions[curJoint]:'---';
  let off=(config.zero_offsets&&config.zero_offsets[curJoint]!=null)?config.zero_offsets[curJoint]:0;
  let diff=(raw!=null)?(raw-off):null;
  $('cur-cts').textContent=raw!=null?raw.toLocaleString():'---';$('cur-cts').className='big-val'+(raw!=null?'':' warn');
  $('cur-deg').textContent=deg!=='---'?(deg*180/Math.PI).toFixed(1)+'°':'---';
  $('cur-diff').textContent=diff!=null?diff.toLocaleString():'---';$('cur-diff').className='big-val'+(diff!=null&&Math.abs(diff)<1000?'':' warn');
  $('off-val').textContent=off.toLocaleString();
  $('btn-calibrate').disabled=calibrating||raw==null;
  $('btn-zero').disabled=calibrating||raw==null;
  let lo=limits.lower||[],hi=limits.upper||[],cpr=config.counts_per_rev[curJoint],gr=config.gear_ratios[curJoint],dir=config.directions[curJoint],zo=config.zero_offsets[curJoint];
  $('lim-lo-val').textContent=lo[curJoint]!=null?lo[curJoint].toLocaleString():'---';$('lim-lo-val').className='val'+(lo[curJoint]!=null?'':' limit-none');
  $('lim-hi-val').textContent=hi[curJoint]!=null?hi[curJoint].toLocaleString():'---';$('lim-hi-val').className='val'+(hi[curJoint]!=null?'':' limit-none');
  $('lim-lo-deg').textContent=lo[curJoint]!=null?(dir*(lo[curJoint]-zo)/(cpr*gr)*360).toFixed(1)+'°':'';
  $('lim-hi-deg').textContent=hi[curJoint]!=null?(dir*(hi[curJoint]-zo)/(cpr*gr)*360).toFixed(1)+'°':'';
}

function setTarget(deg){$('target-deg').value=deg}
function snapLim(w){let r=(counts&&counts[curJoint]!=null)?counts[curJoint]:null;if(r==null)return;if(!limits.lower)limits.lower=Array(7).fill(null);if(!limits.upper)limits.upper=Array(7).fill(null);if(w==='lo')limits.lower[curJoint]=r;else limits.upper[curJoint]=r;limDirty=true;render();toast(LABELS[curJoint]+' '+(w==='lo'?'下限':'上限')+'='+r.toLocaleString(),'ok')}

async function calibrate(forcedDeg){
  let deg=forcedDeg===undefined?Number($('target-deg').value):forcedDeg;
  if(!Number.isFinite(deg)||Math.abs(deg)>720){toast('请输入 -720° 到 720° 的角度','err');return}
  let joint=curJoint,label=LABELS[joint];
  calibrating=true;$('cal-result').textContent='正在保存并同步...';render();
  try{
    let r=await fetch('/api/calibrate_joint',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({joint:joint,target_deg:deg})});
    let d=await r.json();
    if(!d.ok)throw new Error(d.error||'标定失败');
    config.zero_offsets=d.zero_offsets;
    $('cal-result').textContent=`${label} 已保存：${d.raw.toLocaleString()} cts = ${d.target_deg}°`;
    toast(label+' 标定成功','ok');
    await poll();
  }catch(e){$('cal-result').textContent='';toast(e.message,'err')}
  finally{calibrating=false;render()}
}

async function saveLimits(){
  let lo=limits.lower||[],hi=limits.upper||[];
  try{
    let r=await fetch('/api/save_limits',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({lower:lo,upper:hi})});
    let d=await r.json();
    if(d.ok){limDirty=false;toast('✅ 限位已保存','ok')} else toast('❌ '+(d.error||''),'err');
  }catch(e){toast('❌ '+e.message,'err')}
}

function toast(t,ty){let e=$('toast');e.textContent=t;e.className='toast toast-'+ty+' show';setTimeout(()=>e.classList.remove('show'),2000)}

let tx=0;
document.addEventListener('touchstart',e=>{tx=e.touches[0].clientX});
document.addEventListener('touchend',e=>{let dx=e.changedTouches[0].clientX-tx;if(Math.abs(dx)>60)swipe(dx>0?-1:1)});
document.addEventListener('keydown',e=>{if(e.key==='ArrowLeft')swipe(-1);if(e.key==='ArrowRight')swipe(1)});

poll();setInterval(poll,1000);
</script>
</body>
</html>"""

# ── HTTP (fast, non-blocking) ─────────────────────────
class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self,f,*a): pass
    def _json(self,d):
        self.send_response(200); self.send_header("Content-Type","application/json")
        self.send_header("Cache-Control","no-cache"); self.end_headers()
        self.wfile.write(json.dumps(d).encode())

    def do_GET(self):
        if self.path == "/":
            self.send_response(200); self.send_header("Content-Type","text/html; charset=utf-8")
            self.send_header("Cache-Control","no-cache"); self.end_headers(); self.wfile.write(HTML.encode())
        elif self.path == "/api/state":
            with cache_lock: data = dict(CACHE)  # shallow copy under lock
            self._json(data)
        else: self.send_error(404)

    def do_POST(self):
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            if self.path == "/api/calibrate_joint":
                result = action_calibrate_joint(body.get("joint"), body.get("target_deg"))
                self._json({"ok": True, **result})
            elif self.path == "/api/save_limits":
                ok = action_save_limits(body.get("lower", []), body.get("upper", []))
                self._json({"ok": ok})
            else: self.send_error(404)
        except Exception as e:
            self._json({"ok": False, "error": str(e)})

def main():
    if not os.path.exists(YAML_PATH): print(f"ERROR: missing {YAML_PATH}"); sys.exit(1)
    # start persistent topic readers + poller
    threading.Thread(target=_reader_counts, daemon=True).start()
    threading.Thread(target=_reader_joints, daemon=True).start()
    time.sleep(1)  # let DDS discovery complete
    threading.Thread(target=poller, daemon=True).start()
    print(f"  Fast UI: http://192.168.10.200:{PORT}")
    http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()

if __name__ == "__main__":
    main()
