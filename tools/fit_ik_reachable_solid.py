#!/usr/bin/env python3
"""Fit a reachable 3D solid from IK/OMPL benchmark points.

This intentionally avoids heavy scientific dependencies.  The model is a
voxelized KNN occupancy estimate: each voxel center is classified from nearby
success/failure samples, then the largest connected occupied component is
converted to an OBJ surface mesh and a self-contained interactive HTML view.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
from collections import deque
from pathlib import Path
from typing import Any


def parse_xyz(raw: str, label: str) -> tuple[float, float, float]:
    parts = [item.strip() for item in raw.split(",")]
    if len(parts) != 3:
        raise ValueError(f"{label} must be x,y,z")
    values = tuple(float(item) for item in parts)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains non-finite values")
    return values  # type: ignore[return-value]


def read_points(path: Path, success_field: str) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8", newline="") as handle:
        for row in csv.DictReader(handle):
            field = "plan_ok" if success_field == "auto" and "plan_ok" in row else success_field
            if field == "auto":
                field = "ik_ok"
            ok = str(row.get(field, "")).lower() == "true"
            rows.append({
                "x": float(row["x"]),
                "y": float(row["y"]),
                "z": float(row["z"]),
                "ok": ok,
                "ik_ok": str(row.get("ik_ok", "")).lower() == "true",
                "plan_ok": str(row.get("plan_ok", "")).lower() == "true",
            })
    if not rows:
        raise ValueError(f"no points in {path}")
    return rows


def classify_grid(points: list[dict[str, Any]], region_min: tuple[float, float, float], region_max: tuple[float, float, float], grid: tuple[int, int, int], k: int, threshold: float) -> tuple[set[tuple[int, int, int]], dict[str, Any]]:
    nx, ny, nz = grid
    spans = [region_max[i] - region_min[i] for i in range(3)]
    cell = [spans[0] / nx, spans[1] / ny, spans[2] / nz]
    normalized = []
    for p in points:
        normalized.append((
            (p["x"] - region_min[0]) / spans[0],
            (p["y"] - region_min[1]) / spans[1],
            (p["z"] - region_min[2]) / spans[2],
            1 if p["ok"] else 0,
        ))

    occupied: set[tuple[int, int, int]] = set()
    probabilities: list[float] = []
    for ix in range(nx):
        cx = (ix + 0.5) / nx
        for iy in range(ny):
            cy = (iy + 0.5) / ny
            for iz in range(nz):
                cz = (iz + 0.5) / nz
                nearest: list[tuple[float, int]] = []
                for px, py, pz, ok in normalized:
                    d2 = (px - cx) * (px - cx) + (py - cy) * (py - cy) + (pz - cz) * (pz - cz)
                    if len(nearest) < k:
                        heapq.heappush(nearest, (-d2, ok))
                    elif d2 < -nearest[0][0]:
                        heapq.heapreplace(nearest, (-d2, ok))
                prob = sum(ok for _, ok in nearest) / len(nearest)
                probabilities.append(prob)
                if prob >= threshold:
                    occupied.add((ix, iy, iz))

    stats = {
        "grid": {"nx": nx, "ny": ny, "nz": nz},
        "k": k,
        "threshold": threshold,
        "cell_size_m": {"x": cell[0], "y": cell[1], "z": cell[2]},
        "voxel_volume_m3": cell[0] * cell[1] * cell[2],
        "raw_occupied_voxels": len(occupied),
        "mean_voxel_probability": sum(probabilities) / len(probabilities),
    }
    return occupied, stats


def largest_component(occupied: set[tuple[int, int, int]]) -> set[tuple[int, int, int]]:
    seen: set[tuple[int, int, int]] = set()
    best: set[tuple[int, int, int]] = set()
    dirs = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
    for start in occupied:
        if start in seen:
            continue
        comp: set[tuple[int, int, int]] = set()
        q: deque[tuple[int, int, int]] = deque([start])
        seen.add(start)
        while q:
            cell = q.popleft()
            comp.add(cell)
            x, y, z = cell
            for dx, dy, dz in dirs:
                nxt = (x + dx, y + dy, z + dz)
                if nxt in occupied and nxt not in seen:
                    seen.add(nxt)
                    q.append(nxt)
        if len(comp) > len(best):
            best = comp
    return best


def build_mesh(occupied: set[tuple[int, int, int]], region_min: tuple[float, float, float], region_max: tuple[float, float, float], grid: tuple[int, int, int]) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int, int]]]:
    nx, ny, nz = grid
    step = ((region_max[0] - region_min[0]) / nx, (region_max[1] - region_min[1]) / ny, (region_max[2] - region_min[2]) / nz)
    vertices: list[tuple[float, float, float]] = []
    index: dict[tuple[int, int, int], int] = {}
    faces: list[tuple[int, int, int, int]] = []

    def vid(corner: tuple[int, int, int]) -> int:
        if corner not in index:
            index[corner] = len(vertices)
            vertices.append((region_min[0] + corner[0] * step[0], region_min[1] + corner[1] * step[1], region_min[2] + corner[2] * step[2]))
        return index[corner]

    face_defs = [
        ((-1, 0, 0), ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))),
        ((1, 0, 0), ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1))),
        ((0, -1, 0), ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1))),
        ((0, 1, 0), ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0))),
        ((0, 0, -1), ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0))),
        ((0, 0, 1), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))),
    ]
    for x, y, z in occupied:
        for (dx, dy, dz), corners in face_defs:
            if (x + dx, y + dy, z + dz) in occupied:
                continue
            faces.append(tuple(vid((x + cx, y + cy, z + cz)) for cx, cy, cz in corners))  # type: ignore[arg-type]
    return vertices, faces


def write_obj(path: Path, vertices: list[tuple[float, float, float]], faces: list[tuple[int, int, int, int]]) -> None:
    with path.open("w", encoding="utf-8") as handle:
        handle.write("# IK reachable fitted solid mesh\n")
        for x, y, z in vertices:
            handle.write(f"v {x:.9f} {y:.9f} {z:.9f}\n")
        for face in faces:
            handle.write("f " + " ".join(str(i + 1) for i in face) + "\n")


def mesh_neighbors(vertex_count: int, faces: list[tuple[int, int, int, int]]) -> list[set[int]]:
    neighbors = [set() for _ in range(vertex_count)]
    for face in faces:
        for i, a in enumerate(face):
            b = face[(i + 1) % len(face)]
            neighbors[a].add(b)
            neighbors[b].add(a)
    return neighbors


def smooth_vertices(
    vertices: list[tuple[float, float, float]],
    faces: list[tuple[int, int, int, int]],
    iterations: int,
    lam: float,
    mu: float,
) -> list[tuple[float, float, float]]:
    """Taubin-style smoothing to reduce voxel stair-steps without large shrinkage."""
    if iterations <= 0 or not vertices:
        return list(vertices)
    neighbors = mesh_neighbors(len(vertices), faces)
    current = [list(v) for v in vertices]

    def step(points: list[list[float]], factor: float) -> list[list[float]]:
        out = [p[:] for p in points]
        for idx, ns in enumerate(neighbors):
            if not ns:
                continue
            avg = [0.0, 0.0, 0.0]
            for n in ns:
                avg[0] += points[n][0]
                avg[1] += points[n][1]
                avg[2] += points[n][2]
            inv = 1.0 / len(ns)
            avg = [avg[0] * inv, avg[1] * inv, avg[2] * inv]
            out[idx][0] = points[idx][0] + factor * (avg[0] - points[idx][0])
            out[idx][1] = points[idx][1] + factor * (avg[1] - points[idx][1])
            out[idx][2] = points[idx][2] + factor * (avg[2] - points[idx][2])
        return out

    for _ in range(iterations):
        current = step(current, lam)
        current = step(current, mu)
    return [(p[0], p[1], p[2]) for p in current]


def write_html(path: Path, vertices: list[tuple[float, float, float]], faces: list[tuple[int, int, int, int]], points: list[dict[str, Any]], summary: dict[str, Any]) -> None:
    ok_points = [{"x": p["x"], "y": p["y"], "z": p["z"], "ok": p["ok"]} for p in points]
    html = f'''<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IK 可解实体拟合</title><style>
body{{margin:0;display:grid;grid-template-columns:minmax(0,1fr)360px;min-height:100vh;background:#081316;color:#eafffb;font-family:Arial,"Microsoft YaHei",sans-serif}}canvas{{display:block;width:100%;height:100vh;background:radial-gradient(circle at 50% 38%,#173239,#081316 72%);cursor:grab}}aside{{background:#0d1f23;border-left:1px solid #294d52;padding:18px 20px;box-sizing:border-box;overflow:auto}}h1{{font-size:18px;margin:0 0 12px}}.metric{{border:1px solid #24494f;border-radius:8px;background:#10282d;margin:8px 0;padding:10px 12px}}.label{{font-size:12px;color:#9ac8ca}}.value{{font-size:18px;font-weight:700;margin-top:4px;white-space:pre-line}}.small{{font-size:12px;color:#a8cfd0;line-height:1.55;white-space:pre-line}}button{{width:100%;padding:10px;margin-top:10px;border-radius:8px;border:1px solid #2d6970;background:#123238;color:#d7fff8;font-weight:700}}@media(max-width:900px){{body{{grid-template-columns:1fr}}canvas{{height:70vh}}aside{{border-left:0;border-top:1px solid #294d52}}}}
</style></head><body><canvas id="view"></canvas><aside><h1>IK 可解实体拟合</h1><div class="metric"><div class="label">模型</div><div class="value">KNN 体素实体</div></div><div class="metric"><div class="label">拟合体积 / 工作区占比</div><div class="value" id="vol"></div></div><div class="metric"><div class="label">样本成功率</div><div class="value" id="rate"></div></div><div class="metric"><div class="label">网格</div><div class="value" id="mesh"></div></div><div class="metric"><div class="label">参数</div><div class="small" id="params"></div></div><div class="small">拖动旋转，滚轮缩放。绿色透明面是拟合出的可解实体，亮点是成功样本，红点是失败样本。</div><button id="toggle">切换点云显示</button><button id="reset">重置视角</button></aside><script>
const V={json.dumps(vertices,separators=(',',':'))};const F={json.dumps(faces,separators=(',',':'))};const P={json.dumps(ok_points,separators=(',',':'))};const S={json.dumps(summary,ensure_ascii=False,separators=(',',':'))};
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');let yaw=-.75,pitch=.55,zoom=1,drag=false,lx=0,ly=0,showPts=true;const R=S.region;const B={{min:{{x:R.min[0],y:R.min[1],z:R.min[2]}},max:{{x:R.max[0],y:R.max[1],z:R.max[2]}},span:{{x:R.max[0]-R.min[0],y:R.max[1]-R.min[1],z:R.max[2]-R.min[2]}}}};
function put(id,v){{document.getElementById(id).textContent=v}}put('vol',`${{S.fitted_volume_m3.toFixed(6)}} m³\n${{S.fitted_volume_liters.toFixed(2)}} L\n${{(S.workspace_share*100).toFixed(1)}}%`);put('rate',`${{S.success_label}} ${{(S.sample_success_rate*100).toFixed(1)}}%\n${{S.success_points}}/${{S.total_points}}`);put('mesh',`${{S.mesh_vertices}} 顶点\n${{S.mesh_faces}} 面`);put('params',`grid=${{S.grid.nx}}×${{S.grid.ny}}×${{S.grid.nz}}\nk=${{S.k}} threshold=${{S.threshold}}\n保留最大连通体：${{S.kept_voxels}}/${{S.raw_occupied_voxels}} voxels`);
function resize(){{const r=canvas.getBoundingClientRect();canvas.width=Math.max(1,Math.round(r.width*devicePixelRatio));canvas.height=Math.max(1,Math.round(r.height*devicePixelRatio));draw()}}window.addEventListener('resize',resize);
function norm(p){{const cx=(B.min.x+B.max.x)/2,cy=(B.min.y+B.max.y)/2,cz=(B.min.z+B.max.z)/2,s=1/Math.max(B.span.x,B.span.y,B.span.z);return{{x:(p[0]-cx)*s,y:(p[1]-cy)*s,z:(p[2]-cz)*s}}}}function normObj(p){{return norm([p.x,p.y,p.z])}}function rot(p){{const cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);const x=p.x*cy-p.y*sy,y=p.x*sy+p.y*cy,z=p.z;return{{x:x,y:y*cp-z*sp,z:y*sp+z*cp}}}}function proj3(p){{const r=rot(p),pers=1.85/(1.85-r.z*.55),base=Math.min(canvas.width,canvas.height)*.72*zoom;return{{x:canvas.width/2+r.x*base*pers,y:canvas.height/2-r.y*base*pers,z:r.z,p:pers}}}}function pv(v){{return proj3(norm(v))}}
function drawBox(){{const xs=[B.min.x,B.max.x],ys=[B.min.y,B.max.y],zs=[B.min.z,B.max.z],c=[];for(const x of xs)for(const y of ys)for(const z of zs)c.push(pv([x,y,z]));const edges=[[0,1],[0,2],[0,4],[3,1],[3,2],[3,7],[5,1],[5,4],[5,7],[6,2],[6,4],[6,7]];ctx.strokeStyle='rgba(125,224,232,.38)';ctx.lineWidth=1.3*devicePixelRatio;for(const[e1,e2]of edges){{ctx.beginPath();ctx.moveTo(c[e1].x,c[e1].y);ctx.lineTo(c[e2].x,c[e2].y);ctx.stroke()}}}}
function draw(){{ctx.clearRect(0,0,canvas.width,canvas.height);drawBox();const faces=F.map((f,i)=>{{const ps=f.map(j=>pv(V[j]));return{{ps,z:ps.reduce((a,p)=>a+p.z,0)/ps.length,i}}}}).sort((a,b)=>a.z-b.z);for(const fa of faces){{const ps=fa.ps;ctx.beginPath();ctx.moveTo(ps[0].x,ps[0].y);for(let i=1;i<ps.length;i++)ctx.lineTo(ps[i].x,ps[i].y);ctx.closePath();const shade=Math.max(0,Math.min(1,(fa.z+.7)/1.4));ctx.fillStyle=`rgba(${{Math.round(28+shade*30)}},${{Math.round(150+shade*80)}},${{Math.round(95+shade*60)}},0.34)`;ctx.strokeStyle='rgba(170,255,220,.12)';ctx.fill();ctx.stroke()}}if(showPts){{const pts=P.map(p=>{{const q=proj3(normObj(p));return{{...q,ok:p.ok}}}}).sort((a,b)=>a.z-b.z);for(const p of pts){{ctx.beginPath();ctx.arc(p.x,p.y,(p.ok?1.8:2.4)*devicePixelRatio*p.p,0,Math.PI*2);ctx.fillStyle=p.ok?'rgba(220,255,220,.55)':'rgba(239,68,68,.75)';ctx.fill()}}}}}}
canvas.addEventListener('pointerdown',e=>{{drag=true;lx=e.clientX;ly=e.clientY;canvas.setPointerCapture(e.pointerId)}});canvas.addEventListener('pointermove',e=>{{if(!drag)return;yaw+=(e.clientX-lx)*.008;pitch+=(e.clientY-ly)*.008;pitch=Math.max(-1.35,Math.min(1.35,pitch));lx=e.clientX;ly=e.clientY;draw()}});canvas.addEventListener('pointerup',()=>drag=false);canvas.addEventListener('wheel',e=>{{e.preventDefault();zoom*=Math.exp(-e.deltaY*.001);zoom=Math.max(.45,Math.min(3.5,zoom));draw()}},{{passive:false}});document.getElementById('toggle').onclick=()=>{{showPts=!showPts;draw()}};document.getElementById('reset').onclick=()=>{{yaw=-.75;pitch=.55;zoom=1;draw()}};resize();
</script></body></html>'''
    path.write_text(html, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit a 3D reachable solid from IK benchmark points")
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--region-min", required=True)
    parser.add_argument("--region-max", required=True)
    parser.add_argument("--grid", default="40,35,25")
    parser.add_argument("--k", type=int, default=18)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--success-field", choices=["auto", "ik_ok", "plan_ok"], default="auto", help="CSV column used as reachable/success label")
    parser.add_argument("--smooth-iterations", type=int, default=0, help="Taubin smoothing iterations for a less blocky visual mesh")
    parser.add_argument("--smooth-lambda", type=float, default=0.45, help="positive Taubin smoothing factor")
    parser.add_argument("--smooth-mu", type=float, default=-0.47, help="negative Taubin smoothing factor to reduce shrinkage")
    args = parser.parse_args()

    points = read_points(args.csv, args.success_field)
    region_min = parse_xyz(args.region_min, "--region-min")
    region_max = parse_xyz(args.region_max, "--region-max")
    grid = tuple(int(v) for v in parse_xyz(args.grid, "--grid"))
    if len(grid) != 3 or any(v <= 0 for v in grid):
        raise ValueError("--grid values must be positive")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    occupied, model_stats = classify_grid(points, region_min, region_max, grid, args.k, args.threshold)
    kept = largest_component(occupied)
    vertices, faces = build_mesh(kept, region_min, region_max, grid)
    render_vertices = smooth_vertices(vertices, faces, args.smooth_iterations, args.smooth_lambda, args.smooth_mu)

    workspace_volume = (region_max[0] - region_min[0]) * (region_max[1] - region_min[1]) * (region_max[2] - region_min[2])
    fitted_volume = len(kept) * model_stats["voxel_volume_m3"]
    success_points = sum(1 for p in points if p["ok"])
    summary = {
        "model": "voxel_knn_largest_component",
        "region": {"min": list(region_min), "max": list(region_max)},
        "total_points": len(points),
        "success_points": success_points,
        "failure_points": len(points) - success_points,
        "success_field": args.success_field,
        "success_label": "IK" if args.success_field == "ik_ok" else ("IK+OMPL" if args.success_field == "plan_ok" else "成功"),
        "sample_success_rate": success_points / len(points),
        "workspace_volume_m3": workspace_volume,
        "workspace_volume_liters": workspace_volume * 1000,
        "fitted_volume_m3": fitted_volume,
        "fitted_volume_liters": fitted_volume * 1000,
        "workspace_share": fitted_volume / workspace_volume,
        "raw_occupied_voxels": model_stats["raw_occupied_voxels"],
        "kept_voxels": len(kept),
        "removed_voxels": model_stats["raw_occupied_voxels"] - len(kept),
        "mesh_vertices": len(vertices),
        "mesh_faces": len(faces),
        "smooth_iterations": args.smooth_iterations,
        "smooth_lambda": args.smooth_lambda,
        "smooth_mu": args.smooth_mu,
        **model_stats,
    }
    (args.output_dir / "reachable_solid_fit_summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_obj(args.output_dir / "reachable_solid_fit.obj", render_vertices, faces)
    write_html(args.output_dir / "reachable_solid_fit.html", render_vertices, faces, points, summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
