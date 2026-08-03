# MuJoCo VLA Joint Chunk HTTP API

## 启动服务

在 `mujoco` 分支启动 MuJoCo Docker 仿真：

```bash
MUJOCO_VIEWER=0 MUJOCO_CAMERA_ENABLED=0 MUJOCO_WRIST_CAMERA_ENABLED=0 MUJOCO_OVERVIEW_CAMERA_ENABLED=0 ./start_mujoco_sim.sh --docker
```

默认 HTTP 端口是 `8765`。如需换端口：

```bash
WEB_CONTROL_HOST=0.0.0.0 WEB_CONTROL_PORT=8877 MUJOCO_VIEWER=0 MUJOCO_CAMERA_ENABLED=0 MUJOCO_WRIST_CAMERA_ENABLED=0 MUJOCO_OVERVIEW_CAMERA_ENABLED=0 ./start_mujoco_sim.sh --docker
```

局域网访问地址：

```text
http://<运行这台电脑的局域网IP>:<WEB_CONTROL_PORT>
```

本机查询局域网 IP：

```bash
hostname -I
```

## VLA 关节 chunk 接口

请求方法：

```text
POST
```

请求地址：

```text
http://<host-ip>:<port>/api/mujoco/vla_joint_chunk
```

本机示例：

```text
http://127.0.0.1:8765/api/mujoco/vla_joint_chunk
```

局域网示例：

```text
http://192.168.10.38:8765/api/mujoco/vla_joint_chunk
```

Content-Type：

```text
application/json
```

## 请求体

```json
{
  "command_type": "joint_chunk",
  "unit": "rad",
  "arms": "right",
  "fps": 20,
  "execute": false,
  "result_timeout_sec": 5,
  "actions": [
    [0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0]
  ],
  "meta": {
    "instruction": "右臂J1增加0.01rad"
  }
}
```

字段说明：

| 字段 | 必填 | 说明 |
|---|---:|---|
| `command_type` | 否 | 固定用 `joint_chunk`，兼容旧占位 `cartesian_pose` |
| `unit` | 否 | 当前只按 `rad` 理解；数值本身就是弧度 |
| `arms` | 否 | `right`、`left`、`both`，默认 `both` |
| `fps` | 否 | chunk 执行频率，默认 `20` |
| `execute` | 否 | `false` 只校验 dry-run；`true` 执行到 MuJoCo driver |
| `result_timeout_sec` | 否 | HTTP 等待 driver 返回的超时，默认 `5`，最大实际等待 `30` |
| `actions` | 是 | 非空二维数组，形状 `N x 16` |
| `meta` | 否 | 上游 VLA 元信息，接口透传到日志/调试链路 |

## 16 维动作含义

`actions` 里的每一行都是 16 维关节目标，输入数值就是 `rad`：

```text
1.0 = 1 rad
0.01 = 0.01 rad
```

不做 mean/std 反归一化，也不做 `[-1, 1]` 到关节上下限的映射。

当前 HTTP 接口接收的是关节目标轨迹，不是 delta 字段。如果上游要做增量控制，应先取当前关节值，然后生成 `当前值 + 增量` 的 16 维目标 chunk。例如右臂 J1 当前为 `0.02 rad`，要增加 `0.01 rad`，第 8 维目标应发到 `0.03`。

| 下标 | 名称 | 含义 |
|---:|---|---|
| 0 | `Left_Joint1` | 左臂 J1 |
| 1 | `Left_Joint2` | 左臂 J2 |
| 2 | `Left_Joint3` | 左臂 J3 |
| 3 | `Left_Joint4` | 左臂 J4 |
| 4 | `Left_Joint5` | 左臂 J5 |
| 5 | `Left_Joint6` | 左臂 J6 |
| 6 | `Left_Joint7` | 左臂 J7 |
| 7 | `Left_Gripper` | 左夹爪，当前 MuJoCo VLA 关节接口不执行夹爪 |
| 8 | `Right_Joint1` | 右臂 J1 |
| 9 | `Right_Joint2` | 右臂 J2 |
| 10 | `Right_Joint3` | 右臂 J3 |
| 11 | `Right_Joint4` | 右臂 J4 |
| 12 | `Right_Joint5` | 右臂 J5 |
| 13 | `Right_Joint6` | 右臂 J6 |
| 14 | `Right_Joint7` | 右臂 J7 |
| 15 | `Right_Gripper` | 右夹爪，当前 MuJoCo VLA 关节接口不执行夹爪 |

上下限来源：

```text
compiled MuJoCo joint limits
```

也就是 MuJoCo 模型编译后的关节位置上下限。超限时 driver 返回 `ok=false` 或 HTTP 返回错误。

## 响应体

成功 dry-run：

```json
{
  "ok": true,
  "request_id": "web-1784880000000000000",
  "command_type": "joint_chunk",
  "unit": "rad",
  "normalization": "none; input value is rad",
  "limit_source": "compiled MuJoCo joint limits",
  "points": 1,
  "fps": 20.0,
  "duration_sec": 0.05,
  "arms": ["right"],
  "execute": false,
  "topics": {
    "command": "/mujoco/vla_joint_chunk",
    "result": "/mujoco/vla_joint_chunk_result"
  }
}
```

执行成功时 `execute` 为 `true`。

## curl 示例

dry-run：

```bash
curl -sS -X POST "http://127.0.0.1:8765/api/mujoco/vla_joint_chunk" \
  -H "Content-Type: application/json" \
  -d '{
    "command_type": "joint_chunk",
    "unit": "rad",
    "arms": "right",
    "fps": 20,
    "execute": false,
    "actions": [[0,0,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0]]
  }' | python3 -m json.tool
```

执行右臂 J1 到 `0.01 rad`：

```bash
curl -sS -X POST "http://127.0.0.1:8765/api/mujoco/vla_joint_chunk" \
  -H "Content-Type: application/json" \
  -d '{
    "command_type": "joint_chunk",
    "unit": "rad",
    "arms": "right",
    "fps": 20,
    "execute": true,
    "actions": [[0,0,0,0,0,0,0,0,0.01,0,0,0,0,0,0,0]]
  }' | python3 -m json.tool
```

局域网机器访问时，把 `127.0.0.1` 换成运行仿真的电脑 IP。

## Python 示例

```python
import requests

url = "http://192.168.10.38:8765/api/mujoco/vla_joint_chunk"
body = {
    "command_type": "joint_chunk",
    "unit": "rad",
    "arms": "right",
    "fps": 20,
    "execute": True,
    "actions": [[0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0]],
}

resp = requests.post(url, json=body, timeout=8)
resp.raise_for_status()
print(resp.json())
```

## 命令行测试脚本

ROS topic smoke：

```bash
python3 tests/mujoco_vla_bridge_smoke.py --docker --arms right --joint-index 8 --delta-rad 0.01 --steps 5 --fps 20 --execute
```

HTTP smoke：

```bash
python3 tests/mujoco_vla_http_smoke.py --base-url http://127.0.0.1:8765 --execute
```
