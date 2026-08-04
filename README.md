# UnoArm 双臂机器人控制与仿真平台

UnoArm 是一套基于 ROS 2 Humble 的双臂机器人软件栈，集成 MoveIt 2 运动规划、MuJoCo 物理仿真、EtherCAT/CSP 真机桥接、RGB-D 视觉、抓取流程和 Web 控制界面。

当前仓库的默认开发目标是**本地 MuJoCo 仿真**。仿真复用与真机相同的 MoveIt、ROS action/service/topic 和 Web API，但不会启动 EtherCAT master 或 CSP 真机驱动。除非明确进行真机调试，否则请使用：

```bash
./start_mujoco_sim.sh --docker
```

启动完成后打开：

```text
http://127.0.0.1:8765
```

## 主要能力

- 双 7 自由度机械臂、左右夹爪、底座和相机组件的完整 URDF/STL 模型。
- MoveIt 2 的 IK、OMPL 路径规划、轨迹执行、规划场景和碰撞体管理。
- MuJoCo 双臂物理仿真，接口与真机控制链路兼容。
- 头部 D455、左右腕部相机和可选全局相机的 RGB-D 仿真。
- 基于 Vite、Three.js 和 URDFLoader 的浏览器三维控制界面。
- 关节微动、点位/预设/序列、回零、夹爪、抓取、放置和急停控制。
- 点云、目标位姿、工作空间、手工碰撞盒和平台障碍物可视化。
- MuJoCo 转子抓取、智能多姿态搜索、相机标定与抓取调试数据。
- VLM/VLA 接口、可达性采样、IK/规划器评估和抓取 benchmark 工具。
- 双臂 EtherCAT CSP 真机驱动与 MoveIt trajectory bridge（仅用于明确授权的真机环境）。

## 系统架构

```text
浏览器（Three.js + URDFLoader）
              │ HTTP / JSON / MJPEG
              ▼
web_control/web_control/server.py
      │ ROS service/action/topic
      ├──────────────► MoveIt 2 / OMPL / PlanningScene
      │                         │ FollowJointTrajectory
      │                         ▼
      │              ┌──────────┴──────────┐
      │              │                     │
      │       MuJoCo simulation     CSP/EtherCAT bridge
      │        （默认开发）           （真机专用）
      │
      └──────────────► RGB-D、点云、VLM/VLA 与抓取流程
```

MuJoCo 栈提供与真机兼容的核心控制接口，包括：

```text
/arm_controller/follow_joint_trajectory
/left_arm_controller/follow_joint_trajectory
/master2/joint_states
/master0/joint_states
/master2/joint_trajectory
/master0/joint_trajectory
```

## 项目结构

```text
unoarm/
├── start_mujoco_sim.sh          # MuJoCo + MoveIt + Web 一键启动
├── stop_mujoco_sim.sh           # 只停止本项目记录的仿真进程
├── start_robot_all.sh           # 真机完整栈入口（谨慎使用）
├── stop_robot_all.sh            # 真机/全局栈停止脚本
├── start_robot_moveit.sh        # MoveIt 启动入口
├── start_csp_driver.sh          # 真机 CSP bridge 入口
├── run_web_control.sh           # Web 服务启动、检查和停止
├── unoarm_menu.sh               # 真机命令行控制菜单
├── asm0003/                     # ROS 描述、MoveIt 配置和 MuJoCo 资源
│   ├── config/                  # SRDF、IK、OMPL、控制器与关节限位
│   ├── launch/                  # MoveIt/robot_state_publisher 启动文件
│   ├── meshes/                  # 机器人网格模型
│   ├── mujoco/                  # 生成的 URDF、MJB、场景配置和网格
│   ├── scripts/                 # 仿真 bridge、场景生成和关节状态合并
│   └── urdf/                    # ROS/MoveIt 主 URDF
├── asm0003_moveit_config/       # MoveIt 工具兼容配置包
├── web_control/
│   ├── frontend/                # Vite/Three.js 前端源码与 dist
│   └── web_control/             # Python HTTP/ROS 后端及运行配置
├── vision/                      # RealSense/YOLO、目标坐标和场景点云
├── CSP_direct_drive/            # EtherCAT/CSP 真机驱动及 bridge
├── tools/                       # RGB-D、可达性、IK 和抓取研究工具
├── tests/                       # 单元测试和 MuJoCo/VLA smoke tests
├── docs/                        # 专题 API 与算法文档
├── docker/                      # ROS 2 Humble 开发镜像和运行脚本
└── logs/                        # 运行日志与仿真 PID 文件（运行时生成）
```

### 关键文件

| 文件 | 作用 |
| --- | --- |
| `asm0003/urdf/asm0003.urdf` | ROS、TF 和 MoveIt 使用的机器人模型。 |
| `web_control/frontend/public/urdf/unoarm.urdf` | 浏览器显示及 MuJoCo 模型生成的源 URDF。 |
| `asm0003/config/asm0003.srdf` | 左右臂 planning group、自碰撞和语义配置。 |
| `asm0003/config/kinematics.yaml` | MoveIt IK 求解器配置。 |
| `asm0003/config/ompl_planning.yaml` | OMPL 规划器及参数。 |
| `asm0003/config/joint_limits.yaml` | MoveIt 关节速度、加速度和位置限制。 |
| `asm0003/mujoco/scene_config.json` | MuJoCo 相机、桌面、抓取物体等场景配置。 |
| `asm0003/scripts/mujoco_sim_bridge.py` | MuJoCo 与 ROS 2 action/topic/service 桥。 |
| `web_control/web_control/server.py` | Web API、MoveIt 调用和场景管理后端。 |
| `vision/camera_extrinsic.json` | 相机到世界坐标系的外参。 |
| `web_control/web_control/points.json` | 保存的机器人点位。 |
| `web_control/web_control/presets.json` | Web 控制预设。 |
| `web_control/web_control/vlm_config.json` | VLM 服务和提示词配置。 |

## 环境要求

推荐使用仓库提供的容器环境，避免宿主机 ROS、MoveIt、MuJoCo 和 Node.js 版本不一致。

| 组件 | 推荐版本或说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04，或能运行 Docker/Podman 的 Linux 环境 |
| ROS | ROS 2 Humble |
| 运动规划 | MoveIt 2（Humble） |
| Python | Python 3 |
| 仿真 | `mujoco` Python 包 |
| 前端 | Node.js 20、npm、Vite 6 |
| 构建工具 | `colcon`、CMake、ament |
| 容器 | Docker 或 Podman |

容器镜像基于：

```text
docker.io/osrf/ros:humble-desktop
```

镜像还会安装 MoveIt、ros2_control、CycloneDDS、MuJoCo Python 包、Node.js 20 和常用开发工具。`CSP_direct_drive` 依赖真机 EtherCAT SDK/环境，不保证能在通用开发镜像中构建或运行。

## 快速开始：MuJoCo Docker 仿真

### 1. 构建开发镜像

在仓库根目录执行：

```bash
./docker/build-dev.sh
```

脚本会自动选择 Docker 或 Podman。部分环境中的 `docker` 命令实际是 Podman 兼容层，因此优先使用该脚本，不要假设 Docker daemon 一定存在。

### 2. 构建 ROS 包和前端

进入开发容器：

```bash
./docker/run-dev-x11.sh
```

容器内执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config

cd web_control/frontend
npm ci
npm run build
cd ../..
```

仓库通常包含已构建的前端资源，但修改前端或 Web URDF 后必须重新运行 `npm run build`。

### 3. 启动完整仿真栈

退出交互容器后，也可以直接从宿主机启动：

```bash
./start_mujoco_sim.sh --docker
```

无桌面或只使用浏览器时推荐关闭 MuJoCo viewer：

```bash
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh --docker
```

需要本地 Python viewer 时：

```bash
MUJOCO_VIEWER=1 ./start_mujoco_sim.sh --docker
```

脚本会自动完成：

1. 清理 `logs/mujoco_sim_stack.pids` 中记录的旧仿真进程。
2. 检查 Web URDF，并在需要时生成 `asm0003/mujoco/unoarm_mujoco.urdf`。
3. 构建包含相机、工作台和抓取对象的 `asm0003_camera_scene.mjb`。
4. 启动 `mujoco_sim_bridge.py` 并等待双臂 action 与夹爪 service。
5. 启动 MoveIt，等待 `/compute_ik` 和 `/plan_kinematic_path`。
6. 启动 Web 控制服务并等待 `/api/status`。
7. 在前台显示仿真、MoveIt 和 Web 日志。

### 4. 打开页面并验证

```text
http://127.0.0.1:8765
```

接口检查：

```bash
curl -fsS http://127.0.0.1:8765/api/status
curl -fsS http://127.0.0.1:8765/api/robot_state
```

ROS 图检查：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=21
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_NO_DAEMON=1

ros2 node list
ros2 action list | grep follow_joint_trajectory
ros2 service list | grep -E 'compute_ik|plan_kinematic_path|sim_gripper'
```

`ros2 node list` 中不应出现重复的 `/move_group_backend`。重复 MoveIt 节点会使规划请求随机落到错误后端。

### 5. 停止仿真

启动终端中按 `Ctrl+C`，或另开终端执行：

```bash
./stop_mujoco_sim.sh
```

该脚本只停止当前仓库记录的仿真进程和带 UnoArm 标签的仿真容器，不会调用全局 `stop_robot_all.sh`，也不会主动停止真机域中的进程。

## 原生环境运行

宿主机已经安装 Ubuntu 22.04、ROS 2 Humble、MoveIt 和 MuJoCo Python 包时，可以不使用容器：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config
source install/setup.bash
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh
```

前端单独构建：

```bash
cd web_control/frontend
npm ci
npm run build
```

## ROS 域隔离

MuJoCo 与真机必须使用不同 ROS domain。

| 环境 | Domain ID | 默认发现范围 |
| --- | ---: | --- |
| MuJoCo 本地仿真 | `21` | `LOCALHOST` |
| 真机控制 | `25` | `SUBNET` |

`start_mujoco_sim.sh` 使用 `MUJOCO_ROS_DOMAIN_ID`，并将它导出为 `ROS_DOMAIN_ID`。默认同时设置：

```text
ROS_DOMAIN_ID=21
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ROS_LOCALHOST_ONLY=1
ROS_STATIC_PEERS=127.0.0.1
ROS2CLI_NO_DAEMON=1
```

脚本会拒绝让 MuJoCo 使用真机域 `25`。只有明确理解风险时才能覆盖：

```bash
UNOARM_ALLOW_MUJOCO_DOMAIN_25=1 MUJOCO_ROS_DOMAIN_ID=25 ./start_mujoco_sim.sh
```

日常开发不要使用这个覆盖选项。

## MuJoCo 模型、相机与场景

当前默认模型生成链路为：

```text
web_control/frontend/dist/urdf/unoarm.urdf
  → asm0003/scripts/generate_mujoco_urdf.py
  → asm0003/mujoco/unoarm_mujoco.urdf
  → asm0003/scripts/build_mujoco_camera_scene.py
  → asm0003/mujoco/asm0003_camera_scene.mjb
```

源 URDF、生成器、场景配置或抓取模型发生变化时，启动脚本会自动重新生成。也可以强制生成：

```bash
MUJOCO_FORCE_COMPILE=1 ./start_mujoco_sim.sh --docker
```

默认相机：

| 相机 | 默认状态 | RGB | 深度 |
| --- | --- | --- | --- |
| 头部 D455 | 开启 | `640×480 @ 5 Hz` | `32FC1` 米制深度，默认 2 Hz |
| 左腕相机 | 开启 | `480×360 @ 15 Hz` | 默认低频输出 |
| 右腕相机 | 开启 | `480×360 @ 15 Hz` | 默认低频输出 |
| 全局 overview | 关闭 | 默认 2 Hz | 无 |

常用话题：

```text
/mujoco_camera/color/image_raw
/mujoco_camera/depth/image_raw
/mujoco_camera/color/camera_info
/right_wrist_camera/color/image_raw
/right_wrist_camera/depth/image_raw
/left_wrist_camera/color/image_raw
/left_wrist_camera/depth/image_raw
/overview_camera/color/image_raw
```

关闭不需要的渲染可以显著降低资源占用：

```bash
MUJOCO_VIEWER=0 \
MUJOCO_CAMERA_ENABLED=0 \
MUJOCO_WRIST_CAMERA_ENABLED=0 \
MUJOCO_OVERVIEW_CAMERA_ENABLED=0 \
./start_mujoco_sim.sh --docker
```

验证 RGB-D 编码、时间戳、内参和数据长度：

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=21 python3 tools/verify_mujoco_rgbd.py
```

## Web 控制界面

Web 后端默认监听 `0.0.0.0:8765`，同时托管 `web_control/frontend/dist/`。

页面主要区域包括：

- 真实 URDF 三维模型、关节状态、相机模型和场景对象。
- 左右臂选择、关节微动、规划、执行和回零。
- 使能、取消使能、急停与复位（真机使用前先确认安全状态）。
- 左右夹爪控制、目标位姿、approach/grasp/lift/place/pick 流程。
- 点位、预设、动作序列和日志查看。
- 头部/腕部相机画面、点云和视觉目标。
- 平台障碍物、手工碰撞盒、工作空间边界和规划场景。
- OMPL 参数、规划器 benchmark、可达性模型叠加。
- MuJoCo 抓取对象、标定板/标定块和相机外参拟合。
- VLM 配置、图像调用与视觉抓取。

键盘关节微动：

| 关节 | 正向 | 反向 |
| --- | --- | --- |
| J1 | `1` | `Q` |
| J2 | `2` | `W` |
| J3 | `3` | `E` |
| J4 | `4` | `R` |
| J5 | `5` | `T` |
| J6 | `6` | `Y` |
| J7 | `7` | `U` |

`[` 选择左臂，`]` 选择右臂，`Tab` 切换手臂。输入框或文本区域获得焦点时，快捷键自动停用。

## Web API 概览

后端提供的接口较多，以下是常用入口：

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/status` | 服务、ROS、关节和规划状态摘要。 |
| GET | `/api/robot_state` | 使能、急停等机器人状态。 |
| GET | `/api/points` | 已保存点位。 |
| GET | `/api/presets` | 动作预设。 |
| GET | `/api/vision_target` | 当前视觉目标。 |
| GET | `/api/mujoco/scene` | MuJoCo 场景和对象实时状态。 |
| GET | `/api/mujoco/grasp_target` | 仿真抓取目标真值。 |
| GET | `/api/mujoco/visual_grasp_target` | 由 RGB-D 估计的抓取目标。 |
| POST | `/api/plan` | 请求 MoveIt 规划。 |
| POST | `/api/execute` | 执行已规划轨迹。 |
| POST | `/api/joint_jog` | 单关节微动。 |
| POST | `/api/gripper` | 左右夹爪控制。 |
| POST | `/api/pick` | approach、grasp、lift 组合流程。 |
| POST | `/api/sequence` | 执行动作序列。 |
| POST | `/api/platform_obstacle/apply` | 应用平台碰撞体。 |
| POST | `/api/manual_collision_boxes/apply` | 应用手工碰撞盒。 |
| POST | `/api/vlm_call` | 调用已配置的 VLM。 |
| POST | `/api/vlm_grasp` | VLM 视觉抓取流程。 |

MuJoCo VLA 的 16 维关节 chunk 格式、dry-run 和执行示例见 [docs/mujoco_vla_bridge_api.md](docs/mujoco_vla_bridge_api.md)。

## 常用环境变量

### 服务和 ROS

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `WEB_CONTROL_HOST` | `0.0.0.0` | Web 监听地址。 |
| `WEB_CONTROL_PORT` | `8765` | Web 端口。 |
| `MUJOCO_ROS_DOMAIN_ID` | `21` | 仿真专用 ROS domain。 |
| `MUJOCO_ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | 仿真发现范围。 |
| `MUJOCO_ROS_LOCALHOST_ONLY` | `1` | 仅限本机 ROS 通信。 |
| `JOINT_STATE_TOPIC` | `/joint_states` | 合并后的关节状态。 |
| `JOINT_STATE_INPUT_TOPICS` | `/master2/joint_states,/master0/joint_states` | 左右臂输入话题。 |
| `MOVE_GROUP_NAME` | `arm` | 右臂 MoveIt group。 |
| `LEFT_MOVE_GROUP_NAME` | `left_arm` | 左臂 MoveIt group。 |
| `FRAME_ID` | `world` | Web 点位和视觉目标坐标系。 |
| `MOVEIT_PLANNING_FRAME_ID` | `base_link` | MoveIt 请求坐标系。 |

### MuJoCo 与相机

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `MUJOCO_MODEL_PATH` | `asm0003/mujoco/asm0003_camera_scene.mjb` | 自定义 MJB 路径。 |
| `MUJOCO_SOURCE_URDF` | `web_control/frontend/dist/urdf/unoarm.urdf` | 模型生成源文件。 |
| `MUJOCO_VIEWER` | 自动判断 | `1` 打开 viewer，`0` 无窗口运行。 |
| `MUJOCO_PUBLISH_RATE` | `100` | joint state 发布频率。 |
| `MUJOCO_CAMERA_ENABLED` | `1` | 启用头部 RGB-D。 |
| `MUJOCO_CAMERA_WIDTH` | `640` | 头部 RGB 宽度。 |
| `MUJOCO_CAMERA_HEIGHT` | `480` | 头部 RGB 高度。 |
| `MUJOCO_CAMERA_FPS` | `5` | 头部 RGB 帧率。 |
| `MUJOCO_CAMERA_DEPTH_FPS` | `2` | 头部深度帧率。 |
| `MUJOCO_WRIST_CAMERA_ENABLED` | `1` | 启用左右腕部 RGB-D。 |
| `MUJOCO_OVERVIEW_CAMERA_ENABLED` | `0` | 启用全局相机。 |
| `MUJOCO_FORCE_COMPILE` | `0` | 强制重新生成仿真模型。 |

`start_mujoco_sim.sh` 还提供桌面尺寸、目标位置、夹爪速度、抓取接触和对象网格等高级参数。默认值与说明直接记录在脚本开头；修改前建议先保存一次基准抓取结果。

## 单独启动服务

完整仿真优先使用一键脚本。只有定位问题时才建议单独启动组件。

```bash
# Web 服务
./run_web_control.sh

# Web/ROS 就绪检查
./run_web_control.sh --check

# 停止 Web 服务
./run_web_control.sh --stop

# MoveIt（需要 install/setup.bash 和关节/action 后端）
./start_robot_moveit.sh

# 真机 CSP bridge，仿真开发不要运行
./start_csp_driver.sh
```

前端开发服务器：

```bash
cd web_control/frontend
npm ci
npm run dev
```

开发服务器默认监听 `127.0.0.1`，并由 Vite 配置代理 API 和媒体资源。交付前执行：

```bash
npm run build
```

## 日志与运行状态

MuJoCo 栈日志位于 `logs/`：

| 文件 | 内容 |
| --- | --- |
| `logs/mujoco_sim_bridge.log` | 模型加载、物理步进、action、相机和抓取接触。 |
| `logs/moveit.log` | MoveIt、IK、规划器、TF 和 controller 状态。 |
| `logs/web_control.log` | HTTP API、规划请求、执行和场景同步。 |
| `logs/mujoco_sim_stack.pids` | 当前仿真脚本启动的进程清单。 |

常用命令：

```bash
tail -f logs/mujoco_sim_bridge.log logs/moveit.log logs/web_control.log
```

```bash
curl -sS http://127.0.0.1:8765/api/status | python3 -m json.tool
```

## 测试

### 脚本语法

```bash
bash -n start_mujoco_sim.sh
bash -n stop_mujoco_sim.sh
bash -n run_web_control.sh
```

### Python 单元测试

```bash
python3 -m unittest discover -s tests -v
```

### 前端构建

```bash
cd web_control/frontend
npm ci
npm run build
```

### MuJoCo RGB-D smoke test

仿真启动后执行：

```bash
ROS_DOMAIN_ID=21 python3 tools/verify_mujoco_rgbd.py
```

### VLA smoke test

```bash
python3 tests/mujoco_vla_http_smoke.py \
  --base-url http://127.0.0.1:8765
```

### 可达性与规划评估

离线几何筛选：

```bash
python3 tools/reachability_probe.py \
  --config tools/reachability_regions.example.yaml \
  --count 200 \
  --checker geometry \
  --output /tmp/unoarm_reachability.jsonl \
  --summary /tmp/unoarm_reachability_summary.json
```

MoveIt IK 评估需要先启动仿真或真机 ROS 栈：

```bash
python3 tools/reachability_probe.py \
  --config tools/reachability_regions.example.yaml \
  --count 100 \
  --checker moveit_ik \
  --output /tmp/unoarm_ik.jsonl \
  --summary /tmp/unoarm_ik_summary.json
```

更多算法背景见 [docs/reachability_and_grasping_research.md](docs/reachability_and_grasping_research.md)。

## 真机运行与安全边界

> 真机脚本会接触 EtherCAT master 和电机控制。只有在明确需要真机验证、急停可用、工作区无人且零位/限位已核对时运行。

完整真机入口：

```bash
./start_robot_all.sh
```

停止：

```bash
./stop_robot_all.sh
```

命令行菜单：

```bash
./unoarm_menu.sh
```

真机默认使用 `ROS_DOMAIN_ID=25`。不要同时让本地 MuJoCo 加入这个 domain。不要在常规仿真开发中修改电机零位、编码器限位、EtherCAT sudo 配置或 `CSP_direct_drive/` 的控制逻辑。

## 常见问题

### Web 页面打不开

```bash
curl -v http://127.0.0.1:8765/api/status
tail -n 100 logs/web_control.log
```

容器模式会把 `WEB_CONTROL_PORT` 映射到宿主机。如果修改端口，请在启动前设置：

```bash
WEB_CONTROL_PORT=8766 ./start_mujoco_sim.sh --docker
```

### `install/setup.bash not found`

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config
```

### MoveIt service 未就绪

```bash
tail -n 150 logs/moveit.log
```

确认仿真 action server 已先启动，并检查：

```bash
ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 ros2 action list
ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 ros2 service list
```

### 出现多个 `/move_group_backend`

先停止当前仿真：

```bash
./stop_mujoco_sim.sh
```

随后确认没有其他终端或容器使用 `ROS_DOMAIN_ID=21`。不要用全局停止脚本作为常规仿真清理手段。

### 前端修改后页面没有变化

```bash
cd web_control/frontend
npm run build
cd ../..
./run_web_control.sh --stop
./run_web_control.sh
```

浏览器再执行强制刷新。

### MuJoCo 无窗口渲染失败

无窗口模式默认在启用相机时使用 EGL：

```bash
MUJOCO_VIEWER=0 MUJOCO_GL=egl ./start_mujoco_sim.sh --docker
```

检查 `logs/mujoco_sim_bridge.log` 中的 OpenGL/EGL 错误。如果只验证控制链路，可临时关闭所有相机。

### Docker/Podman GUI 无法显示

确认 `DISPLAY` 和 X11 socket 可用。必要时：

```bash
xhost +local:
./docker/run-dev-x11.sh
```

### 机器人或点云位置不正确

依次检查：

1. `vision/camera_extrinsic.json` 的平移和 ZYX 欧拉角。
2. `asm0003/urdf/asm0003.urdf` 与 Web URDF 的安装位姿。
3. `FRAME_ID=world` 与 `MOVEIT_PLANNING_FRAME_ID=base_link` 的转换。
4. 页面中平台障碍物、手工碰撞盒和工作空间边界是否重复启用。

## 开发约定

- 当前分支优先修改 MuJoCo、MoveIt/Web 适配、视觉、工具和文档；不要无关修改真机驱动。
- 修改 `web_control/frontend/` 后运行 `npm run build`，并同步 `dist/`。
- 修改 `asm0003/` 后重新构建对应 ROS 包。
- Shell 脚本保持可执行，并在提交前运行 `bash -n`。
- Git 提交说明使用中文。
- 不提交 `build/`、`log/`、`logs/`、`node_modules/` 等运行或构建产物；仓库已跟踪且项目需要的模型资源除外。
- 真机调试前确认急停、使能状态、编码器零位、软限位和周围安全空间。

## 延伸文档

- [最小使用流程.md](最小使用流程.md)：最短启动和验证路径。
- [docker/README.md](docker/README.md)：容器构建、Podman 差异和 X11 转发。
- [docs/mujoco_vla_bridge_api.md](docs/mujoco_vla_bridge_api.md)：VLA 关节 chunk HTTP API。
- [docs/reachability_and_grasping_research.md](docs/reachability_and_grasping_research.md)：IK、规划器、抓取候选和评估路线。
- [CSP_direct_drive/src/light_test/MOVEIT_BRIDGE.md](CSP_direct_drive/src/light_test/MOVEIT_BRIDGE.md)：真机 CSP/MoveIt bridge 说明。
