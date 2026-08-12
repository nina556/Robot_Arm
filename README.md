# 双臂机器人控制、仿真与视觉作业平台

本项目是一套基于 ROS 2 Humble 的双臂机器人软件栈，面向机械臂运动规划、物理仿真、视觉感知、抓取作业、浏览器控制和真机接入等场景。系统采用分层设计：上层应用通过统一的 Web API 和 ROS 2 接口发起任务，MoveIt 2 负责运动学与路径规划，MuJoCo 或 EtherCAT/CSP 控制链路负责执行，RGB-D 与点云模块提供环境感知能力。

当前开发默认使用本地 MuJoCo 仿真。仿真与真机复用主要 ROS action、service、topic 和 Web API，但不会启动 EtherCAT master 或真机 CSP 驱动。除非明确进行真机调试，否则请使用：

```bash
./start_mujoco_sim.sh --docker
```

启动完成后访问：

```text
http://127.0.0.1:8765
```

## 项目定位

平台将机器人描述、规划、执行、视觉和人机交互放在同一套工程中，主要用于：

- 双 7 自由度机械臂和左右夹爪的联合建模与控制。
- 基于 MuJoCo 的动力学仿真、相机仿真和抓取过程验证。
- 基于 MoveIt 2 的逆运动学、碰撞检测、OMPL 路径规划和轨迹执行。
- RGB-D 图像、点云、目标位姿与工作空间的采集和可视化。
- 点到点运动、关节微动、回零、抓取、放置和动作序列编排。
- 规划器对比、可达性采样、IK 成功率和抓取流程评估。
- 通过 EtherCAT/CSP 桥接真实机器人，并复用仿真阶段验证过的上层接口。
- 为 VLM/VLA、视觉抓取及其他具身智能算法提供统一接入层。

## 总体架构

```text
浏览器控制台
  │  HTTP / JSON / MJPEG
  ▼
Web 控制与任务编排层
  ├── 状态查询、参数管理、点位与动作序列
  ├── 规划、执行、夹爪和急停接口
  ├── 视觉目标、点云与场景对象管理
  └── VLM/VLA 与抓取流程编排
  │  ROS 2 service / action / topic
  ▼
MoveIt 2 规划层
  ├── TF 与 RobotModel
  ├── IK 与状态有效性检查
  ├── OMPL 路径规划
  └── PlanningScene 与碰撞体管理
  │  FollowJointTrajectory
  ├───────────────────────────┐
  ▼                           ▼
MuJoCo 仿真执行层             EtherCAT/CSP 真机执行层
  ├── 关节动力学               ├── 周期同步位置控制
  ├── 夹爪与接触                ├── 编码器和状态反馈
  ├── RGB-D 虚拟相机            └── MoveIt 轨迹桥接
  └── 场景与抓取对象
  │
  ▼
日志、测试、标定与评估工具
```

### 分层职责

| 层级 | 主要职责 | 关键技术 |
| --- | --- | --- |
| 表现层 | 三维模型、状态面板、视频、点云、交互控制 | Vite、JavaScript、Three.js、URDFLoader |
| 服务层 | HTTP API、任务编排、配置持久化、媒体输出 | Python、ROS 2 客户端、JSON、MJPEG |
| 规划层 | IK、规划、碰撞检测、约束和轨迹生成 | MoveIt 2、OMPL、SRDF、KDL/运动学插件 |
| 仿真层 | 物理步进、关节执行、接触、传感器和场景 | MuJoCo、ROS 2 action/service/topic |
| 真机层 | 总线通信、电机状态、CSP 控制和轨迹下发 | EtherCAT、CSP、C++、ros2_control 接口语义 |
| 感知层 | RGB-D、点云、目标坐标、相机外参和视觉推理 | RealSense、OpenCV、点云处理、VLM/VLA |
| 工程支撑 | 构建、容器、日志、自动化测试和故障定位 | Docker/Podman、colcon、CMake、ament、unittest |

## 技术栈

### ROS 2 与通信

系统基于 ROS 2 Humble。连续状态通过 topic 发布，耗时任务和轨迹执行使用 action，同步控制与配置操作使用 service。TF 负责世界坐标系、机器人基座、各关节、末端执行器和相机坐标系之间的变换。

仿真提供与真机兼容的核心控制接口：

```text
/arm_controller/follow_joint_trajectory
/left_arm_controller/follow_joint_trajectory
/master2/joint_states
/master0/joint_states
/master2/joint_trajectory
/master0/joint_trajectory
```

这种接口对齐使上层规划、Web 控制和视觉流程无需感知底层当前连接的是仿真还是真机。

### 运动规划

MoveIt 2 负责载入 URDF/SRDF、构建 RobotModel、求解 IK、维护规划场景并调用 OMPL 生成无碰轨迹。左右臂分别配置 planning group、末端链路、关节限制和控制器映射。系统支持：

- 笛卡尔目标位姿和关节目标规划。
- 多规划器与多次尝试参数配置。
- 平台、手工碰撞盒和环境对象管理。
- 规划前 IK 与状态有效性检查。
- 轨迹预览、执行、取消和执行结果记录。
- 工作空间采样以及规划器成功率、耗时对比。

### MuJoCo 仿真

仿真桥接进程加载编译后的模型，驱动物理步进，并把 MuJoCo 状态转换为 ROS 2 关节状态、相机消息和场景状态。轨迹 action server 接收 MoveIt 生成的轨迹，按时间插值执行目标；夹爪服务负责开合、接触判断和抓取对象约束。

仿真场景可包含工作台、待抓物体、标定板、头部相机、腕部相机和全局观察相机。无显示器环境可以关闭 viewer 并使用 EGL 渲染，也可以关闭不需要的相机以降低 GPU 和 CPU 占用。

### Web 控制

后端同时承担静态资源托管、ROS 2 调用、状态聚合、配置读写和视频流输出。前端显示机器人 URDF、关节状态、末端位姿、轨迹、点云、碰撞体和仿真物体，并提供左右臂切换、点位管理、动作序列、抓取流程与规划器评估界面。

### 视觉与智能接口

视觉层支持头部和腕部 RGB-D 数据、相机内外参、静态场景点云和目标位姿。视觉目标会统一转换到规划坐标系，再进入 IK、预抓取、接近、闭合夹爪、抬升和放置流程。VLM/VLA 接口用于连接外部视觉理解或动作生成服务，并保留 dry-run、输入检查和分段执行能力。

## 目录结构

```text
project/
├── start_mujoco_sim.sh          # 仿真完整栈入口
├── stop_mujoco_sim.sh           # 仿真栈定向停止
├── start_robot_all.sh           # 真机完整栈入口
├── stop_robot_all.sh            # 真机栈停止入口
├── start_robot_moveit.sh        # MoveIt 启动入口
├── start_csp_driver.sh          # CSP 驱动桥接入口
├── run_web_control.sh           # Web 服务启动、检查和停止
├── asm0003/                     # ROS 描述、MoveIt 配置和仿真资源
│   ├── config/                  # SRDF、IK、OMPL、控制器和关节限制
│   ├── launch/                  # ROS 2 与 MoveIt launch 文件
│   ├── meshes/                  # 机器人网格模型
│   ├── mujoco/                  # MuJoCo 模型和场景配置
│   ├── scripts/                 # 仿真桥、模型生成和状态合并
│   └── urdf/                    # ROS/MoveIt 主模型
├── asm0003_moveit_config/       # MoveIt 兼容配置包
├── web_control/
│   ├── frontend/                # Web 前端源码和构建产物
│   └── web_control/             # Python 后端及运行配置
├── vision/                      # RGB-D、目标坐标和场景点云
├── CSP_direct_drive/            # EtherCAT/CSP 驱动与轨迹桥
├── tools/                       # 验证、可达性、IK 和研究工具
├── tests/                       # 单元测试与联调测试
├── docs/                        # API 和算法专题文档
├── docker/                      # 开发镜像与容器脚本
└── logs/                        # 运行时日志和 PID 记录
```

## 环境要求

| 组件 | 版本或说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04，或支持 Docker/Podman 的 Linux |
| ROS | ROS 2 Humble |
| 运动规划 | MoveIt 2 Humble |
| 仿真 | MuJoCo Python 包 |
| Python | Python 3 |
| 前端 | Node.js 20、npm、Vite 6 |
| 构建 | colcon、CMake、ament |
| 容器 | Docker 或 Podman |

容器镜像基于 `docker.io/osrf/ros:humble-desktop`，并补充 MoveIt、ros2_control、CycloneDDS、MuJoCo、Node.js 和常用开发工具。真机 EtherCAT SDK 依赖具体设备环境，不保证能在通用开发镜像中运行。

## Windows、WSL 2 与 Docker Desktop

Windows 用户建议在 WSL 2 的 Linux 文件系统中保存和构建项目，并由 Windows 上的 Docker Desktop 提供容器引擎。不要另外在 WSL 中安装一套独立的 Docker Engine，否则两套服务、上下文和 socket 容易发生冲突。

### 1. Windows 侧准备

在管理员 PowerShell 中检查并更新 WSL：

```powershell
wsl --status
wsl --update
wsl -l -v
```

目标 Linux 发行版的 `VERSION` 应为 `2`。如仍为 WSL 1，执行：

```powershell
wsl --set-version Ubuntu-22.04 2
wsl --set-default Ubuntu-22.04
```

发行版名称以 `wsl -l -v` 的实际输出为准。

安装并启动 Docker Desktop，然后完成以下设置：

1. 在 `Settings > General` 启用 `Use the WSL 2 based engine`；部分新版本默认启用且不显示该选项。
2. 在 `Settings > Resources > WSL Integration` 中打开目标 Linux 发行版。
3. 确认 Docker Desktop 当前使用 Linux containers。
4. 点击 `Apply & restart`，等待容器引擎启动完成。

### 2. WSL 侧验证 Docker 连接

打开 Ubuntu 终端后执行：

```bash
docker version
docker context show
docker run --rm hello-world
```

`docker version` 应同时显示 Client 和 Server。若只有 Client 或提示无法连接 daemon，先确认 Docker Desktop 正在运行，并重新检查该发行版的 WSL Integration 开关。必要时在 PowerShell 中执行 `wsl --shutdown`，重新打开 Docker Desktop 和 Ubuntu 后再测试。

### 3. 保存和打开项目

Linux 构建工具在 WSL 自身文件系统中运行更快。推荐把仓库放在类似 `~/projects/robot-platform` 的目录，不建议放在 `/mnt/c/` 或 `/mnt/d/` 下执行 colcon、npm 和大量模型生成任务。

在 WSL 中进入当前仓库并用 VS Code 打开：

```bash
cd <仓库目录>
code .
```

只打开本文档：

```bash
cd <仓库目录>
code README.md
```

用 Windows 文件资源管理器打开当前目录：

```bash
cd <仓库目录>
explorer.exe .
```

VS Code 需要安装 WSL 扩展。窗口左下角应显示当前连接的 WSL 发行版，终端也应运行在 Linux 环境中。

### 4. 在 WSL 中构建并启动

以下命令均在 WSL 终端执行，实际容器由 Windows Docker Desktop 运行：

```bash
cd <仓库目录>
docker info
./docker/build-dev.sh
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh --docker
```

现有容器脚本会把 Web 端口显式映射到宿主机，不依赖 Docker Desktop 下行为不同的 Linux host network。启动后直接用 Windows 浏览器打开：

```text
http://127.0.0.1:8765
```

也可以从 WSL 调用 Windows 默认浏览器：

```bash
explorer.exe http://127.0.0.1:8765
```

首次运行建议保持 `MUJOCO_VIEWER=0`，通过浏览器查看三维界面。需要桌面 viewer 时再确认 Windows 已启用 WSLg、WSL 中存在 `DISPLAY`，然后执行：

```bash
MUJOCO_VIEWER=1 ./start_mujoco_sim.sh --docker
```

### 5. WSL 常见问题

- `docker: command not found`：目标发行版没有启用 Docker Desktop 的 WSL Integration，或需要重启 WSL。
- `Cannot connect to the Docker daemon`：确认 Docker Desktop 已启动并处于 Linux containers 模式。
- 端口 `8765` 被占用：先运行 `./stop_mujoco_sim.sh`，再用 `docker ps` 检查残留容器。
- 构建或 npm 安装很慢：确认仓库不在 `/mnt/c/`、`/mnt/d/` 等 Windows 挂载目录。
- 脚本没有执行权限：在 WSL 中运行 `chmod +x docker/*.sh *.sh`。
- Windows 浏览器无法访问：用 `docker ps` 确认存在 `0.0.0.0:8765->8765/tcp` 端口映射，再检查 Windows 防火墙或安全软件。
- WSL 或容器占用内存未及时释放：停止仿真后，可在 PowerShell 中执行 `wsl --shutdown`；这会同时终止所有正在运行的 WSL 发行版。

## 快速开始

### 1. 构建开发镜像

```bash
./docker/build-dev.sh
```

脚本会自动选择 Docker 或 Podman。若需要图形界面，确保宿主机 X11 可用。

### 2. 构建 ROS 包与前端

进入容器：

```bash
./docker/run-dev-x11.sh
```

在容器中执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config

cd web_control/frontend
npm ci
npm run build
cd ../..
```

修改前端源码或浏览器使用的机器人模型后，需要重新运行 `npm run build`。

### 3. 启动仿真栈

```bash
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh --docker
```

启动器依次完成以下工作：

1. 清理 PID 文件中记录的旧仿真进程。
2. 检查模型源文件并按需重新生成 MuJoCo 模型。
3. 编译包含相机、工作台和抓取对象的场景。
4. 启动仿真桥并等待双臂轨迹 action 与夹爪 service。
5. 启动 MoveIt 并等待 IK 与规划服务。
6. 启动 Web 服务并检查状态接口。
7. 在前台汇总关键日志，直到收到 `Ctrl+C`。

如需本地 MuJoCo viewer：

```bash
MUJOCO_VIEWER=1 ./start_mujoco_sim.sh --docker
```

### 4. 验证服务

```bash
curl -fsS http://127.0.0.1:8765/api/status
curl -fsS http://127.0.0.1:8765/api/robot_state
```

检查 ROS 图：

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

节点列表中不应出现重复的 `/move_group_backend`。重复规划节点可能让请求落到错误后端。

### 5. 停止仿真

在启动终端按 `Ctrl+C`，或执行：

```bash
./stop_mujoco_sim.sh
```

停止脚本只处理当前项目记录的仿真进程与容器，不会主动停止真机控制域中的进程。

## 原生环境运行

宿主机已安装完整依赖时，可以不使用容器：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config
source install/setup.bash
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh
```

## ROS 域隔离

| 环境 | Domain ID | 发现范围 |
| --- | ---: | --- |
| 本地仿真 | `21` | `LOCALHOST` |
| 真机控制 | `25` | `SUBNET` |

仿真启动器默认设置：

```text
ROS_DOMAIN_ID=21
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ROS_LOCALHOST_ONLY=1
ROS_STATIC_PEERS=127.0.0.1
ROS2CLI_NO_DAEMON=1
```

不要让常规仿真加入真机域。域隔离可以避免旧节点、其他开发者的节点或真机控制器接收到不属于它们的命令。

## 相机与场景

| 相机 | 默认状态 | RGB | 深度 |
| --- | --- | --- | --- |
| 头部 RGB-D | 开启 | `640×480 @ 5 Hz` | `32FC1` 米制深度，默认 2 Hz |
| 左腕相机 | 开启 | `480×360 @ 15 Hz` | 低频输出 |
| 右腕相机 | 开启 | `480×360 @ 15 Hz` | 低频输出 |
| 全局观察相机 | 关闭 | 默认 2 Hz | 无 |

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

关闭不需要的渲染：

```bash
MUJOCO_VIEWER=0 \
MUJOCO_CAMERA_ENABLED=0 \
MUJOCO_WRIST_CAMERA_ENABLED=0 \
MUJOCO_OVERVIEW_CAMERA_ENABLED=0 \
./start_mujoco_sim.sh --docker
```

验证 RGB-D 编码、时间戳、内参和数据长度：

```bash
ROS_DOMAIN_ID=21 python3 tools/verify_mujoco_rgbd.py
```

## Web 使用流程

1. 打开 `http://127.0.0.1:8765`，确认服务状态为在线。
2. 检查左右臂关节状态、末端位姿和三维模型是否一致。
3. 选择操作手臂，使用小步长关节微动确认方向和限制。
4. 输入目标位姿，先执行规划并检查轨迹、碰撞体和工作空间。
5. 确认安全后执行轨迹；仿真中观察接触，真机中保持急停可达。
6. 抓取任务先验证视觉目标和坐标系，再依次执行预抓取、接近、夹持、抬升和放置。
7. 将稳定动作保存为点位、规划预设或动作序列，以便重复使用。
8. 通过日志和 benchmark 记录成功率、耗时、失败阶段及参数。

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

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/status` | 服务、ROS、关节和规划状态摘要 |
| GET | `/api/robot_state` | 使能、急停等机器人状态 |
| GET | `/api/points` | 已保存点位 |
| GET | `/api/presets` | 动作和规划预设 |
| GET | `/api/vision_target` | 当前视觉目标 |
| GET | `/api/mujoco/scene` | 仿真场景实时状态 |
| GET | `/api/mujoco/grasp_target` | 仿真抓取目标真值 |
| POST | `/api/plan` | 请求 MoveIt 规划 |
| POST | `/api/execute` | 执行已规划轨迹 |
| POST | `/api/joint_jog` | 单关节微动 |
| POST | `/api/gripper` | 左右夹爪控制 |
| POST | `/api/pick` | 接近、夹持和抬升组合流程 |
| POST | `/api/sequence` | 执行动作序列 |
| POST | `/api/platform_obstacle/apply` | 应用平台碰撞体 |
| POST | `/api/manual_collision_boxes/apply` | 应用手工碰撞盒 |
| POST | `/api/vlm_call` | 调用视觉语言模型 |
| POST | `/api/vlm_grasp` | 执行视觉抓取流程 |

更完整的 VLA 关节数据格式、dry-run 和执行示例见 [VLA 桥接接口文档](docs/mujoco_vla_bridge_api.md)。

## 支撑能力综述

### 模型与配置支撑

URDF 描述机器人连杆、关节、惯量、碰撞几何和可视网格；SRDF 描述规划组、末端执行器和允许碰撞关系；YAML 文件集中管理运动学插件、OMPL 规划器、控制器映射和关节限制。模型生成脚本把浏览器使用的模型转换为 MuJoCo 可加载格式，并叠加场景与传感器配置。

### 开发与交付支撑

容器固定 ROS、MoveIt、MuJoCo 和 Node.js 依赖，降低宿主机差异。ROS 包采用 colcon 与 ament 构建，前端采用 npm 与 Vite 构建。开发时可使用源码软链接安装，前端交付时生成静态资源，由 Python 后端统一托管。

### 运行与可观测性支撑

启动脚本按依赖顺序拉起仿真、规划和 Web 服务，并在每一步执行就绪检查。运行日志按组件拆分，PID 文件记录本次启动的进程，停止脚本据此精确清理。状态接口可用于人工检查、脚本探活和上层监控。

| 日志 | 内容 |
| --- | --- |
| `logs/mujoco_sim_bridge.log` | 模型加载、物理步进、轨迹、相机和接触 |
| `logs/moveit.log` | IK、规划器、TF、碰撞和控制器状态 |
| `logs/web_control.log` | HTTP 请求、任务编排、执行与场景同步 |
| `logs/mujoco_sim_stack.pids` | 当前仿真栈的进程记录 |

### 算法评估支撑

工具目录提供离线几何筛选、MoveIt IK 检查、OMPL 规划评估、随机采样、工作空间边界生成和 RGB-D 验证。评估时建议同时记录采样区域、随机种子、关节限制、规划器、超时、尝试次数、成功率和平均耗时，保证结果可复现。

### 测试支撑

```bash
# Shell 语法
bash -n start_mujoco_sim.sh
bash -n stop_mujoco_sim.sh
bash -n run_web_control.sh

# Python 单元测试
python3 -m unittest discover -s tests -v

# 前端构建
cd web_control/frontend
npm ci
npm run build
```

仿真启动后可执行：

```bash
ROS_DOMAIN_ID=21 python3 tools/verify_mujoco_rgbd.py
python3 tests/mujoco_vla_http_smoke.py --base-url http://127.0.0.1:8765
```

## 单独启动组件

完整仿真优先使用一键启动器。只有定位问题时才建议拆分启动：

```bash
./run_web_control.sh
./run_web_control.sh --check
./run_web_control.sh --stop
./start_robot_moveit.sh
```

真机 CSP 驱动仅在明确需要真机联调时启动：

```bash
./start_csp_driver.sh
```

## 真机运行与安全边界

> 真机链路会操作 EtherCAT master 和电机。运行前必须确认急停有效、工作区无人、工具安装牢固，并核对编码器零位、关节软限位、运动方向和速度限制。

启动完整真机栈：

```bash
./start_robot_all.sh
```

停止：

```bash
./stop_robot_all.sh
```

真机默认使用 `ROS_DOMAIN_ID=25`。不要让本地仿真加入该域，也不要在常规仿真开发中修改电机零位、EtherCAT 权限或底层控制逻辑。

建议按照以下顺序验收真机链路：

1. 不使能电机，检查总线、编码器和急停状态。
2. 单臂低速使能，逐关节核对正负方向和软限位。
3. 验证关节状态、TF 和 Web 三维模型一致。
4. 执行短距离、低速、无障碍轨迹。
5. 再启用双臂规划、视觉抓取和动作序列。

## 常见问题

### 页面打不开

```bash
curl -v http://127.0.0.1:8765/api/status
tail -n 100 logs/web_control.log
```

### ROS 构建环境未生成

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config
```

### MoveIt 服务未就绪

```bash
tail -n 150 logs/moveit.log
ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 ros2 action list
ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=1 ROS2CLI_NO_DAEMON=1 ros2 service list
```

确认仿真轨迹 action server 已先启动，并检查是否存在重复规划节点。

### 无窗口渲染失败

```bash
MUJOCO_VIEWER=0 MUJOCO_GL=egl ./start_mujoco_sim.sh --docker
```

检查仿真日志中的 OpenGL/EGL 错误。只验证控制链路时，可以临时关闭全部相机。

### 前端修改后未生效

```bash
cd web_control/frontend
npm run build
cd ../..
./run_web_control.sh --stop
./run_web_control.sh
```

随后在浏览器中强制刷新。

### 模型、机器人或点云位置不一致

依次检查：

1. 相机外参中的平移和欧拉角定义。
2. ROS 主模型与浏览器模型的安装位姿。
3. `world`、`base_link`、末端和相机坐标系的 TF。
4. 平台障碍物、手工碰撞盒和工作空间边界是否重复启用。
5. 深度单位、相机内参和时间戳是否一致。

## 开发约定

- 当前分支优先修改 MuJoCo、MoveIt/Web 适配、视觉、工具和文档。
- 修改前端后运行 `npm run build`，并同步交付资源。
- 修改 ROS 描述或配置后重新构建对应包。
- Shell 脚本保持可执行，提交前运行 `bash -n`。
- Git 提交说明使用中文。
- 不提交运行日志、缓存、依赖目录和临时构建产物。
- 真机调试前完成急停、零位、限位、方向和安全空间检查。

## 延伸文档

- [最小使用流程](最小使用流程.md)
- [容器开发环境](docker/README.md)
- [VLA 桥接接口](docs/mujoco_vla_bridge_api.md)
- [可达性与抓取研究](docs/reachability_and_grasping_research.md)
- [真机轨迹桥说明](CSP_direct_drive/src/light_test/MOVEIT_BRIDGE.md)
