# UnoArm URDF 控制系统

这是 UnoArm 双臂机器人控制项目的 URDF 重构版。项目保留原有机器人驱动、MoveIt、视觉识别、点云和 Web API 能力，浏览器界面改为使用 Vite + Three.js + URDFLoader 渲染真实 URDF 模型。

新版页面不再使用简化连杆模型，而是直接加载双臂、夹爪、底盘、头部相机安装座、RealSense D455、相机角铁连接件等 URDF/STL 资源。网页同时提供机器人状态、双臂使能、取消使能、急停复位、回零、点位管理、目标识别、点云显示、避障区显示和规划场景应用等功能。

本文档中的路径默认都以项目根目录为基准，例如 `./start_robot_all.sh`、`vision/target_xyz.tsv`、`web_control/frontend`。

## 项目结构

```text
unoarm-urdf/
├── README.md
├── AGENTS.md
├── 最小使用流程.md
├── start_robot_all.sh
├── start_mujoco_sim.sh
├── stop_mujoco_sim.sh
├── stop_robot_all.sh
├── start_csp_driver.sh
├── start_robot_moveit.sh
├── run_web_control.sh
├── unoarm_menu.sh
├── docker/
│   ├── build-dev.sh
│   ├── run-dev-x11.sh
│   └── unoarm-dev.Dockerfile
├── docker-compose.dev.yml
├── asm0003/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   ├── launch/
│   ├── meshes/
│   ├── scripts/
│   └── urdf/
├── CSP_direct_drive/
│   ├── start_moveit_bridge.sh
│   ├── master2_control.sh
│   ├── master2_control_menu.py
│   └── src/light_test/
├── vision/
│   ├── exp.pt
│   ├── camera_extrinsic.json
│   ├── index.html
│   ├── start_vision_target_writer.sh
│   └── vision_target_writer.py
├── web_control/
│   ├── frontend/
│   │   ├── package.json
│   │   ├── vite.config.js
│   │   ├── public/urdf/
│   │   ├── src/
│   │   └── dist/
│   └── web_control/
│       ├── server.py
│       ├── start_web_control.sh
│       ├── points.json
│       ├── presets.json
│       └── fastdds_no_shm.xml
└── tests/
    └── test_project_structure.py
```

## 各目录作用

| 目录或文件 | 作用 |
| --- | --- |
| `asm0003/` | ROS 2 机器人描述包，包含 URDF、MoveIt 配置、网格模型、启动文件和 joint state 合并脚本。 |
| `asm0003/urdf/asm0003.urdf` | MoveIt 和机器人状态发布使用的主 URDF。 |
| `asm0003/config/` | SRDF、关节限位、运动学、控制器和规划参数。 |
| `CSP_direct_drive/` | EtherCAT CSP 驱动、编码器配置、左右臂 joint state 发布和 MoveIt bridge。 |
| `CSP_direct_drive/src/light_test/config/` | 左右臂电机、编码器、零点、限位和控制器相关配置。 |
| `vision/` | RealSense + YOLO 视觉服务，输出目标坐标、MJPEG 预览和场景点云。 |
| `web_control/frontend/` | 新版网页前端，使用真实 URDF 模型渲染机器人。 |
| `web_control/frontend/public/urdf/` | 浏览器端加载的 URDF 和 STL 资源。 |
| `web_control/web_control/` | Python + ROS 2 Web 控制服务，负责 HTTP API、MoveIt 调用、轨迹发送和规划场景管理。 |
| `tests/` | 项目结构和关键文件检查。 |
| `start_robot_all.sh` | 一键启动视觉、CSP、MoveIt、Web 控制。 |
| `start_mujoco_sim.sh` | 一键启动 MuJoCo 仿真、MoveIt、Web 控制，不启动真机 EtherCAT/CSP。 |
| `stop_mujoco_sim.sh` | 只停止由 MuJoCo 仿真脚本记录的本地仿真进程。 |
| `stop_robot_all.sh` | 停止视觉、MoveIt、Web 控制和相关进程。 |

## 系统组成

整体运行链路如下：

```text
浏览器页面
  ↓ HTTP/Web API
web_control/web_control/server.py
  ↓ ROS 2 service/action/topic
MoveIt + robot_state_publisher + joint_state_merger
  ↓ FollowJointTrajectory action
CSP_direct_drive / light_test
  ↓ EtherCAT
左右臂电机驱动器
```

视觉链路如下：

```text
RealSense D455
  ↓ 彩色图 + 深度图
vision/vision_target_writer.py
  ↓ 目标坐标 / 点云 / MJPEG
web_control 页面显示目标、点云和避障区
```

## 运行环境

机器人端建议环境：

| 项目 | 要求 |
| --- | --- |
| 系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| Python | Python 3 |
| 构建工具 | `colcon` |
| 前端工具 | Node.js、npm |
| 相机 | Intel RealSense D455 |
| 通信 | EtherCAT Master0 / Master2 |
| ROS Domain | 默认 `ROS_DOMAIN_ID=25` |

MuJoCo 仿真入口会忽略外层 `ROS_DOMAIN_ID`，默认使用 `MUJOCO_ROS_DOMAIN_ID=21` 并导出为 `ROS_DOMAIN_ID=21`。仿真同时默认只做本机发现，避免连到真机 `ROS_DOMAIN_ID=25`。仿真启动/退出只清理 `logs/mujoco_sim_stack.pids` 里记录的进程，不自动调用全局 `stop_robot_all.sh`。

开发和调试时先区分运行目标：

| 目标 | 启动脚本 | 是否接触真机驱动 |
| --- | --- | --- |
| MuJoCo 仿真 | `./start_mujoco_sim.sh --docker` | 否 |
| 真机机器人 | `./start_robot_all.sh` | 是 |

当前分支如果只做 MuJoCo 开发，默认在本地 Docker/Podman 容器里启动仿真，不连接远端机器人，也不运行真机启动脚本。

## Docker/Podman 开发环境

本地非 Ubuntu 22.04 环境可以使用 Docker/Podman 开发容器。镜像基于 OSRF 的 ROS 2 Humble desktop：

```text
docker.io/osrf/ros:humble-desktop
```

构建镜像：

```bash
./docker/build-dev.sh
```

进入容器：

```bash
./docker/run-dev-x11.sh
```

容器内构建 MoveIt/URDF 包：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003
```

当前开发机的 `docker` 命令是 Podman 兼容层，不是 Docker daemon。不要直接使用 `docker compose ... build`，优先使用 `./docker/build-dev.sh`。

常用端口：

| 服务 | 默认地址 |
| --- | --- |
| Web 控制页面 | `http://192.168.10.200:8765` |
| 视觉预览页面 | `http://192.168.10.200:8090` |

## 开发提交约定

- 提交代码时，`git commit` 的提交说明使用中文。
- push 或同步代码时，上传说明和变更备注也使用中文描述。
- 前端源码变更后同步执行 `npm run build`，并提交对应的 `web_control/frontend/dist/` 构建产物。

## 第一次构建

进入项目根目录后执行。

### 1. 构建机器人描述和 MoveIt 包

```bash
colcon build --symlink-install --packages-select asm0003
```

构建完成后，项目根目录下会生成：

```text
build/
install/
log/
```

### 2. 构建 CSP 驱动包

```bash
cd CSP_direct_drive
colcon build --symlink-install --packages-select light_test
cd ..
```

构建完成后，`CSP_direct_drive/` 下会生成：

```text
CSP_direct_drive/build/
CSP_direct_drive/install/
CSP_direct_drive/log/
```

### 3. 构建前端页面

```bash
cd web_control/frontend
npm ci
npm run build
cd ../..
```

构建结果会输出到：

```text
web_control/frontend/dist/
```

Python Web 服务会优先托管这个目录。

### 4. 检查脚本权限

如果脚本没有执行权限，执行：

```bash
chmod +x *.sh
chmod +x vision/start_vision_target_writer.sh
chmod +x web_control/web_control/start_web_control.sh
chmod +x CSP_direct_drive/start_moveit_bridge.sh
```

## 日常启动

进入项目根目录后执行：

```bash
./start_robot_all.sh
```

脚本会按顺序处理：

1. 检查 sudo 权限。
2. 重启 EtherCAT Master0 / Master2。
3. 停止旧的机器人相关进程。
4. 启动视觉目标输出和视觉预览。
5. 启动 CSP 驱动和 MoveIt bridge。
6. 启动 MoveIt。
7. 启动 Web 控制服务。
8. 前台跟踪输出日志。

启动成功后，浏览器打开：

```text
http://192.168.10.200:8765
```

视觉预览页面：

```text
http://192.168.10.200:8090
```

## MuJoCo 仿真模式

MuJoCo 仿真复用 Web 控制、MoveIt 规划和现有 ROS 接口，但不启动 EtherCAT / CSP 真机驱动：

```bash
./start_mujoco_sim.sh --docker
```

如果已经在 Ubuntu 22.04 / ROS 2 Humble 环境或开发容器内部，也可以直接运行：

```bash
./start_mujoco_sim.sh
```

脚本启动顺序：

1. 按 `logs/mujoco_sim_stack.pids` 清理上一次由仿真脚本启动的本地进程。
2. 启动 `asm0003/scripts/mujoco_sim_bridge.py`。
3. 等待左右臂 FollowJointTrajectory action ready。
4. 启动 `start_robot_moveit.sh`。
5. 启动 `run_web_control.sh`。
6. 前台跟踪 `logs/mujoco_sim_bridge.log`、`logs/moveit.log`、`logs/web_control.log`。

仿真后端会提供和真机一致的接口：

```text
/arm_controller/follow_joint_trajectory
/left_arm_controller/follow_joint_trajectory
/master2/joint_states
/master0/joint_states
/master2/joint_trajectory
/master0/joint_trajectory
```

默认模型是 `asm0003/mujoco/asm0003_rotated.mjb`。如果该文件不存在，脚本会尝试用 `/home/guanghe-sun/Downloads/mujoco-3.10.0/bin/compile` 从 `asm0003/mujoco/asm0003_rotated.urdf` 自动生成。

常用环境变量：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `MUJOCO_MODEL_PATH` | `asm0003/mujoco/asm0003_rotated.mjb` | MuJoCo 二进制模型路径。 |
| `MUJOCO_MODEL_URDF` | `asm0003/mujoco/asm0003_rotated.urdf` | 自动编译 `.mjb` 时使用的 URDF。 |
| `MUJOCO_COMPILE` | `/home/guanghe-sun/Downloads/mujoco-3.10.0/bin/compile` | MuJoCo compile 工具路径。 |
| `MUJOCO_VIEWER` | 自动判断 | `1` 打开 viewer，`0` 禁用 viewer。 |
| `MUJOCO_PUBLISH_RATE` | `100` | 仿真 joint state 发布频率。 |
| `MUJOCO_ROS_DOMAIN_ID` | `21` | MuJoCo 仿真专用 ROS 2 Domain ID，脚本会导出为 `ROS_DOMAIN_ID`。 |
| `MUJOCO_ROS_AUTOMATIC_DISCOVERY_RANGE` | `LOCALHOST` | MuJoCo 仿真默认只发现本机 ROS 节点。 |
| `MUJOCO_ROS_LOCALHOST_ONLY` | `1` | MuJoCo 仿真默认启用 localhost-only 隔离。 |
| `MUJOCO_ROS_STATIC_PEERS` | `127.0.0.1` | MuJoCo 仿真默认 DDS 静态 peer。 |

打开 MuJoCo Python viewer：

```bash
MUJOCO_VIEWER=1 ./start_mujoco_sim.sh --docker
```

有桌面显示环境时，`./start_mujoco_sim.sh` 会默认打开 MuJoCo viewer。如需只运行网页仿真而不弹出 MuJoCo 窗口：

```bash
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh --docker
```

只调试网页和接口时建议用无窗口模式：

```bash
MUJOCO_VIEWER=0 ./start_mujoco_sim.sh --docker
```

启动后检查：

```bash
curl http://127.0.0.1:8765/api/status
ROS_DOMAIN_ID=21 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1 ros2 node list
ROS_DOMAIN_ID=21 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1 ros2 action list | grep follow_joint_trajectory
```

如果 `ros2 node list` 出现多个 `/move_group_backend`，说明同一个 ROS domain 中有旧 MoveIt 或其它仿真栈。先运行：

```bash
./stop_mujoco_sim.sh
```

如果停止后仍能看到重复节点，检查是否有其它本机终端或容器使用同一个 `ROS_DOMAIN_ID=21`。重复 move_group 会让规划请求落到错误后端，表现为日志中出现与当前配置不一致的规划器信息，例如 CHOMP 报错。

如果需要调整视觉预览参数，可以在启动前传环境变量：

```bash
VISION_STREAM_WIDTH=848 VISION_JPEG_QUALITY=80 VISION_WEB_PORT=8090 ./start_robot_all.sh
```

如果脚本在前台运行，按 `Ctrl+C` 会调用停止脚本并退出。

## 停止服务

MuJoCo 仿真在启动窗口按 `Ctrl+C`，或者在项目根目录运行：

```bash
./stop_mujoco_sim.sh
```

真机/全局机器人栈需要停止时运行：

```bash
./stop_robot_all.sh
```

全局停止脚本会清理以下进程：

| 进程类型 | 说明 |
| --- | --- |
| `vision_target_writer.py` | 视觉目标输出和视觉预览 |
| `server.py` | Web 控制服务 |
| `ros2 launch asm0003 ...` | MoveIt 启动进程 |
| `move_group` | MoveIt 规划节点 |
| `joint_state_merger.py` | 左右臂 joint state 合并 |
| `robot_state_publisher` | URDF 状态发布 |
| `master2_moveit_bridge` | CSP 到 MoveIt 的控制桥 |
| `mujoco_sim_bridge.py` | MuJoCo 仿真桥 |

## 单独启动某个服务

### 单独启动 Web 控制

```bash
./run_web_control.sh
```

仅启动服务端，不额外包装：

```bash
./run_web_control.sh --server-only
```

检查 ROS 图是否就绪：

```bash
./run_web_control.sh --check
```

停止 Web 控制：

```bash
./run_web_control.sh --stop
```

### 单独启动 MoveIt

```bash
./start_robot_moveit.sh
```

这个脚本会加载 `install/setup.bash`，并启动：

```text
asm0003/launch/web_control_moveit.launch.py
```

MuJoCo 开发时通常不单独启动 MoveIt，而是让 `./start_mujoco_sim.sh` 负责启动顺序，避免 MoveIt 先于仿真 action server ready。

### 单独启动 CSP 驱动

```bash
./start_csp_driver.sh
```

这个脚本会调用：

```text
CSP_direct_drive/start_moveit_bridge.sh
```

### 单独启动视觉服务

```bash
./vision/start_vision_target_writer.sh
```

如果希望视觉文件全部落在项目内，可以显式指定相对路径：

```bash
./vision/start_vision_target_writer.sh \
  --model vision/exp.pt \
  --output vision/target_xyz.tsv \
  --scene-pointcloud vision/scene_pointcloud.bin \
  --extrinsic-file vision/camera_extrinsic.json \
  --web-host 0.0.0.0 \
  --web-port 8090
```

相机外参使用标准 ZYX 欧拉角约定：

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
world_point = R * optical_point + [x, y, z]
```

其中光学坐标为 X 向右、Y 向下、Z 向前，角度单位为弧度。新配置应包含
`"rpy_convention": "standard_zyx"`。缺少该字段的旧配置会按历史上互换
`roll/pitch` 的格式读取并自动转换；再次通过 Web 页面保存后会写成标准格式。
`image_rotation_deg` 只用于确实需要旋转输入图像的相机，不再用于补偿欧拉角字段约定。

## Web 页面功能

### URDF 显示

网页会加载：

```text
web_control/frontend/public/urdf/unoarm.urdf
```

以及对应网格：

```text
web_control/frontend/public/urdf/meshes/
```

前端构建后，资源会复制到：

```text
web_control/frontend/dist/urdf/
```

页面显示内容包括：

| 内容 | 说明 |
| --- | --- |
| 双臂 URDF | 左臂、右臂、底座、关节、夹爪。 |
| 相机结构 | 头部相机安装座、角铁连接件、RealSense D455。 |
| 地面和坐标 | 用于观察模型姿态和点云位置。 |
| 点云 | 可显示视觉服务输出的当前场景点云。 |
| 避障区 | 可根据场景点云生成平台避障区，并应用到 MoveIt 规划场景。 |

### 机器人控制按钮

按钮会根据实时状态改变文字和颜色。

| 按钮 | 状态逻辑 |
| --- | --- |
| `双臂使能` / `已全部使能` | 未使能时显示普通状态，全部使能后显示绿色高亮。 |
| `取消使能` / `未使能` | 有电机使能时显示可点击状态；全部未使能时显示灰色。 |
| `复位急停` / `急停正常` | 有急停状态时显示警告色；正常时显示 `急停正常`。 |
| `双臂回零` | 在机器人已使能且无急停时可用。 |
| `显示点云` / `隐藏点云` | 点云显示后按钮进入高亮状态。 |
| `显示避障区` / `隐藏避障区` | 控制页面中避障区几何体是否显示。 |
| `应用避障区` / `取消避障区` | 控制 MoveIt 规划场景中是否存在避障区。 |

### 左右臂控制

页面支持左右臂切换控制。两个手臂不会同时响应同一组关节键盘控制。

- `[` 选择左臂，`]` 选择右臂，`Tab` 在左右臂之间切换。
- 键盘事件在输入框、下拉框和文本框获得焦点时自动停用，避免编辑参数时误动作。
- 关节状态区域的按键也可用鼠标点击；机械臂未使能或处于急停状态时，这些按键不可用。
- 单次微动默认是 `0.8°`，按住按键可以连续微动。后端限制单次请求的最大角度，并由 CSP 驱动按照当前编码器限位再次校验。

键盘控制规则：

| 关节 | 正向 | 反向 |
| --- | --- | --- |
| 关节 1 | `1` | `Q` |
| 关节 2 | `2` | `W` |
| 关节 3 | `3` | `E` |
| 关节 4 | `4` | `R` |
| 关节 5 | `5` | `T` |
| 关节 6 | `6` | `Y` |
| 关节 7 | `7` | `U` |

如果后续 URDF 增加更多可动关节，页面应继续按照键盘上方数字键和对应字母键的规律扩展。

## 控制菜单

`unoarm_menu.sh` 是命令行控制入口，适合不用网页时快速操作机器人。

进入项目根目录后运行：

```bash
./unoarm_menu.sh
```

按键说明：

| 按键 | 功能 |
| --- | --- |
| `e` | 双臂使能电机，需要短时间内再次确认。 |
| `d` | 双臂正常停止并取消使能。 |
| `空格` | 双臂紧急快速停止。 |
| `r` | 双臂复位急停。 |
| `s` | 查看当前状态。 |
| `h` | 双臂回零，需要短时间内再次确认。 |
| `1` 到 `6` | 设置速度倍率。 |
| `[` / `]` | 降低或提高速度倍率。 |
| `q` | 退出菜单。 |

也可以直接使用命令行参数：

```bash
./unoarm_menu.sh status
./unoarm_menu.sh enable
./unoarm_menu.sh disable
./unoarm_menu.sh estop
./unoarm_menu.sh reset
```

## 前端开发

进入前端目录：

```bash
cd web_control/frontend
```

安装依赖：

```bash
npm ci
```

启动开发服务器：

```bash
npm run dev
```

开发服务器会把以下资源代理到机器人 Web 服务：

```text
/api
/snapshot.ply
/vision_stream.mjpg
```

开发完成后重新构建：

```bash
npm run build
```

回到项目根目录：

```bash
cd ../..
```

如果 Web 页面已经在浏览器中打开，更新前端后建议强制刷新页面。

## 关键配置文件

| 文件 | 说明 |
| --- | --- |
| `asm0003/urdf/asm0003.urdf` | ROS 和 MoveIt 使用的机器人模型。 |
| `web_control/frontend/public/urdf/unoarm.urdf` | 浏览器端显示使用的机器人模型。 |
| `asm0003/config/joint_limits.yaml` | MoveIt 关节限位。 |
| `CSP_direct_drive/src/light_test/config/joint_limits_calibrated.yaml` | 电机侧校准限位。 |
| `CSP_direct_drive/src/light_test/config/master0_moveit_bridge.yaml` | 左臂 bridge 配置。 |
| `CSP_direct_drive/src/light_test/config/master2_moveit_bridge.yaml` | 右臂 bridge 配置。 |
| `vision/camera_extrinsic.json` | 相机外参。 |
| `web_control/web_control/points.json` | Web 页面保存的点位。 |
| `web_control/web_control/presets.json` | Web 页面预设数据。 |

## 常用环境变量

| 变量 | 默认值 | 作用 |
| --- | --- | --- |
| `ROS_DOMAIN_ID` | `25` | 真机/通用 ROS 2 Domain ID。MuJoCo 仿真入口忽略该值，改用 `MUJOCO_ROS_DOMAIN_ID`。 |
| `ROS_AUTOMATIC_DISCOVERY_RANGE` | `SUBNET` | ROS 2 自动发现范围。 |
| `ROS_STATIC_PEERS` | `192.168.10.200` | DDS 静态发现地址。 |
| `WEB_CONTROL_HOST` | `0.0.0.0` | Web 控制监听地址。 |
| `WEB_CONTROL_PORT` | `8765` | Web 控制端口。 |
| `VISION_WEB_HOST` | `0.0.0.0` | 视觉预览监听地址。 |
| `VISION_WEB_PORT` | `8090` | 视觉预览端口。 |
| `VISION_STREAM_WIDTH` | `0` | 视觉预览输出宽度，`0` 表示使用原始宽度。 |
| `VISION_STREAM_HEIGHT` | `0` | 视觉预览输出高度，`0` 表示使用原始高度。 |
| `VISION_JPEG_QUALITY` | `92` | MJPEG 图像质量。 |
| `VISION_MIN_DEPTH` | `0.20` | 点云和目标深度下限。 |
| `VISION_MAX_DEPTH` | `4.0` | 点云和目标深度上限。 |
| `VISION_POINT_STRIDE` | `4` | 点云采样步长。 |
| `JOINT_STATE_TOPIC` | `/joint_states` | Web 控制读取的关节状态话题。 |
| `MOVE_GROUP_NAME` | `arm` | 右臂 MoveIt group。 |
| `LEFT_MOVE_GROUP_NAME` | `left_arm` | 左臂 MoveIt group。 |
| `FRAME_ID` | `world` | Web 显示、点位和视觉目标使用的坐标系。 |
| `MOVEIT_PLANNING_FRAME_ID` | `base_link` | 发给 MoveIt IK、轨迹规划和 PlanningScene 的规划坐标系。 |

示例：

```bash
WEB_CONTROL_PORT=8766 ./run_web_control.sh
```

```bash
VISION_WEB_PORT=8091 VISION_JPEG_QUALITY=80 ./start_robot_all.sh
```

## 日志

完整启动脚本会把各服务日志写入：

```text
logs/
```

常见日志文件：

| 文件 | 说明 |
| --- | --- |
| `logs/vision_target.log` | 视觉服务启动、相机、YOLO、点云和预览日志。 |
| `logs/csp_driver.log` | CSP 驱动和 MoveIt bridge 日志。 |
| `logs/moveit.log` | MoveIt、robot_state_publisher、joint state 合并日志。 |
| `logs/web_control.log` | Web 控制服务启动和 API 日志。 |

查看日志：

```bash
tail -f logs/web_control.log
```

同时查看多个日志：

```bash
tail -f logs/vision_target.log logs/csp_driver.log logs/moveit.log logs/web_control.log
```

## 测试和检查

运行 Python 结构检查：

```bash
python3 -m unittest discover -s tests -v
```

检查 Web 服务是否可访问：

```bash
curl http://127.0.0.1:8765/api/status
```

检查机器人状态接口：

```bash
curl http://127.0.0.1:8765/api/robot_state
```

检查 ROS 图：

```bash
./run_web_control.sh --check
```

## Git 使用流程

项目已经是 Git 仓库，正常修改后不要重新执行 `git init`。

常用流程：

```bash
git status
git add README.md
git commit -m "docs: update README"
git push
```

如果修改了多个文件：

```bash
git status
git add <文件或目录>
git commit -m "简短说明本次修改"
git push
```

查看最近提交：

```bash
git log --oneline --decorate -n 10
```

## 常见问题

### 1. `CSP action server not ready`

常见原因：

| 原因 | 处理 |
| --- | --- |
| EtherCAT master 没有启动 | 执行 `sudo ethercatctl status` 检查。 |
| sudo 密码错误或过期 | 重新运行 `./start_robot_all.sh`，按提示输入 sudo 密码。 |
| CSP 驱动没有构建 | 进入 `CSP_direct_drive` 后重新执行 `colcon build --symlink-install --packages-select light_test`。 |

手动重启 EtherCAT：

```bash
sudo ethercatctl restart
```

### 2. `MoveIt services not ready`

常见原因：

| 原因 | 处理 |
| --- | --- |
| `install/setup.bash` 不存在 | 重新构建 `asm0003`。 |
| joint state 没有发布 | 检查 CSP 驱动是否启动。 |
| ROS Domain 不一致 | 真机确认 `ROS_DOMAIN_ID=25`；MuJoCo 仿真确认 `MUJOCO_ROS_DOMAIN_ID=21`、`ROS_LOCALHOST_ONLY=1`。 |

重新构建：

```bash
colcon build --symlink-install --packages-select asm0003
```

### 3. 网页打不开

先检查服务：

```bash
curl http://127.0.0.1:8765/api/status
```

如果没有响应，查看日志：

```bash
tail -f logs/web_control.log
```

也可以单独重启 Web 控制：

```bash
./run_web_control.sh --stop
./run_web_control.sh
```

### 4. 页面资源没有更新

重新构建前端：

```bash
cd web_control/frontend
npm run build
cd ../..
```

然后重启 Web 控制：

```bash
./run_web_control.sh --stop
./run_web_control.sh
```

浏览器中强制刷新页面。

### 5. RealSense 打不开

常见原因：

| 原因 | 处理 |
| --- | --- |
| 相机被其他程序占用 | 停止其他 RealSense 程序后再启动。 |
| USB 连接异常 | 重新插拔相机或更换 USB 口。 |
| 视觉脚本没有权限或环境不对 | 检查 `vision/start_vision_target_writer.sh` 是否可执行。 |

### 6. 点云或避障区位置不对

优先检查：

| 检查项 | 文件或页面 |
| --- | --- |
| 相机外参 | `vision/camera_extrinsic.json` |
| URDF 中相机安装位置 | `web_control/frontend/public/urdf/unoarm.urdf` |
| 机器人主 URDF | `asm0003/urdf/asm0003.urdf` |
| 页面点云开关 | Web 页面中的点云显示按钮 |
| 避障区状态 | Web 页面中的显示避障区、应用避障区按钮 |

### 7. Git push 被拒绝

先不要重新 `git init`。查看状态：

```bash
git status
git branch -vv
git remote -v
```

如果远端比本地新，先同步：

```bash
git pull --no-rebase
```

解决冲突后再提交和推送：

```bash
git add <冲突文件>
git commit
git push
```

## 维护注意事项

- URDF、MoveIt 限位、电机侧限位要保持一致，尤其是左右臂关节方向和零点。
- 修改 `web_control/frontend/public/urdf/` 后，需要重新执行 `npm run build`。
- 修改 `asm0003/` 后，需要重新执行 `colcon build --symlink-install --packages-select asm0003`。
- 修改 `CSP_direct_drive/src/light_test/` 后，需要在 `CSP_direct_drive` 内重新构建 `light_test`。
- 不要把 `build/`、`install/`、`log/`、`logs/`、`node_modules/` 提交到 Git。
- 真实机械臂调试前，确认急停、使能状态和周围安全空间。
