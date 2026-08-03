# UnoArm Docker 开发环境

这个 Docker 环境使用 OSRF 的 `docker.io/osrf/ros:humble-desktop` 作为基础镜像，并额外安装 MoveIt、ros2_control、colcon、Node/npm、MuJoCo Python 包和常用开发工具。

镜像不安装 Codex CLI 和 cc-switch。

## 构建镜像

在项目根目录执行：

当前开发机的 `docker` 命令是 Podman 兼容层，不是 Docker daemon。推荐使用脚本构建，脚本会自动选择 Podman 或 Docker：

```bash
./docker/build-dev.sh
```

脚本在 Podman 环境下会使用：

```bash
podman build --network=host --format docker ...
```

这样可以让构建容器访问宿主机 `127.0.0.1` 代理，并避免 OCI 格式忽略 `SHELL` 指令的警告。

进入容器：

```bash
./docker/run-dev-x11.sh
```

如果通过容器启动 MuJoCo 仿真，包装脚本会把仿真 ROS 图隔离到本机：

```bash
./start_mujoco_sim.sh --docker
```

该路径默认使用 `MUJOCO_ROS_DOMAIN_ID=21`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_LOCALHOST_ONLY=1`，不会继承真机常用的 `ROS_DOMAIN_ID=25`。

Podman 环境默认不启用 `--userns=keep-id`。部分文件系统不支持 shifting，
启用 keep-id 时 Podman 会复制整层镜像来做 ID 映射，首次启动可能长时间卡在
`creating an ID-mapped copy of layer`。如确实需要 keep-id，可显式启用：

```bash
UNOARM_PODMAN_KEEP_ID=1 ./docker/run-dev-x11.sh
```

如果需要转发 GUI 窗口，例如 `rviz2` 或 `joint_state_publisher_gui`：

```bash
./docker/run-dev-x11.sh
```

如果宿主机没有 `xauth` 或窗口打不开，可临时放行本地 Docker 连接：

```bash
xhost +local:
./docker/run-dev-x11.sh
```

## 编译项目

容器内：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select asm0003 asm0003_moveit_config
```

前端：

```bash
cd web_control/frontend
npm ci
npm run build
cd ../..
```

检查 Web 控制服务：

```bash
./run_web_control.sh --check
```

`CSP_direct_drive/src/light_test` 依赖机器人 EtherCAT SDK/私有依赖，默认镜像不包含这些组件。
