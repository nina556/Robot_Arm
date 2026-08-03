# AGENTS.md — UnoArm 开发指南

本文件面向 AI 助手和开发者，说明项目结构、开发流程和调试约定。

## 项目结构

```
unoarm/
├── README.md                  # 人类用户入门文档
├── AGENTS.md                  # 本文件
├── start_robot_all.sh         # 一键启动完整机器人栈（前台）
├── stop_robot_all.sh          # 一键停止机器人栈
├── start_mujoco_sim.sh        # 一键启动 MuJoCo 仿真栈（不启动真机 EtherCAT/CSP）
├── stop_mujoco_sim.sh         # 停止由 MuJoCo 仿真脚本记录的本地仿真栈
├── start_csp_driver.sh        # 启动 CSP 驱动桥接
├── start_robot_moveit.sh      # 启动 MoveIt
├── run_web_control.sh         # 启动/检查/停止 Web 控制服务
├── unoarm_menu.sh             # 双手控制菜单（功能对齐 master2_control.sh）
├── master2_control.sh         # 命令行控制入口
├── CSP_direct_drive/          # CSP 驱动包
│   ├── master2_control.sh
│   ├── master2_control_menu.py
│   ├── start_moveit_bridge.sh
│   └── src/light_test/        # 主要 ROS 包源码
├── asm0003/                   # MoveIt/URDF 包
├── web_control/               # Web 控制后端
├── vision/                    # 视觉目标、点云和预览服务
├── tools/                     # 可达性、IK/规划辅助工具
└── install/                   # colcon 编译产物
```

## 开发环境

### 当前开发重点：MuJoCo 仿真

- 当前分支按本地 MuJoCo Docker 开发处理，不需要连接或操作远端机器人。
- 默认仿真入口：`./start_mujoco_sim.sh --docker`
- 非容器仿真入口：`./start_mujoco_sim.sh`
- 仿真停止：前台运行时按 `Ctrl+C`，或运行 `./stop_mujoco_sim.sh`
- 仿真默认隔离：脚本强制使用 `MUJOCO_ROS_DOMAIN_ID=21` 并导出为 `ROS_DOMAIN_ID=21`，同时默认 `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`、`ROS_LOCALHOST_ONLY=1`。
- 不要让 MuJoCo 仿真使用真机域 `ROS_DOMAIN_ID=25`；脚本会默认拒绝该配置，除非显式设置 `UNOARM_ALLOW_MUJOCO_DOMAIN_25=1`。
- `start_mujoco_sim.sh` 只清理自己记录的仿真 PID（`logs/mujoco_sim_stack.pids`），不自动调用全局 `stop_robot_all.sh`，避免误停真机域进程。
- MuJoCo 栈复用 MoveIt、Web 控制和 ROS API，但不启动 `start_csp_driver.sh`，也不重启 EtherCAT master。
- 仿真桥接脚本：`asm0003/scripts/mujoco_sim_bridge.py`
- 仿真模型默认路径：
  - `asm0003/mujoco/asm0003_rotated.mjb`
  - `asm0003/mujoco/asm0003_rotated.urdf`
- 如需无窗口运行：`MUJOCO_VIEWER=0 ./start_mujoco_sim.sh`
- 如需打开 MuJoCo viewer：`MUJOCO_VIEWER=1 ./start_mujoco_sim.sh`

MuJoCo 开发时避免触碰：

- 不 SSH 到远端机器人做验证，除非用户明确要求。
- 不改电机零位、编码器限位、EtherCAT sudo 配置。
- 不运行 `./start_robot_all.sh` 做验证，除非用户明确要求真机测试。
- 不改 `CSP_direct_drive/` 真机驱动逻辑，除非问题明确来自真机桥接。

### 远端机器人

- 当前分支默认不使用远端机器人；以下信息仅用于用户明确要求真机调试时。
- 主机：`niic@192.168.10.200`
- 仓库目录：`/home/niic/workspace/unoarm`
- 注意：`/home/niic/workspace` 是上层目录，不是仓库根目录。
- 当前机器已配置免密 SSH，可直接从本地连入调试

### 本地仓库

本地仓库路径：`/home/guanghe-sun/Desktop/Projects/unosimulate/unoarm`

代码修改流程：

1. 在本地仓库修改、提交、push
2. SSH 到机器人 `niic@192.168.10.200`
3. 在 `/home/niic/workspace/unoarm` 执行 `git pull`
4. 如需重新编译：`colcon build --symlink-install`
5. 在机器人上测试运行

## 调试命令

检查服务是否就绪：

```bash
ssh niic@192.168.10.200 'cd /home/niic/workspace/unoarm && ./run_web_control.sh --check'
```

查看运行中的进程：

```bash
ssh niic@192.168.10.200 'ps aux | grep -E "csp|moveit|web_control|master2|master0" | grep -v grep'
```

查看日志：

```bash
ssh niic@192.168.10.200 'tail -f /home/niic/workspace/unoarm/logs/{mujoco_sim_bridge,moveit,web_control}.log'
```

检查 ROS 图是否被其它仿真或旧节点污染：

```bash
source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=21; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST; export ROS_LOCALHOST_ONLY=1; ros2 node list
```

如果看到重复的 `/move_group_backend`，先停止当前栈并确认没有旧的 MuJoCo/MoveIt/Web 控制节点残留。重复 move_group 会导致 `/plan_kinematic_path` 请求落到错误后端。

## 修改约定

- 脚本保持可执行：`chmod +x *.sh`
- 兼容 workspace-root install 和包内 install 两种布局
- 不要硬编码 sudo 密码；如需免密，建议配置 `sudoers`
- 菜单按键行为与 `CSP_direct_drive/master2_control.sh` 保持一致
- 以后提交和上传代码时，`git commit` 的提交说明必须使用中文；push 或同步说明也使用中文描述改动。
- 只做 MuJoCo 开发时，优先修改仿真脚本、MoveIt/Web 控制适配和文档，不动真机驱动代码。
- 提交前在机器人上 `bash -n` 检查脚本语法。

## 关键测试

MuJoCo 文档或仿真相关修改后至少验证：

1. `bash -n start_mujoco_sim.sh`
2. `bash -n stop_mujoco_sim.sh`
3. `MUJOCO_VIEWER=0 ./start_mujoco_sim.sh --docker` 能启动到 Web 控制服务
4. `http://127.0.0.1:8765/api/status` 返回 JSON
5. ROS 图中不要出现重复的 `/move_group_backend`

真机链路修改后至少验证：

1. `bash -n unoarm_menu.sh`
2. `bash -n start_robot_all.sh`
3. 机器人上 `./start_robot_all.sh` 能完整启动
4. `http://192.168.10.200:8765/api/status` 返回 JSON
5. `./unoarm_menu.sh` 中 `s` 能显示双手状态
