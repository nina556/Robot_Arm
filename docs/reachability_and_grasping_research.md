# 随机点可解性与抓取算法调研

## 已接入的第一层：点/姿态可解性

新增工具：

```bash
python3 tools/reachability_probe.py \
  --config tools/reachability_regions.example.yaml \
  --count 200 \
  --checker geometry \
  --output /tmp/unoarm_reachability.jsonl \
  --summary /tmp/unoarm_reachability_summary.json
```

`geometry` 是离线快速筛选，只判断目标点是否落在 URDF 推导出的粗略半径包络内。它不能证明 IK 成功，也不能证明避障规划成功。

样例配置用 `frame_id: world` 描述采样区域，同时用 `moveit_request_frame: base_link` 发送 MoveIt 请求。这样随机区域仍和 Web 控制使用的世界坐标一致，但 `/compute_ik` 不依赖 MoveIt 自己解析 `world -> base_link` 的 TF。

随机点生成阶段默认启用 `sampling.enforce_arm_reach: true`。实际采样空间是“配置区域”和“对应机械臂最大展开半径球”的交集：工具先从区域里抽点，再按 `distance(target, arm_base) <= max_reach` 拒绝半径外的点。`max_reach` 默认由 URDF 链长加 tool offset 推导，`geometry.reach_margin` 默认是 `0.0`，也就是不额外放大机械极限边界。

MoveIt/MuJoCo 或真机栈启动后，可以跑真实 IK：

```bash
source /opt/ros/humble/setup.bash
[ -f install/setup.bash ] && source install/setup.bash
python3 tools/reachability_probe.py \
  --config tools/reachability_regions.example.yaml \
  --count 100 \
  --checker moveit_ik \
  --output /tmp/unoarm_ik.jsonl \
  --summary /tmp/unoarm_ik_summary.json
```

对比当前 `asm0003/config/ompl_planning.yaml` 里的多个 OMPL planner：

```bash
python3 tools/reachability_probe.py \
  --config tools/reachability_regions.example.yaml \
  --count 50 \
  --checker-set ompl \
  --output /tmp/unoarm_ompl.jsonl \
  --summary /tmp/unoarm_ompl_summary.json
```

外部算法可以热插拔：

```bash
python3 tools/reachability_probe.py \
  --checker python:/abs/path/my_checker.py:MyChecker
```

插件类只需要实现 `check(sample)`。返回值可以是普通 `dict`，也可以是带 `as_dict()` 或 `ok/reason/details` 属性的对象。

## 建议的评估指标

先把抓取拆成两段评估：

1. 候选点质量：视觉/点云算法输出的抓取 pose 是否稳定、是否夹到物体、夹爪是否会撞平台或邻近物。
2. 机器人可执行性：IK 成功率、MoveIt 规划成功率、平均规划时间、轨迹点数、路径长度、是否接近关节限位。

当前工具解决第 2 段。第 1 段后续可以把算法输出的 grasp pose 写成 JSONL，再喂给同一个 checker 流程，不需要重写可解性评估。

## 成熟算法路线

### IK/运动规划

- KDL：当前 `asm0003/config/kinematics.yaml` 已使用。优点是 MoveIt 默认、稳定、低成本；缺点是 7 自由度冗余臂在复杂姿态下可能需要多次重启。
- TRAC-IK：成熟的数值 IK 备选，常用于替换 KDL 来提高收敛率和速度。需要安装对应 MoveIt kinematics plugin，并把 `kinematics_solver` 换成 TRAC-IK 插件。
- IKFast：解析 IK，速度快、结果稳定；代价是需要为机械臂生成解析求解器，7 自由度冗余臂未必直接省事。
- Cached IK：MoveIt 提供缓存包装器，适合大量重复测试相近姿态，可减少求解时间。
- OMPL：当前已配置 RRTConnect、RRT、RRTstar、EST、LBKPIECE。实践上先用 RRTConnect 做主 baseline；RRTstar 更偏路径质量但慢；LBKPIECE/EST 可以作为狭窄空间和失败 case 的对照。

### 抓取候选生成

- GPD：输入 3D 点云，输出平行夹爪的 6-DOF 抓取姿态；不需要 CAD 模型，适合先用 RealSense 点云快速接入。
- Dex-Net/GQ-CNN：基于大量合成数据和抓取鲁棒性标签，适合 2.5D 深度图平行夹爪抓取评分；对台面单物体/箱拣很成熟。
- GG-CNN：轻量 depth image 网络，输出每个像素的抓取质量和姿态；适合实时闭环、主要做 top-down 或近似平面抓取。
- GraspNet/AnyGrasp：更现代的通用 6/7-DOF 抓取方案，直接面向 clutter scene；但 AnyGrasp SDK 有 CUDA/MinkowskiEngine 和许可要求。
- Contact-GraspNet：从原始场景点云预测 6-DOF 抓取分布，适合杂乱场景；依赖 TensorFlow/CUDA，工程接入成本较高。

## 当前推荐路线

短期最稳：

1. 用 `reachability_probe.py` 在 MuJoCo + MoveIt 里把右臂/左臂工作空间热区扫出来。
2. 用 `moveit_ik` 和 `moveit_plan:RRTConnectkConfigDefault` 记录成功率，先找到“能稳定到”的区域和姿态范围。
3. 视觉候选点先用简单几何规则：从目标点云取最高点/法向，生成若干 yaw 角，交给可解性工具筛选。

中期推荐：

1. 接 GPD 或 GraspNet baseline，输出多个 6-DOF grasp pose。
2. 对每个 grasp pose 加 approach/lift 两个姿态，一起跑 MoveIt IK + RRTConnect。
3. 排序分数用：算法抓取置信度、IK 成功、规划成功、路径短、远离关节限位、夹爪 approach 方向无遮挡。

如果要上机器学习：

- 有 GPU 并能接受依赖复杂度：优先试 GraspNet baseline 或 AnyGrasp SDK。
- 想低风险快速闭环：先试 GPD 或 GG-CNN。
- 真正要“夹取好算法”，不要只看网络分数；最终要用 UnoArm 自己的数据做在线统计，把真实抓取成败回填到候选排序里。
