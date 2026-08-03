# Light_test

EtherCAT Master 2 第 0 轴最小交互控制程序。

程序启动后默认不使能，只注册扫描到的第一个 CiA402 电机。

## 编译

```bash
cd /home/niic/Light_test
source /opt/ros/humble/setup.bash
colcon build --packages-select light_test
```

## 运行

运行期间不能有其他程序同时占用 EtherCAT Master 2。

```bash
cd /home/niic/Light_test
sudo bash -c "source /opt/ros/humble/setup.bash && \
source install/setup.bash && \
export LD_LIBRARY_PATH=/home/niic/Desktop/test_ws/NIIC-EtherCAT-Demo-4.1.5-NECRO/lib:\$LD_LIBRARY_PATH && \
ros2 run light_test master2_axis0"
```

交互命令：

```text
status
enable
zero
rad 0.01
rad 0
count 123456
disable
quit
```

参数说明：

- `rad`：按每圈 `8388608` 编码器计数、减速比 `1`、方向 `1` 换算。
- `count`：绝对原始编码器目标值，不是增量。
- 两种命令共用限速，每个 `1 ms` EtherCAT 周期最多改变 `100` 个计数。
- `zero`：将当前位置保存到 `/home/niic/Light_test/zero_count.txt`。
