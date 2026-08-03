# v0.1 Local MoveIt Control

Local MoveIt/RViz launcher for the remote v0.1 controller on:

```text
niic@192.168.10.200:~/workspace/v0.1
```

This folder is intentionally independent from the v1.0 bridge/mapper path.

## Remote Prerequisite

On the control box:

```bash
cd ~/workspace/v0.1
./start_moveit_bridge.sh
```

The remote bridge should expose:

```text
/master2/joint_states
/arm_controller/follow_joint_trajectory
```

It starts disabled. Enable only when ready for real motion:

```bash
cd ~/workspace/v0.1
./master2_control.sh enable
```

Emergency stop:

```bash
./master2_control.sh estop
```

## Local Checks

```bash
cd /home/guanghe-sun/Desktop/Projects/v0.1control
./scripts/check_v01_remote.sh
```

## Display Only

```bash
cd /home/guanghe-sun/Desktop/Projects/v0.1control
./scripts/launch_display_v01.sh
```

## MoveIt + RViz

```bash
cd /home/guanghe-sun/Desktop/Projects/v0.1control
./scripts/launch_moveit_v01.sh
```

Default DDS settings:

```text
ROS_DOMAIN_ID=25
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ROS_STATIC_PEERS=192.168.10.200
```

