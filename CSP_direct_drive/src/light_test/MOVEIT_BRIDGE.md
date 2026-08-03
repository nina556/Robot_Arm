# Master 2 MoveIt Bridge

The controller starts with all motors disabled.

## Operator commands

```bash
cd /home/niic/Light_test
./master2_control.sh status
./master2_control.sh enable
./master2_control.sh disable
./master2_control.sh estop
./master2_control.sh reset
```

An emergency stop is latched. Publishing `false` does not clear it. Reset is
allowed only when EtherCAT feedback is available, all limited joints are inside
their raw encoder ranges, and no drive reports an error. Reset does not enable
the motors.

J3 and J5 are continuous joints without software position limits. Their target
angle is mapped to the nearest equivalent encoder turn to prevent a full-turn
jump when MoveIt crosses from `+pi` to `-pi`. Slew-rate, following-error,
drive-error and emergency-stop protection still applies.

## MoveIt controller

Use `config/moveit_controllers.yaml`. The action endpoint is:

```text
/arm_controller/follow_joint_trajectory
```

MoveIt targets are radians. They are converted to raw encoder coordinates with:

```text
raw = zero_offset + direction * radians * counts_per_rev * gear_ratio / (2*pi)
```

The bridge validates every trajectory point against the raw encoder limits
before accepting a goal.

## Safety scope

The ROS emergency stop is software protection. A hard-wired emergency stop
that removes drive torque is still required for work around the robot.
