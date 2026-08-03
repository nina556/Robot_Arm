#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="/home/niic/Light_test"
SDK_LIB="/home/niic/Desktop/test_ws/NIIC-EtherCAT-Demo-4.1.5-NECRO/lib"
PARAMS="${WORKSPACE}/src/light_test/config/master2_joints.yaml"

echo "启动 Master 2 七轴编码器发布器。"
echo "发布话题：/master2/joint_states、/master2/encoder_counts"
echo "注意：不要同时运行 master2_axis0。"

cd "${WORKSPACE}"
exec sudo bash -c "
  source /opt/ros/humble/setup.bash
  source '${WORKSPACE}/install/setup.bash'
  export LD_LIBRARY_PATH='${SDK_LIB}':\"\${LD_LIBRARY_PATH:-}\"
  export ROS_DOMAIN_ID=25
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  export CYCLONEDDS_URI="file://${WORKSPACE}/src/light_test/config/cyclonedds.xml"
  exec ros2 run light_test master2_encoder_publisher --ros-args \
    --params-file '${PARAMS}'
"
