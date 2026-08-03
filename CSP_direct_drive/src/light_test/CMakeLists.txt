cmake_minimum_required(VERSION 3.12)
project(light_test)

find_package(ament_cmake REQUIRED)
find_package(Necro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)

if(NOT TARGET ethercat-cpp::ethercat-cpp)
  find_package(
    ethercat-cpp REQUIRED
    PATHS "/home/niic/Desktop/test_ws/NIIC-EtherCAT-Demo-4.1.5-NECRO"
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
  )
endif()

add_executable(master2_axis0 src/master2_axis0.cpp)
target_compile_features(master2_axis0 PRIVATE cxx_std_17)
target_compile_options(master2_axis0 PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(master2_axis0
  Necro::qiuniu
  ethercat-cpp::ethercat-cpp
  ${QIUNIU_LIBRARY}
)

add_executable(master2_encoder_publisher src/master2_encoder_publisher.cpp)
target_compile_features(master2_encoder_publisher PRIVATE cxx_std_17)
target_compile_options(master2_encoder_publisher PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(master2_encoder_publisher
  Necro::qiuniu
  ethercat-cpp::ethercat-cpp
  ${QIUNIU_LIBRARY}
)
ament_target_dependencies(master2_encoder_publisher
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(master2_moveit_bridge src/master2_moveit_bridge.cpp)
target_compile_features(master2_moveit_bridge PRIVATE cxx_std_17)
target_compile_options(master2_moveit_bridge PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(master2_moveit_bridge
  Necro::qiuniu
  ethercat-cpp::ethercat-cpp
  ${QIUNIU_LIBRARY}
)
ament_target_dependencies(master2_moveit_bridge
  control_msgs
  rclcpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)

install(TARGETS
  master2_axis0
  master2_encoder_publisher
  master2_moveit_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
