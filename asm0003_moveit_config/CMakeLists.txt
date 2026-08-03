cmake_minimum_required(VERSION 3.8)
project(asm0003_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    ../asm0003/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  FILES
    ../asm0003/urdf/asm0003.urdf
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    ../asm0003/urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
