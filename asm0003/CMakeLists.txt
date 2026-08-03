cmake_minimum_required(VERSION 3.8)
project(asm0003)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf config launch scripts mujoco
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
