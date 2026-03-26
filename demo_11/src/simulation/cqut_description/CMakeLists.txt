cmake_minimum_required(VERSION 3.8)
project(cqut_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes models rviz urdf world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

