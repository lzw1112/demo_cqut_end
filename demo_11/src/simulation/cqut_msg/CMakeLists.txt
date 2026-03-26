cmake_minimum_required(VERSION 3.8)
project(cqut_msg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Vector3Ext.msg"
  "msg/CarInfo.msg"
  "msg/Cmd.msg"
  "msg/WheelSpeeds.msg"
  "msg/TopicState.msg"
  "msg/SimHealth.msg"
  "msg/ResState.msg"
  "msg/TopicsHealth.msg"
  "msg/Mission.msg"
  "msg/Track.msg"
  "msg/State.msg"
  "msg/Cone.msg"
  "msg/ControlCommand.msg"
  "msg/CarState.msg"
  "msg/CarStateDt.msg"
  "msg/Map.msg"
  "msg/Trajectory.msg"
  "msg/TrajectoryPoint.msg"
  DEPENDENCIES std_msgs geometry_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
