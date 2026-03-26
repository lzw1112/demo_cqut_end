cmake_minimum_required(VERSION 3.8)
project(cqut_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cqut_msg REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

add_executable(cqut_interface_node src/cqut_interface_node.cpp)
target_include_directories(cqut_interface_node PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(cqut_interface_node
  "rclcpp"
  "tf2"
  "tf2_ros"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "cqut_msg"
  "pcl_conversions"
  "tf2_geometry_msgs"
)
target_link_libraries(cqut_interface_node
  ${PCL_LIBRARIES}
)

install(
  TARGETS cqut_interface_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
