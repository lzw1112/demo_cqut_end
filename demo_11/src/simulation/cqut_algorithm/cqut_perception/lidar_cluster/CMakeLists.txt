cmake_minimum_required(VERSION 3.8)
project(lidar_cluster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED)

add_executable(${PROJECT_NAME}
  src/lidar_cluster_handle.cpp
  src/lidar_cluster.cpp
  src/main.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
"rclcpp"
"std_msgs"
"geometry_msgs"
"pcl_conversions"
"pcl_msgs"
)
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME}
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
