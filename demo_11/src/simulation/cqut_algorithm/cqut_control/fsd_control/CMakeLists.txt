cmake_minimum_required(VERSION 3.8)
project(fsd_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cqut_tools REQUIRED)
find_package(cqut_msg REQUIRED)

find_package(IPOPT REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME}
  src/control.cpp
  src/control_handle.cpp
  src/main.cpp
  include/fsd_control/Utils/types.h
  src/Solver/solver_base.cpp
  src/Solver/mpc_solver.cpp
  src/Solver/pure_pursuit_solver.cpp
  src/Utils/visual.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC 
  include
  /usr/include/eigen3
  ${IPOPT_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "nav_msgs"
  "visualization_msgs"
  "cqut_tools"
  "cqut_msg"
)
target_link_libraries(${PROJECT_NAME}
/usr/local/lib/libcppad_lib.so
  ${OpenCV_LIBS}
  ${IPOPT_LIBRARIES}
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
