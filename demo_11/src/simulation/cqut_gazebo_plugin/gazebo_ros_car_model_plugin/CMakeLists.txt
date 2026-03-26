add_library(gazebo_ros_car_model SHARED
  src/gazebo_ros_race_car_model.cpp
  src/vehicle.cpp
  src/wheel.cpp
  src/axle.cpp
  ../gazebo_utils/src/gazebo_utils.cpp
)

target_include_directories(gazebo_ros_car_model PUBLIC 
  ${PROJECT_SOURCE_DIR}
)
ament_target_dependencies(gazebo_ros_car_model
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "tf2"
  "visualization_msgs"
  "cqut_msg"
  "nav_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

ament_export_libraries(gazebo_ros_car_model)
ament_export_include_directories(${PROJECT_SOURCE_DIR})
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)
 
install(
  TARGETS gazebo_ros_car_model
  DESTINATION lib/${PROJECT_NAME}
)