#pragma once

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fstream>
#include <cmath>

namespace ns_skidpad_detector 
{

class SkidpadDetector 
{

public:
  
    SkidpadDetector(rclcpp::Node::SharedPtr nh);

    // Getters
    std_msgs::msg::Float64MultiArray getTransMatrix();

    // Setters
    void setclusterFiltered(sensor_msgs::msg::PointCloud msg);


    void runAlgorithm();
private:

    rclcpp::Node::SharedPtr nh_;

    std::string path_pcd_, path_x_, path_y_;
    double start_length_, lidar2imu_, threshold_;

    bool getClusterFlag, matchFlag;
    sensor_msgs::msg::PointCloud cluster, skidpad_map;
    nav_msgs::msg::Path trans_path, standard_path;
    std_msgs::msg::Float64MultiArray trans_matrix_in_1D;
    Eigen::Matrix4f transformation;

    void loadParameters();
    void loadFiles();
};
}
