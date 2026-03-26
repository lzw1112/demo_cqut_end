#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

namespace ns_line_detector 
{

class LineDetector 
{

public:
    // Constructor
    LineDetector(rclcpp::Node::SharedPtr nh);

    // Getters
    geometry_msgs::msg::Point getendPoint();

    // Setters
    void setlidarCluster(sensor_msgs::msg::PointCloud msgs);

    void runAlgorithm();

private:
    void createPath();

    rclcpp::Node::SharedPtr nh_;
    
    sensor_msgs::msg::PointCloud cluster;
    geometry_msgs::msg::Point end_point;

    bool getPath = false;
    double path_length;
    double allow_angle_error;

};
}
