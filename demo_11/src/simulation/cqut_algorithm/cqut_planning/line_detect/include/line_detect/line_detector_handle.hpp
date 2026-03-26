#pragma once

#include "line_detect/line_detector.hpp"

namespace ns_line_detector 
{

class LineDetectorHandle 
{
public:
  // Constructor
  LineDetectorHandle(rclcpp::Node::SharedPtr nodeHandle);//ros2自带的shared_ptr,Node继承于一个templete class

    // Getters 获取器
    int getNodeRate() const;//返回node_rate_

    // Methods
    void loadParameters();
    void subscribeToTopics();
    void publishToTopics();
    void run();
    void sendMsg();

private:
    rclcpp::Node::SharedPtr nodeHandle_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr lidarClusterSubscriber_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr endPointPublisher_{nullptr};

    void lidarClusterCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

    std::string lidar_cluster_topic_name_;
    std::string end_point_topic_name_;

    int node_rate_;

    LineDetector line_detector_;

};
}
