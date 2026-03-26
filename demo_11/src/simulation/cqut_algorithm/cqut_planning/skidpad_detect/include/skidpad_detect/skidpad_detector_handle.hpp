#pragma once

#include "skidpad_detect/skidpad_detector.hpp"

namespace ns_skidpad_detector 
{

class SkidpadDetectorHandle 
{

public:
    SkidpadDetectorHandle(rclcpp::Node::SharedPtr nodeHandle);

    // Getters
    int getNodeRate() const;

    // Methods
    void loadParameters();
    void subscribeToTopics();
    void publishToTopics();
    void run();
    void sendMsg();

private:
    rclcpp::Node::SharedPtr nodeHandle_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr clusterFilteredSubscriber_{nullptr};
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr transformMatrixPublisher_{nullptr};

    void clusterFilteredCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg);

    std::string cluster_filtered_topic_name_;
    std::string transform_matrix_topic_name_;

    int node_rate_;

    SkidpadDetector skidpad_detector_;

};
}
