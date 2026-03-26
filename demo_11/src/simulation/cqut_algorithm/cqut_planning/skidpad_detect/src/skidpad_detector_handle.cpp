#include "skidpad_detect/skidpad_detector_handle.hpp"

namespace ns_skidpad_detector 
{

SkidpadDetectorHandle::SkidpadDetectorHandle(rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle), skidpad_detector_(nodeHandle) 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

// Getters
int SkidpadDetectorHandle::getNodeRate() const 
{ 
    return node_rate_; 
}

// Methods
void SkidpadDetectorHandle::loadParameters() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "loading handle parameters");
    nodeHandle_->declare_parameter("cluster_filtered_topic_name", "/perception/lidar_cluster");
    nodeHandle_->declare_parameter("transform_matrix_topic_name", "/transform_matrix");
    nodeHandle_->declare_parameter("node_rate", 50);


    if (!nodeHandle_->get_parameter("cluster_filtered_topic_name", cluster_filtered_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load cluster_filtered_topic_name. Standard value is: %s", cluster_filtered_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("transform_matrix_topic_name", transform_matrix_topic_name_)) 
    {
       RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load transform_matrix_topic_name. Standard value is: %s", transform_matrix_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("node_rate", node_rate_)) 
    {
       RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load node_rate. Standard value is: %d", node_rate_);
    }
}

void SkidpadDetectorHandle::subscribeToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "subscribe to topics");
    clusterFilteredSubscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud>(cluster_filtered_topic_name_, 1, std::bind(&SkidpadDetectorHandle::clusterFilteredCallback, this, std::placeholders::_1));
}

void SkidpadDetectorHandle::publishToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "publish to topics");
    transformMatrixPublisher_ = nodeHandle_->create_publisher<std_msgs::msg::Float64MultiArray>(transform_matrix_topic_name_, 1);
}

void SkidpadDetectorHandle::run() 
{
    skidpad_detector_.runAlgorithm();
    sendMsg();
}

void SkidpadDetectorHandle::sendMsg() 
{
    transformMatrixPublisher_->publish(skidpad_detector_.getTransMatrix());
}

void SkidpadDetectorHandle::clusterFilteredCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg) 
{
    skidpad_detector_.setclusterFiltered(*msg);  
}
}