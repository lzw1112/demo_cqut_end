#include "line_detect/line_detector_handle.hpp"

namespace ns_line_detector 
{

// Constructor
LineDetectorHandle::LineDetectorHandle(rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle), line_detector_(nodeHandle) 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

// Getters
int LineDetectorHandle::getNodeRate() const 
{ 
    return node_rate_; 
}

// Methods
void LineDetectorHandle::loadParameters() 
{
    nodeHandle_->declare_parameter("lidar_cluster_topic_name", "/lidar_cluster");
    nodeHandle_->declare_parameter("end_point_topic_name", "/planning/end_point");
    nodeHandle_->declare_parameter("node_rate", 50);

    RCLCPP_INFO(nodeHandle_->get_logger(), "loading handle parameters");
    if (!nodeHandle_->get_parameter("lidar_cluster_topic_name", lidar_cluster_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load lidar_cluster_topic_name. Standard value is: %s", lidar_cluster_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("end_point_topic_name", end_point_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load end_point_topic_name. Standard value is: %s", end_point_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("node_rate", node_rate_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load node_rate. Standard value is: %d", node_rate_);
    }
}

void LineDetectorHandle::subscribeToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "subscribe to topics");
    lidarClusterSubscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud>(lidar_cluster_topic_name_, 1, std::bind(&LineDetectorHandle::lidarClusterCallback, this, std::placeholders::_1));
}

void LineDetectorHandle::publishToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "publish to topics");
    endPointPublisher_ = nodeHandle_->create_publisher<geometry_msgs::msg::Point>(end_point_topic_name_, 1);
}

void LineDetectorHandle::run() 
{
    line_detector_.runAlgorithm();
    sendMsg();
}

void LineDetectorHandle::sendMsg() 
{
    endPointPublisher_->publish(line_detector_.getendPoint());
}

void LineDetectorHandle::lidarClusterCallback(const sensor_msgs::msg::PointCloud::SharedPtr msg) 
{
    line_detector_.setlidarCluster(*msg);
}

}