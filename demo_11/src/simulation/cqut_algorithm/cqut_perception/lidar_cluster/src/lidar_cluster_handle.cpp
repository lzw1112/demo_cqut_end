#include <rclcpp/rclcpp.hpp>
#include "lidar_cluster/lidar_cluster_handle.hpp"

namespace ns_lidar_cluster 
{

// Constructor
LidarClusterHandle::LidarClusterHandle(rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle), lidar_cluster_(nodeHandle) 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

// Getters
int LidarClusterHandle::getNodeRate() const 
{ 
    return node_rate_; 
}

// Methods
void LidarClusterHandle::loadParameters() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "loading handle parameters");

    nodeHandle_->declare_parameter("raw_lidar_topic_name", "/velodyne_points_init");    // sub
    nodeHandle_->declare_parameter("lidar_cluster_topic_name", "/perception/lidar_cluster");    // pub
    nodeHandle_->declare_parameter("node_rate", 10);

    if (!nodeHandle_->get_parameter("raw_lidar_topic_name", raw_lidar_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load raw_lidar_topic_name. Standard value is: %s", raw_lidar_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("lidar_cluster_topic_name", lidar_cluster_topic_name_))
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load lidar_cluster_topic_name. Standard value is: %s", lidar_cluster_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("node_rate", node_rate_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load node_rate. Standard value is: %d", node_rate_);
    }
}

void LidarClusterHandle::subscribeToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "subscribe to topics");
    rawLidarSubscriber_ = nodeHandle_->create_subscription<sensor_msgs::msg::PointCloud2>(raw_lidar_topic_name_, 1, std::bind(&LidarClusterHandle::rawLidarCallback, this, std::placeholders::_1));
}

void LidarClusterHandle::publishToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "publish to topics");
    lidarClusterPublisher_ = nodeHandle_->create_publisher<sensor_msgs::msg::PointCloud>(lidar_cluster_topic_name_, 1);
}

void LidarClusterHandle::run() 
{
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    lidar_cluster_.runAlgorithm();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t2).count();
    RCLCPP_INFO(nodeHandle_->get_logger(), "time: %f", time_round);
    sendMsg();
}

void LidarClusterHandle::sendMsg() 
{
    if(!lidar_cluster_.is_ok())
        return;
    lidarClusterPublisher_->publish(lidar_cluster_.getLidarCluster());
}

void LidarClusterHandle::rawLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    lidar_cluster_.setRawLidar(*msg);
}
}