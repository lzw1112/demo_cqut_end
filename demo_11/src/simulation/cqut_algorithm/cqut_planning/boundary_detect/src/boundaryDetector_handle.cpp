#include <rclcpp/rclcpp.hpp>
#include "boundary_detect/boundaryDetector_handle.hpp"

namespace ns_boundaryDetector 
{

// Constructor
BoundaryDetectorHandle::BoundaryDetectorHandle(rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle), boundaryDetector_(nodeHandle) 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

// Getters
int BoundaryDetectorHandle::getNodeRate() const 
{ 
    return node_rate_; 
}

// Methods
void BoundaryDetectorHandle::loadParameters() 
{
    nodeHandle_->declare_parameter("local_map_topic_name", "/map");
    nodeHandle_->declare_parameter("boundary_detections_topic_name", "/planning/boundary_detections");
    nodeHandle_->declare_parameter("visual_triangles_topic_name", "/visualization/visual_triangles");
    nodeHandle_->declare_parameter("visual_boundary_topic_name", "/visualization/visual_boundary");
    nodeHandle_->declare_parameter("visual_tree_topic_name", "/visualization/visual_tree");
    nodeHandle_->declare_parameter("visual_path_topic_name", "/visualization/visual_path");
    nodeHandle_->declare_parameter("node_rate", 50);

    RCLCPP_INFO(nodeHandle_->get_logger(), "loading handle parameters");
    if (!nodeHandle_->get_parameter("local_map_topic_name", local_map_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load local_map_topic_name. Standard value is: %s", local_map_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("boundary_detections_topic_name", boundary_detections_topic_name_))
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load boundary_detections_topic_name. Standard value is: %s", boundary_detections_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("visual_triangles_topic_name", visual_triangles_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load visual_triangles_topic_name. Standard value is: %s", visual_triangles_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("visual_boundary_topic_name", visual_boundary_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load visual_boundary_topic_name. Standard value is: %s", visual_boundary_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("visual_tree_topic_name", visual_tree_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load visual_tree_topic_name. Standard value is: %s", visual_tree_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("visual_path_topic_name", visual_path_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load visual_path_topic_name. Standard value is: ", visual_path_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("node_rate", node_rate_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load node_rate. Standard value is: %d", node_rate_);
    }
}

void BoundaryDetectorHandle::subscribeToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "subscribe to topics");
    localMapSubscriber = nodeHandle_->create_subscription<cqut_msg::msg::Map>(local_map_topic_name_, 1, std::bind(&BoundaryDetectorHandle::localMapCallback, this, std::placeholders::_1));
}

void BoundaryDetectorHandle::publishToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "publish to topics");
    boundaryDetectionsPublisher = nodeHandle_->create_publisher<cqut_msg::msg::Map>(boundary_detections_topic_name_, 1);
    visualTrianglesPublisher = nodeHandle_->create_publisher<visualization_msgs::msg::Marker>(visual_triangles_topic_name_, 1);
    visualBoundaryPublisher = nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(visual_boundary_topic_name_, 1);
    visualTreePublisher = nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(visual_tree_topic_name_, 1);
    visualPathPublisher = nodeHandle_->create_publisher<visualization_msgs::msg::Marker>(visual_path_topic_name_, 1);
}

void BoundaryDetectorHandle::run() 
{
    boundaryDetector_.runAlgorithm();
    sendMsg();
}

void BoundaryDetectorHandle::sendMsg() 
{
    boundaryDetectionsPublisher->publish(boundaryDetector_.getboundaryDetections());
    visualTrianglesPublisher->publish(boundaryDetector_.getVisualTriangles());
    visualBoundaryPublisher->publish(boundaryDetector_.getVisualBoundary());
    visualTreePublisher->publish(boundaryDetector_.getVisualTree());
    visualPathPublisher->publish(boundaryDetector_.getVisualPath());
}

void BoundaryDetectorHandle::localMapCallback(const cqut_msg::msg::Map::SharedPtr msg) 
{
    boundaryDetector_.setLocalMap(*msg);
}

}