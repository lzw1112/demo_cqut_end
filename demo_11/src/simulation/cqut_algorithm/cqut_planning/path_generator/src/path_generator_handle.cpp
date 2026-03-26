#include <rclcpp/rclcpp.hpp>
#include "path_generator/path_generator_handle.hpp"

namespace ns_path_generator 
{

PathGeneratorHandle::PathGeneratorHandle(rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle), path_generator_(nodeHandle) 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

int PathGeneratorHandle::getNodeRate() const 
{ 
    return node_rate_; 
}

void PathGeneratorHandle::loadParameters() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "loading handle parameters");

    nodeHandle_->declare_parameter("car_state_topic_name", "/estimation/slam/state"); // sub
    nodeHandle_->declare_parameter("transform_matrix_topic_name", "/transform_matrix"); // sub
    nodeHandle_->declare_parameter("end_point_topic_name", "/planning/end_point"); // sub
    nodeHandle_->declare_parameter("node_rate", 10);
    nodeHandle_->declare_parameter("map_topic_name", "/map"); // sub
    nodeHandle_->declare_parameter("ref_path_topic_name", "/visual/ref_path");  // pub
    nodeHandle_->declare_parameter("path_generate_topic_name", "/planning/ref_path");   // pub

    if (!nodeHandle_->get_parameter("car_state_topic_name", car_state_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load car_state_topic_name. Standard value is: %s", car_state_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("transform_matrix_topic_name", transform_matrix_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load transform_matrix_topic_name. Standard value is: %s", transform_matrix_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("end_point_topic_name", end_point_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load end_point_topic_name. Standard value is: %s", end_point_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("node_rate", node_rate_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load node_rate. Standard value is: %d", node_rate_);
    }
    if (!nodeHandle_->get_parameter("map_topic_name", map_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load map_topic_name. Standard value is : %s", map_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("ref_path_topic_name", ref_path_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load ref_path_topic_name_. Standard value is: %s", ref_path_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("path_generate_topic_name", path_generate_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load path_generate_topic_name. Standard value is: %s", path_generate_topic_name_.c_str());
    }
}

void PathGeneratorHandle::subscribeToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "subscribe to topics");

    rclcpp::QoS qos_sub(10);
    qos_sub.durability(rclcpp::DurabilityPolicy::TransientLocal);

    //from line_detector
    endPointSubscriber_ = nodeHandle_->create_subscription<geometry_msgs::msg::Point>(end_point_topic_name_, 1, std::bind(&PathGeneratorHandle::endPointCallback, this, std::placeholders::_1));
    //from skidpad_detector
    transMatSubscriber_ = nodeHandle_->create_subscription<std_msgs::msg::Float64MultiArray>(transform_matrix_topic_name_, 1, std::bind(&PathGeneratorHandle::transMatCallback, this, std::placeholders::_1));
    //for trackdrive
    localMapSubscriber_ = nodeHandle_->create_subscription<cqut_msg::msg::Map>(map_topic_name_, qos_sub, std::bind(&PathGeneratorHandle::localMapCallback, this, std::placeholders::_1));
    //others
    carStateSubscriber_ = nodeHandle_->create_subscription<cqut_msg::msg::CarState>(car_state_topic_name_, 10, std::bind(&PathGeneratorHandle::carStateCallback, this, std::placeholders::_1));
}

void PathGeneratorHandle::publishToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "publish to topics");

    refPathVisualPublisher_ = nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(ref_path_topic_name_, 10);
    refPathPublisher_ = nodeHandle_->create_publisher<cqut_msg::msg::Trajectory>(path_generate_topic_name_, 10);
}

void PathGeneratorHandle::run() 
{
    path_generator_.runAlgorithm();
    sendMsg();
}

void PathGeneratorHandle::sendMsg() 
{
    refPathVisualPublisher_->publish(path_generator_.getRefPath());
    refPathPublisher_->publish(path_generator_.getRefTrajectory());
}

void PathGeneratorHandle::endPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) 
{
    path_generator_.setEndPoint(*msg);
}

void PathGeneratorHandle::localMapCallback(const cqut_msg::msg::Map::SharedPtr msg) 
{
    path_generator_.setLocalMap(*msg);
}

void PathGeneratorHandle::carStateCallback(const cqut_msg::msg::CarState::SharedPtr msg) 
{
    path_generator_.setCarState(*msg);
}

void PathGeneratorHandle::transMatCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) 
{
    path_generator_.setTransMat(*msg);
}

}