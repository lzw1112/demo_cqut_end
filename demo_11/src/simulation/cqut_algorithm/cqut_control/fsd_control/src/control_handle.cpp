#include <rclcpp/rclcpp.hpp>
#include "fsd_control/control_handle.hpp"
#include <chrono>

namespace ns_control 
{

ControlHandle::ControlHandle(rclcpp::Node::SharedPtr nodeHandle) : nodeHandle_(nodeHandle), control_(nodeHandle) 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "Constructing Handle");
    loadParameters();
    subscribeToTopics();
    publishToTopics();
}

int ControlHandle::getNodeRate() const 
{ 
    return node_rate_; 
}


void ControlHandle::loadParameters() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "loading handle parameters");

    nodeHandle_->declare_parameter("car_state_topic_name", "/estimation/slam/state");   // sub
    nodeHandle_->declare_parameter("node_rate", 10);
    nodeHandle_->declare_parameter("map_topic_name", "/map");
    nodeHandle_->declare_parameter("ctrl_cmd_topic_name", "/control/pure_pursuit/control_command"); // pub
    nodeHandle_->declare_parameter("ref_path_topic_name", "/planning/ref_path");    // sub
    nodeHandle_->declare_parameter("predict_path_topic_name", "/visual/pre_path");  // pub

    if (!nodeHandle_->get_parameter("car_state_topic_name", car_state_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load car_state_topic_name. Standard value is: %s", car_state_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("node_rate", node_rate_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load node_rate. Standard value is: %d", node_rate_);
    }
    if (!nodeHandle_->get_parameter("map_topic_name", map_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load map_topic_name. Standard value is : %s", map_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("ctrl_cmd_topic_name", ctrl_cmd_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load ctrl_cmd_topic_name. Standard value is : %s", ctrl_cmd_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("ref_path_topic_name", ref_path_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load ref_path_topic_name_. Standard value is: %s", ref_path_topic_name_.c_str());
    }
    if (!nodeHandle_->get_parameter("predict_path_topic_name", predict_path_topic_name_)) 
    {
        RCLCPP_WARN(nodeHandle_->get_logger(), "Did not load visual_map_topic_name. Standard value is: %s", predict_path_topic_name_.c_str());
    }
}

void ControlHandle::subscribeToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "subscribe to topics");
    refTrajectorySubscriber_ = nodeHandle_->create_subscription<cqut_msg::msg::Trajectory>(ref_path_topic_name_, 1, std::bind(&ControlHandle::refTrajCallback, this, std::placeholders::_1));
    carStateSubscriber_ = nodeHandle_->create_subscription<cqut_msg::msg::CarState>(car_state_topic_name_, 10, std::bind(&ControlHandle::carStateCallback, this, std::placeholders::_1));
}

void ControlHandle::publishToTopics() 
{
    RCLCPP_INFO(nodeHandle_->get_logger(), "publish to topics");
    cmdPublisher_ = nodeHandle_->create_publisher<cqut_msg::msg::ControlCommand>(ctrl_cmd_topic_name_, 1);
    prePathPublisher_ = nodeHandle_->create_publisher<visualization_msgs::msg::MarkerArray>(predict_path_topic_name_, 1);
}

void ControlHandle::carStateCallback(const cqut_msg::msg::CarState::SharedPtr msg) 
{
    control_.setCarState(*msg);
}

void ControlHandle::refTrajCallback(const cqut_msg::msg::Trajectory::SharedPtr msg) 
{
    Trajectory ref_path;
    for (auto point : msg->trajectory) 
    {
        TrajectoryPoint tpt;
        tpt.pts.x = point.pts.x;
        tpt.pts.y = point.pts.y;
        tpt.acc = point.acc.data;
        tpt.curvature = point.curvature.data;
        tpt.yaw = point.yaw.data;
        tpt.r = point.r.data;
        tpt.velocity = point.velocity.data;
        ref_path.push_back(tpt);
    }
    control_.setTrack(ref_path);
}

void ControlHandle::run() 
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    control_.runAlgorithm();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double time_round = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    RCLCPP_INFO(nodeHandle_->get_logger(), "time cost = %f, frequency = %f", time_round, 1 / time_round);
    sendMsg();
}

void ControlHandle::sendMsg() 
{
    cmdPublisher_->publish(control_.getCmd());
    prePathPublisher_->publish(control_.getPrePath());
}

}
