#pragma once

#include "fsd_control/control.hpp"
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>

namespace ns_control 
{

class ControlHandle 
{
public:
    ControlHandle(rclcpp::Node::SharedPtr nodeHandle);

    int getNodeRate() const;

    void loadParameters();
    void subscribeToTopics();
    void publishToTopics();
    void run();
    void sendMsg();

private:
    void carStateCallback(const cqut_msg::msg::CarState::SharedPtr msg);
    void refTrajCallback(const cqut_msg::msg::Trajectory::SharedPtr msg);


    rclcpp::Node::SharedPtr nodeHandle_;

    rclcpp::Subscription<cqut_msg::msg::CarState>::SharedPtr carStateSubscriber_{nullptr};
    rclcpp::Subscription<cqut_msg::msg::Trajectory>::SharedPtr refTrajectorySubscriber_{nullptr};

    rclcpp::Publisher<cqut_msg::msg::ControlCommand>::SharedPtr cmdPublisher_{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr prePathPublisher_{nullptr};

    std::string car_state_topic_name_;
    std::string map_topic_name_;
    std::string ctrl_cmd_topic_name_;
    std::string predict_path_topic_name_;
    std::string ref_path_topic_name_;

    int node_rate_{};

    Control control_;
};

}
