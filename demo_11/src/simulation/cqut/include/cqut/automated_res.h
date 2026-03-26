#pragma once

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <yaml-cpp/yaml.h>

#include "cqut/ecu.h"
#include "cqut/statistics.h"
#include "cqut/vehicle_position_validate.h"
#include "cqut_msg/msg/sim_health.hpp"
#include "cqut_msg/msg/topics_health.hpp"
#include "cqut_msg/msg/state.hpp"
#include "cqut_msg/msg/mission.hpp"
#include "cqut_msg/msg/res_state.hpp"


#define DISALLOW_COPY_AND_ASSIGN(ClassName) \
        ClassName(const ClassName&) {}      \
        void operator=(const ClassName&) {}

class A
{
public:
    A() {}
private:
    DISALLOW_COPY_AND_ASSIGN(A);
};

#undef DISALLOW_COPY_AND_ASSIGN(ClassName)

namespace cqut
{

class AutomatedRes
{
public:
    AutomatedRes(const rclcpp::Node::SharedPtr nh, const std::string &config, int sim_id);

    void run();
private:
    void topic_health_callback(const cqut_msg::msg::TopicsHealth::SharedPtr msg);
    void state_callback(const cqut_msg::msg::State::SharedPtr msg);
    void mission_callback(const cqut_msg::msg::Mission::SharedPtr msg);

    std::unique_ptr<tf2_ros::Buffer> buffer_{nullptr};
    std::unique_ptr<tf2_ros::TransformListener> listener_{nullptr};

    rclcpp::Subscription<cqut_msg::msg::TopicsHealth>::SharedPtr sub_topic_health_{nullptr};
    rclcpp::Subscription<cqut_msg::msg::State>::SharedPtr sub_state_{nullptr};
    rclcpp::Subscription<cqut_msg::msg::Mission>::SharedPtr sub_mission_finished_{nullptr};

    rclcpp::Publisher<cqut_msg::msg::ResState>::SharedPtr pub_res_{nullptr};
    rclcpp::Publisher<cqut_msg::msg::SimHealth>::SharedPtr pub_health_{nullptr};
    rclcpp::Publisher<cqut_msg::msg::Mission>::SharedPtr pub_mission_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_initialpose_{nullptr};

    rclcpp::Node::SharedPtr nh_{nullptr};

    double delay_after_start_command_;
    cqut_msg::msg::SimHealth sim_health_;
    cqut_msg::msg::Mission mission_;

    Ecu ecu_;
    LapStaticstic statistics_;
    VehiclePositionCheck track_checks_;

    YAML::Node config_;
    int sim_id_;

    bool checks_is_in_track_;
    std::string robot_name_;

    double start_time_;
};

} // namespace  cqut
