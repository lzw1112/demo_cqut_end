#include "cqut/automated_res.h"

namespace cqut
{
    
AutomatedRes::AutomatedRes(const rclcpp::Node::SharedPtr nh, const std::string &config, int sim_id) : nh_(nh)
{
    buffer_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, nh);

    delay_after_start_command_ = 2.0;
    sim_health_ = cqut_msg::msg::SimHealth();

    ecu_ = Ecu();

    sub_topic_health_ = nh_->create_subscription<cqut_msg::msg::TopicsHealth>("/cqut/topics_health", 1, std::bind(&AutomatedRes::topic_health_callback, this, std::placeholders::_1));
    sub_state_ = nh_->create_subscription<cqut_msg::msg::State>("/cqut/base_pose_ground_truth", 1, std::bind(&AutomatedRes::state_callback, this, std::placeholders::_1));
    sub_mission_finished_ = nh_->create_subscription<cqut_msg::msg::Mission>("/cqut/mission_finished", 1, std::bind(&AutomatedRes::mission_callback, this, std::placeholders::_1));

    pub_res_ = nh_->create_publisher<cqut_msg::msg::ResState>("/cqut/res_state", 1);
    pub_health_ = nh_->create_publisher<cqut_msg::msg::SimHealth>("/cqut/health", 1);
    pub_mission_ = nh_->create_publisher<cqut_msg::msg::Mission>("/cqut/mission", 1);
    pub_initialpose_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);

    config_ = YAML::LoadFile(config);
    sim_id_ = sim_id;

    checks_is_in_track_ = config_["res"]["checks"]["is_in_track"].as<bool>();
    robot_name_ = config_["robot_name"].as<std::string>();

    std::string mission_type = config_["repetitions"][sim_id]["track_name"].as<std::string>();
    mission_ = cqut_msg::msg::Mission();
    if(mission_type == "skidpad")
    {
        mission_.mission = "skidpad";
        checks_is_in_track_ = false;
    }
    else if(mission_type == "acceleration")
    {
        mission_.mission = "acceleration";
        checks_is_in_track_ = false;
    }
    else
    {
        mission_.mission = "trackdrive";
    }
    RCLCPP_INFO(nh->get_logger(), "Mission: %s", mission_.mission.c_str());
    pub_mission_->publish(mission_);

    statistics_ = LapStaticstic(mission_.mission);

    track_checks_ = VehiclePositionCheck(nh, mission_.mission, checks_is_in_track_);

    start_time_ = 0.0;
}

void AutomatedRes::run()
{
    RCLCPP_INFO(nh_->get_logger(), "Setting Initial Pose");
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.position.x = 0;
    init_pose.pose.pose.position.y = 0;
    init_pose.pose.pose.position.z = 0;
    init_pose.pose.pose.orientation.w = 1;
    init_pose.pose.pose.orientation.w = 0;
    init_pose.pose.pose.orientation.w = 0;
    init_pose.pose.pose.orientation.w = 0;
    pub_initialpose_->publish(init_pose);

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        
        if(!track_checks_.is_car_in_track())
        {
            RCLCPP_WARN(nh_->get_logger(), "Car Is not in the track");
        }

        

        rate.sleep();
        rclcpp::spin_some(nh_);
    }
    
}

void AutomatedRes::topic_health_callback(const cqut_msg::msg::TopicsHealth::SharedPtr msg)
{

}

void AutomatedRes::state_callback(const cqut_msg::msg::State::SharedPtr msg)
{

}

void AutomatedRes::mission_callback(const cqut_msg::msg::Mission::SharedPtr msg)
{

}

} // namespace cqut


