#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "cqut_msg/msg/track.hpp"

namespace cqut
{
    
bool is_left_cones(const geometry_msgs::msg::TransformStamped &trans, const std::vector<cv::Point2f> &cones);
bool is_inside(const geometry_msgs::msg::TransformStamped &trans, const std::vector<cv::Point2f> &polygon_outside, const std::vector<cv::Point2f> &polygon_inside);

class VehiclePositionCheck
{
public:
    VehiclePositionCheck() = default;
    VehiclePositionCheck(const rclcpp::Node::SharedPtr &nh, const std::string &mission, bool ignore_track_check, const YAML::Node config = YAML::Node());

    bool is_track_valid();
    bool is_car_in_track();
private:
    void track_callbacl(const cqut_msg::msg::Track::SharedPtr msg);
    geometry_msgs::msg::TransformStamped get_trans(const std::string &source_frame);
    bool is_car_left_from_line(const std::vector<cv::Point2f> &line);
    bool is_car_right_from_line(const std::vector<cv::Point2f> &line);
    bool is_bounding_box(const geometry_msgs::msg::TransformStamped &trans, double size = 25.0);

    rclcpp::Subscription<cqut_msg::msg::Track>::SharedPtr sub_track_{nullptr};

    bool recevied_track_;
    bool ignore_track_check_;
    std::string mission_;

    YAML::Node config_;

    std::vector<cv::Point2f> cones_left_;
    std::vector<cv::Point2f> cones_right_;
    std::vector<cv::Point2f> polygon_outside_;
    std::vector<cv::Point2f> polygon_inside_;
    cv::Point2f start_A_;
    cv::Point2f start_B_;
    cv::Point2f end_A_;
    cv::Point2f end_B_;

    std::unique_ptr<tf2_ros::TransformListener> listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> buffer_{nullptr};
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_{nullptr};
};

} // namespace cqut
