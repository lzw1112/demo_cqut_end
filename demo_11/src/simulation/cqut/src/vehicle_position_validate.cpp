#include "cqut/vehicle_position_validate.h"

namespace cqut
{
    
bool is_left_cones(const geometry_msgs::msg::TransformStamped &trans, const std::vector<cv::Point2f> &cones)
{
    cv::Point2f A = cones.front();
    cv::Point2f B = cones.back();

    return (B.x - A.x) * (trans.transform.translation.y - A.y) > (B.y - A.y) * (trans.transform.translation.x - A.x);
}

bool is_inside(const geometry_msgs::msg::TransformStamped &trans, const std::vector<cv::Point2f> &polygon_outside, const std::vector<cv::Point2f> &polygon_inside)
{
    cv::Point2f point(trans.transform.translation.x, trans.transform.translation.y);
    return cv::pointPolygonTest(polygon_inside, point, false) && cv::pointPolygonTest(polygon_outside, point, false);
}

VehiclePositionCheck::VehiclePositionCheck(const rclcpp::Node::SharedPtr &nh, const std::string &mission, bool ignore_track_check, const YAML::Node node = YAML::Node())
{   
    sub_track_ = nh->create_subscription<cqut_msg::msg::Track>("/cqut/track", 10, std::bind(&VehiclePositionCheck::track_callbacl, this, std::placeholders::_1));
    buffer_ = std::make_unique<tf2_ros::Buffer>(nh->get_clock());
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, nh);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh);

    recevied_track_ = false;
    ignore_track_check_ = ignore_track_check;
    mission_ = mission;

    config_ = node;
}

bool VehiclePositionCheck::is_track_valid()
{
    return recevied_track_ || ignore_track_check_;
}

bool VehiclePositionCheck::is_car_in_track()
{
    if(mission_ == "trackdrive")
    {
        bool is_left_front = is_inside(get_trans("left_front_wheel"), polygon_outside_, polygon_inside_);
        bool is_right_front = is_inside(get_trans("right_front_wheel"), polygon_outside_, polygon_inside_);
        bool is_left_rear = is_inside(get_trans("left_rear_wheel"), polygon_outside_, polygon_inside_);
        bool is_right_rear = is_inside(get_trans("right_rear_wheel"), polygon_outside_, polygon_inside_);
        return is_left_front || is_right_front || is_left_rear || is_right_rear;
    }
    else if(mission_ == "acceleration")
    {
        bool is_left_from_right_cones = is_car_left_from_line(cones_right_);
        bool is_right_from_left_cones = is_car_right_from_line(cones_left_);
        return is_left_from_right_cones && is_right_from_left_cones;
    }
    else if(mission_ == "skidp")
    {
        bool left_front_wheel = is_bounding_box(get_trans("left_front_wheel"));
        bool right_front_wheel = is_bounding_box(get_trans("right_front_wheel"));
        bool left_rear_wheel = is_bounding_box(get_trans("left_rear_wheel"));
        bool right_rear_wheel = is_bounding_box(get_trans("right_rear_wheel"));
        return left_front_wheel && right_front_wheel && left_rear_wheel && right_rear_wheel;
    }
}

void VehiclePositionCheck::track_callbacl(const cqut_msg::msg::Track::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("Track"), "Track was recevied!");

    cones_left_.clear();
    cones_right_.clear();

    for(const auto &cone : msg->cones_left)
    {
        cv::Point2f point;
        point.x = cone.x;
        point.y = cone.y;
        cones_left_.emplace_back(point);
    }
    for(const auto &cone : msg->cones_right)
    {
        cv::Point2f point;
        point.x = cone.x;
        point.y = cone.y;
        cones_right_.emplace_back(point);
    }

    if(msg->tk_device_start.size() == 2)
    {
        start_A_.x = msg->tk_device_start[0].x;
        start_A_.y = msg->tk_device_start[0].y;
        start_B_.x = msg->tk_device_start[1].x;
        start_B_.y = msg->tk_device_start[1].y;
    }
    if(msg->tk_device_end.size() == 2)
    {
        end_A_.x = msg->tk_device_end[0].x;
        end_A_.y = msg->tk_device_end[0].y;
        end_B_.x = msg->tk_device_end[1].x;
        end_B_.y = msg->tk_device_end[1].y;
    }
    else
    {
        end_A_ = start_A_;
        end_B_ = start_B_;
    }

    if(cones_left_.size() == 0 || cones_right_.size() == 0)
        return;
    
    if(cv::pointPolygonTest(cones_right_, cones_left_.back(), false))
    {
        polygon_outside_ = cones_right_;
        polygon_inside_ = cones_left_;
    }
    else
    {
        polygon_outside_ = cones_left_;
        polygon_inside_ = cones_right_;
    }
}

geometry_msgs::msg::TransformStamped VehiclePositionCheck::get_trans(const std::string &source_frame)
{
    geometry_msgs::msg::TransformStamped trans;
    try
    {
        trans = buffer_->lookupTransform("/map", source_frame, tf2::TimePointZero);
    }
    catch(const tf2::LookupException& e)
    {
        rclcpp::sleep_for(std::chrono::seconds(1));
        return trans;
    }
    return trans;
}

bool VehiclePositionCheck::is_car_left_from_line(const std::vector<cv::Point2f> &line)
{
    bool is_left_front = is_left_cones(get_trans("left_front_wheel"), line);
    bool is_right_front = is_left_cones(get_trans("right_front_wheel"), line);
    bool is_left_rear = is_left_cones(get_trans("left_rear_wheel"), line);
    bool is_right_rear = is_left_cones(get_trans("right_rear_wheel"), line);
    return is_left_front && is_right_front && is_left_rear && is_right_rear;
}

bool VehiclePositionCheck::is_car_right_from_line(const std::vector<cv::Point2f> &line)
{
    bool is_left_front = !is_left_cones(get_trans("left_front_wheel"), line);
    bool is_right_front = !is_left_cones(get_trans("right_front_wheel"), line);
    bool is_left_rear = !is_left_cones(get_trans("left_rear_wheel"), line);
    bool is_right_rear = !is_left_cones(get_trans("right_rear_wheel"), line);
    return is_left_front && is_right_front && is_left_rear && is_right_rear;
}

bool VehiclePositionCheck::is_bounding_box(const geometry_msgs::msg::TransformStamped &trans, double size)
{
    double x = trans.transform.translation.x;
    double y = trans.transform.translation.y;
    return x <= size && x >= -size && y <= size && y >= -size;
}

} // namespace cqut
