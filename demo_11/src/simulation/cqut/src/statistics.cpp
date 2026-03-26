#include "cqut/statistics.h"

#include <rclcpp/rclcpp.hpp>

namespace cqut
{
    
bool ccw(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C)
{
    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x);
}

bool intersect(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C, const cv::Point2f &D)
{
    return ccw(A, C, D) != ccw(B, C, D) && ccw(A, B, C) != ccw(A, B, D);
}

cv::Point2f to_point(const cqut_msg::msg::State &state)
{
    return cv::Point2f(state.x, state.y);
}

LapStaticstic::LapStaticstic(const std::string mission)
{
    mission_ = mission;

    start_A_ = cv::Point2f(0.f, 2.f);
    start_B_ = cv::Point2f(0.f, -2.f);

    end_A_ = cv::Point2f(0.f, 0.f);
    end_B_ = cv::Point2f(0.f, 0.f);

    lap_count_ = 0;
    vel_avg_ = 0.0;

    rosbag_name_ = "";
    state_received_ = false;    

    starting_time_ = 0.0;
    res_go_time_ = 0.0;
}

bool LapStaticstic::request_stop()
{
    if(mission_ == "trackdrive")
        return lap_count_ > 110;
    return false;
}

bool LapStaticstic::is_mission_finnished()
{
    if(mission_ == "trackdrive")
    {
        RCLCPP_INFO(rclcpp::get_logger("Statistics"), "Lap count: %d, speed: %f", lap_count_, last_state_.vx);
        return lap_count_ == 11 && last_state_.vx <= 1.5;
    }
    else if(mission_ == "acceleration")
    {
        RCLCPP_INFO(rclcpp::get_logger("Statistics"), "State x: %f", last_state_.x);
        return last_state_.x > 76 && last_state_.x < 120 && lap_time_.size() != 0;
    }
    else if(mission_ == "skidpad")
    {
        RCLCPP_INFO(rclcpp::get_logger("Statistics"), "State x: %f, Laps: %d", last_state_.x, lap_count_);
        return last_state_.x > 4 && last_state_.x < 20 && lap_count_ == 5;
    }
}

void LapStaticstic::update_state(const cqut_msg::msg::State &state)
{
    state_received_ = true;
    if(mission_ == "trackdrive" || mission_ == "skidpad")
    {
        if(intersect(start_A_, start_B_, to_point(last_state_), to_point(state)))
        {
            lap_count_ += 1;
            if(lap_count_ == 1)
            {
                starting_time_ = rclcpp::Clock().now().seconds();
                res_go_time_ = rclcpp::Clock().now().seconds();
            }
            else
            {
                current_time_ = rclcpp::Clock().now().seconds();
                lap_time_.emplace_back(current_time_ - starting_time_);
                starting_time_ = current_time_;
                RCLCPP_INFO(rclcpp::get_logger("Statistics"), "LAP Time: %f", lap_time_.back());
            }
            RCLCPP_INFO(rclcpp::get_logger("Statistics"), "LAP: %d", lap_count_);
        }
    }
    else if(mission_ == "acceleration")
    {
        if(intersect(start_A_, start_B_, to_point(last_state_), to_point(state)))
        {
            RCLCPP_INFO(rclcpp::get_logger("Statistics"), "Starting  to measure");
            starting_time_ = rclcpp::Clock().now().seconds();
            res_go_time_ = rclcpp::Clock().now().seconds();
        }
        if(intersect(end_A_, end_B_, to_point(last_state_), to_point(state)))
        {
            lap_time_.emplace_back(rclcpp::Clock().now().seconds());
            RCLCPP_INFO(rclcpp::get_logger("Statistics"), "STOP  stopwatch wioth time: %f", lap_time_.back());
        }
    }

    double vel = std::sqrt(state.vx * state.vx + state.vy * state.vy);
    if(vel_avg_ == 0.0)
        vel_avg_ = vel;
    else
        vel_avg_ = (vel + vel_avg_) / 2.0;
    
    last_state_ = state;
}

} // namespace cqut
