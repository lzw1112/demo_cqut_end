#include "path_generator/Track/line_track.h"
#include <rclcpp/rclcpp.hpp>

namespace ns_path_generator 
{

    bool Line_Track::genTraj() 
    {
        const double interval = param_.interval;
        const double desire_vel = param_.desire_vel;

        double distance = std::hypot(endPoint_.x, endPoint_.y);
        double gradient = endPoint_.y / endPoint_.x;

        TrajectoryPoint tmp_pt;
        trajectory_.clear();
        if (interval == 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Line_track"), "Interval is too small.");
            return false;
        }
        for (double i = 0; i < distance; i += interval) 
        {  
            // y = kx and i^2 = x^2 + y^2
            tmp_pt.pts.x = i / std::hypot(1.0, gradient);;
            tmp_pt.pts.y = tmp_pt.pts.x * gradient;
            tmp_pt.yaw = atan2(tmp_pt.pts.y, tmp_pt.pts.x);
            tmp_pt.curvature = 0;
            tmp_pt.velocity = desire_vel;
            tmp_pt.r = 0;
            trajectory_.push_back(tmp_pt);
        }
        return true;
    }

    bool Line_Track::CalculateTraj(Trajectory &refline) 
    {

        if (trajectory_.empty()) 
        {
           RCLCPP_ERROR(rclcpp::get_logger("Line_track"), "Trajectory is empty !");
            return false;
        }
        refline.clear();
        TrajectoryPoint tmp_pt;

        double px = state_.x;
        double py = state_.y;
        double psi = state_.yaw;

        int count = 0;
        int index_min = -1;
        double min = 999;
        for (size_t i = 0; i < trajectory_.size(); i++) 
        {
            double delta_x = trajectory_[i].pts.x - px;
            double delta_y = trajectory_[i].pts.y - py;
            double dist = std::hypot(delta_x, delta_y);
            if (dist < min) 
            {
                min = dist;
                index_min = i;
            }
        }

        if (index_min < 0)
            return false;

        const double desired_velocity = param_.desire_vel;
        const int N = param_.N;
        const double dt = param_.dt;
        TrajectoryPoint tmp;
        double v = std::fmax(state_.v, param_.initial_velocity);

        double s_tmp = 0;

        for (int i = 0; i < N; i++) 
        {
            if (i != 0)
                s_tmp += v * dt;

            int index = int(s_tmp / param_.interval);

            int next = index + index_min > trajectory_.size() ? trajectory_.size() - 1 : index + index_min;

            double delta_x = trajectory_[next].pts.x - px;
            double delta_y = trajectory_[next].pts.y - py;
            double delta_yaw = trajectory_[next].yaw - psi;

            while ((delta_yaw) >= M_PI)
                delta_yaw -= M_PI * 2.0;
            while ((delta_yaw) <= -1.0 * M_PI)
                delta_yaw += M_PI * 2.0;

            double temp_x, temp_y;
            temp_x = delta_x * cos(psi) + delta_y * sin(psi);
            temp_y = delta_y * cos(psi) - delta_x * sin(psi);

            tmp.curvature = fabs(trajectory_[next].curvature);
            tmp.pts = cv::Point2f(temp_x, temp_y);
            tmp.yaw = delta_yaw;
            tmp.velocity = std::min(sqrt(param_.max_lat_acc / tmp.curvature), desired_velocity);
            refline.push_back(tmp);
        }
    }

}//namespace ns_path_generator
