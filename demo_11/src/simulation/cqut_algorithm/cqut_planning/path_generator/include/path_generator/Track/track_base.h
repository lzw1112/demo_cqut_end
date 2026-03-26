#pragma once

#include "cqut_msg/msg/map.hpp"
#include "cqut_tools/cubic_spline.h"
#include "path_generator/Utils/types.h"
#include "path_generator/Utils/param.h"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/int8.hpp>

#include <vector>

namespace ns_path_generator 
{

    class Track 
    {
    public:
        Track() = default;

        ~Track() = default;

        virtual bool genTraj() = 0;

        virtual bool CalculateTraj(Trajectory &refline) = 0;

        void setMap(const cqut_msg::msg::Map &map);

        void setState(const VehicleState &state);

        void setTransMat(const Eigen::Matrix4f &transMat);

        void setEndPoint(const geometry_msgs::msg::Point &endPoint);

    protected:
        cqut_msg::msg::Map map_;
        VehicleState state_;
        Trajectory trajectory_;
        geometry_msgs::msg::Point endPoint_; // an endpoint for acceleration
        Eigen::Matrix4f transMat_;
    };

} // namespace ns_path_generator