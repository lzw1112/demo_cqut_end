// TF Includes
#include <tf2/transform_datatypes.h>

// ROS Messages
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <geometry_msgs/msg/point32.hpp>

// CQUT_CAR MSGS
#include "cqut_msg/msg/car_state.hpp"
#include "cqut_msg/msg/car_state_dt.hpp"
#include "cqut_msg/msg/map.hpp"
#include "cqut_msg/msg/control_command.hpp"
#include "cqut_msg/msg/cone.hpp"
#include "cqut_msg/msg/cmd.hpp"
#include "cqut_msg/msg/state.hpp"
#include "cqut_msg/msg/track.hpp"

// ROS_To_PCL
#include <pcl_conversions/pcl_conversions.h>

namespace cqut
{

cqut_msg::msg::Cmd getFssimCmd(const cqut_msg::msg::ControlCommand &msg) 
{
    cqut_msg::msg::Cmd cmd;
    cmd.dc = msg.throttle.data;
    cmd.delta = msg.steering_angle.data;
    return cmd;
}

cqut_msg::msg::CarStateDt getStateDt(const nav_msgs::msg::Odometry &odom) 
{
    cqut_msg::msg::CarStateDt msg;
    msg.header = odom.header;
    msg.car_state_dt.x = odom.twist.twist.linear.x;
    msg.car_state_dt.y = odom.twist.twist.linear.y;
    msg.car_state_dt.theta = odom.twist.twist.angular.z;
    return msg;
}

cqut_msg::msg::CarStateDt getStateDt(const cqut_msg::msg::State &odom) 
{
    cqut_msg::msg::CarStateDt msg;
    msg.header = odom.header;
    msg.car_state_dt.x = odom.vx;
    msg.car_state_dt.y = odom.vy;
    msg.car_state_dt.theta = odom.r;
    msg.car_state_a.x = odom.ax;
    msg.car_state_a.y = odom.ay;
    msg.car_state_a.theta = odom.theta;
    return msg;
}

cqut_msg::msg::CarState getState(const cqut_msg::msg::State &odom) 
{
    cqut_msg::msg::CarState msg;
    msg.header = odom.header;
    msg.car_state.x = odom.x;
    msg.car_state.y = odom.y;
    msg.car_state.theta = odom.yaw;
    msg.car_state_dt = getStateDt(odom);
    return msg;
}

cqut_msg::msg::Cone getConeFromPoint(const geometry_msgs::msg::Point &p, const std::string &color) 
{
    cqut_msg::msg::Cone cone;
    cone.color.data = color;
    cone.position.x = p.x;
    cone.position.y = p.y;
    cone.position.z = p.z;
    return cone;
}

cqut_msg::msg::Map getMap(const cqut_msg::msg::Track &track) 
{
    cqut_msg::msg::Map msg;
    msg.header = track.header;
    msg.cone_red.clear();
    for (const geometry_msgs::msg::Point &c : track.cones_left) 
    {
        msg.cone_red.push_back(getConeFromPoint(c, "r"));
    }

    msg.cone_blue.clear();
    for (const geometry_msgs::msg::Point &c : track.cones_right) 
    {
        msg.cone_blue.push_back(getConeFromPoint(c, "b"));
    }
    return msg;
}

cqut_msg::msg::Map getLocalMap(const sensor_msgs::msg::PointCloud2 &cones) 
{
    cqut_msg::msg::Map msg;
    msg.header = cones.header;
    msg.cone_red.clear();
    msg.cone_blue.clear();

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(cones, cloud);

    geometry_msgs::msg::Point tmp;
    for (int i = 0; i < cloud.size(); i++) 
    {
        tmp.x = cloud[i].x;
        tmp.y = cloud[i].y;
        tmp.z = cloud[i].z;
        if (cloud[i].intensity < 0.1 && cloud[i].intensity > 0.005) 
        {
            //red
            msg.cone_red.push_back(getConeFromPoint(tmp, "r"));
        } 
        else if (cloud[i].intensity > 0.9) 
        {
            //blue
            msg.cone_blue.push_back(getConeFromPoint(tmp, "b"));
        }
    }
    return msg;
}

sensor_msgs::msg::PointCloud getLidarCluster(const sensor_msgs::msg::PointCloud2 &cones) 
{
    sensor_msgs::msg::PointCloud msg;
    msg.header = cones.header;

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(cones, cloud);

    geometry_msgs::msg::Point32 tmp;
    for (int i = 0; i < cloud.size(); i++) 
    {
        if (cloud[i].intensity > 0) 
        {
            tmp.x = cloud[i].x;
            tmp.y = cloud[i].y;
            tmp.z = cloud[i].z;
            msg.points.push_back(tmp);
        }
    }
    return msg;
}

}  // namespace cqut
