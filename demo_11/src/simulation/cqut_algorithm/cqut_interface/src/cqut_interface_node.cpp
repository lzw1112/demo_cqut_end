// ROS Includes
#include <rclcpp/rclcpp.hpp>

// TF
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "cqut_interface/interface.hpp"

static rclcpp::Subscription<cqut_msg::msg::State>::SharedPtr sub_fssim_odom{nullptr};
static rclcpp::Subscription<cqut_msg::msg::Track>::SharedPtr sub_fssim_track{nullptr};
static rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fssim_obse{nullptr};
static rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_fssim_lidar{nullptr};

static rclcpp::Publisher<cqut_msg::msg::Cmd>::SharedPtr pub_fssim_cmd{nullptr};

static rclcpp::Subscription<cqut_msg::msg::ControlCommand>::SharedPtr sub_fsd_car_command{nullptr};

static rclcpp::Publisher<cqut_msg::msg::CarState>::SharedPtr pub_fsd_state{nullptr};
static rclcpp::Publisher<cqut_msg::msg::Map>::SharedPtr pub_fsd_map{nullptr};
static rclcpp::Publisher<cqut_msg::msg::Map>::SharedPtr pub_fsd_localmap{nullptr};
static rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_fsd_lidar_cluster{nullptr};

static bool tf_base_link = false;
static std::string cqut_vehicle;
static std::string origin;
static std::shared_ptr<tf2_ros::TransformBroadcaster> br{nullptr};

void callbackFssimOdom(const cqut_msg::msg::State::SharedPtr msg) 
{
    pub_fsd_state->publish(cqut::getState(*msg));
}

void callbackFssimTrack(const cqut_msg::msg::Track::SharedPtr msg) 
{
    pub_fsd_map->publish(cqut::getMap(*msg));
}


void callbackFssimLidar(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    pub_fsd_lidar_cluster->publish(cqut::getLidarCluster(*msg));
}

void callbackFssimCone(const sensor_msgs::msg::PointCloud2::SharedPtr msg) 
{
    pub_fsd_localmap->publish(cqut::getLocalMap(*msg));
}

void callbackFsdCmd(const cqut_msg::msg::ControlCommand::SharedPtr msg) 
{
    pub_fssim_cmd->publish(cqut::getFssimCmd(*msg));
}

template<class Type>
Type getParam(rclcpp::Node::SharedPtr nh, const std::string &name) 
{
    Type val;
    const bool success = nh->get_parameter(name, val);
    assert(success && "PARAMETER DOES NOT EXIST");
    return val;
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr n = rclcpp::Node::make_shared("cqut_interface");

    br = std::make_shared<tf2_ros::TransformBroadcaster>(n);

    n->declare_parameter("pub/cmd", "/cmd_vel");
    n->declare_parameter("pub/state", "/state");
    n->declare_parameter("pub/map", "/map");
    n->declare_parameter("pub/localmap", "/local_map");
    n->declare_parameter("pub/lidar_cluster", "/lidar_cluster");
    n->declare_parameter("sub/cmd", "/cmd_vel");
    n->declare_parameter("sub/topic_odom", "/odom");
    n->declare_parameter("sub/track", "/track");
    n->declare_parameter("sub/cones", "/cones");
    n->declare_parameter("sub/cluster", "/cluster");
    n->declare_parameter("tf/publish_car_base_link", true);
    n->declare_parameter("tf/origin", "map");
    n->declare_parameter("tf/base_link", "base_link");

    sub_fsd_car_command = n->create_subscription<cqut_msg::msg::ControlCommand>(n->get_parameter("sub/cmd").as_string(), 5, std::bind(callbackFsdCmd, std::placeholders::_1));

    rclcpp::QoS qos_pub(1);
    qos_pub.durability(rclcpp::DurabilityPolicy::TransientLocal);

    pub_fsd_state = n->create_publisher<cqut_msg::msg::CarState>(n->get_parameter("pub/state").as_string(), 1);
    pub_fsd_map = n->create_publisher<cqut_msg::msg::Map>(n->get_parameter("pub/map").as_string(), qos_pub);
    pub_fsd_localmap = n->create_publisher<cqut_msg::msg::Map>(n->get_parameter("pub/localmap").as_string(), qos_pub);
    pub_fsd_lidar_cluster = n->create_publisher<sensor_msgs::msg::PointCloud>(n->get_parameter("pub/lidar_cluster").as_string(), qos_pub);

    rclcpp::QoS qos_sub(5);
    qos_sub.durability(rclcpp::DurabilityPolicy::TransientLocal);

    sub_fssim_odom = n->create_subscription<cqut_msg::msg::State>(n->get_parameter("sub/topic_odom").as_string(), 5, std::bind(callbackFssimOdom, std::placeholders::_1));
    sub_fssim_track = n->create_subscription<cqut_msg::msg::Track>(n->get_parameter("sub/track").as_string(), qos_sub, std::bind(callbackFssimTrack, std::placeholders::_1));
    sub_fssim_obse = n->create_subscription<sensor_msgs::msg::PointCloud2>(n->get_parameter("sub/cones").as_string(), 5, std::bind(callbackFssimCone, std::placeholders::_1));
    sub_fssim_lidar = n->create_subscription<sensor_msgs::msg::PointCloud2>(n->get_parameter("sub/cluster").as_string(), 5, std::bind(callbackFssimLidar, std::placeholders::_1));

    pub_fssim_cmd = n->create_publisher<cqut_msg::msg::Cmd>(n->get_parameter("pub/cmd").as_string(), 1);

    tf_base_link = n->get_parameter("tf/publish_car_base_link").as_bool();
    origin = n->get_parameter("tf/origin").as_string();
    cqut_vehicle = n->get_parameter("tf/base_link").as_string();
    rclcpp::spin(n);
    return 0;
}
