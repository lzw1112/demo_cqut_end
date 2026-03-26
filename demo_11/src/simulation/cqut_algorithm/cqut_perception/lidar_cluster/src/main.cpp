#include <rclcpp/rclcpp.hpp>
#include "lidar_cluster/lidar_cluster_handle.hpp"

typedef ns_lidar_cluster::LidarClusterHandle LidarClusterHandle;

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("lidar_cluster");
    LidarClusterHandle myLidarClusterHandle(nh);
    rclcpp::Rate loop_rate(myLidarClusterHandle.getNodeRate());
    while (rclcpp::ok()) 
    {
        myLidarClusterHandle.run();

        rclcpp::spin_some(nh);                // Keeps node alive basically
        loop_rate.sleep();                    // Sleep for loop_rate
    }
  return 0;
}

