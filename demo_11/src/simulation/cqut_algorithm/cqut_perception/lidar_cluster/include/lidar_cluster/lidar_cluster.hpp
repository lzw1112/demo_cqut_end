#pragma once

#include <geometry_msgs/msg/point32.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ns_lidar_cluster 
{

class LidarCluster 
{

public:
  // Constructor
  LidarCluster(rclcpp::Node::SharedPtr nh);

  // Getters
  sensor_msgs::msg::PointCloud getLidarCluster();

  bool is_ok() const;

  // Setters
  void setRawLidar(const sensor_msgs::msg::PointCloud2 &msg);

  void runAlgorithm();

private:
  rclcpp::Node::SharedPtr nh_;

  void loadParameters();

  bool getRawLidar_, is_ok_flag_;

  sensor_msgs::msg::PointCloud cluster_;

  sensor_msgs::msg::PointCloud2 raw_pc2_;

  sensor_msgs::msg::PointCloud2 filter_ground_, filter_cones_;

  pcl::PointCloud<pcl::PointXYZI> raw_pc_;

  void preprocessing(pcl::PointCloud<pcl::PointXYZI> &raw,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_ground,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_cones);
  void ClusterProcessing(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double threshold);
};
} // namespace ns_lidar_cluster
