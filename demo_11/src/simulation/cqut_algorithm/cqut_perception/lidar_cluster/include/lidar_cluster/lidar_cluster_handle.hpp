#pragma once

#include "lidar_cluster/lidar_cluster.hpp"

namespace ns_lidar_cluster 
{

class LidarClusterHandle 
{

public:
  // Constructor
  LidarClusterHandle(rclcpp::Node::SharedPtr nodeHandle);

//  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void run();
  void sendMsg();
//  void sendVisualization();

private:
  rclcpp::Node::SharedPtr nodeHandle_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rawLidarSubscriber_{nullptr};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr lidarClusterPublisher_{nullptr};
  // ros::Publisher filterGroundPublisher_;
  // ros::Publisher filterConesPublisher_;
  // ros::Publisher filterIntensityPublisher_;
  // ros::Publisher coneReconstructPublisher_;

  void rawLidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::string raw_lidar_topic_name_;
  std::string lidar_cluster_topic_name_;
  std::string filter_ground_topic_name_;
  std::string filter_cones_topic_name_;
  std::string filter_intensity_topic_name_;
  std::string cone_reconstruct_topic_name_;

  int node_rate_;

  LidarCluster lidar_cluster_;

};
}
