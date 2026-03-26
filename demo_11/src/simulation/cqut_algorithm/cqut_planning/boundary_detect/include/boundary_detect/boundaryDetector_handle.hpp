#pragma once

#include "boundary_detect/boundaryDetector.hpp"

namespace ns_boundaryDetector 
{

class BoundaryDetectorHandle 
{

public:
    // Constructor
    BoundaryDetectorHandle(rclcpp::Node::SharedPtr nodeHandle);

    // Getters
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
    rclcpp::Subscription<cqut_msg::msg::Map>::SharedPtr localMapSubscriber{nullptr};

    rclcpp::Publisher<cqut_msg::msg::Map>::SharedPtr boundaryDetectionsPublisher{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualTrianglesPublisher{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualBoundaryPublisher{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualTreePublisher{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr visualPathPublisher{nullptr};

  void localMapCallback(const cqut_msg::msg::Map::SharedPtr msg);

  std::string local_map_topic_name_;
  std::string boundary_detections_topic_name_;
  std::string visual_triangles_topic_name_;
  std::string visual_boundary_topic_name_;
  std::string visual_tree_topic_name_;
  std::string visual_path_topic_name_;

  int node_rate_;

  BoundaryDetector boundaryDetector_;

};
}
