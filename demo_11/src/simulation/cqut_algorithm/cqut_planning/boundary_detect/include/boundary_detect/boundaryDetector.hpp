#pragma once

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <opencv2/imgproc.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "cqut_msg/msg/cone.hpp"
#include "cqut_msg/msg/map.hpp"
#include "boundary_detect/type.hpp"
#include "boundary_detect/visual_path.hpp"

namespace ns_boundaryDetector 
{

using ConePos = FSD::ConePos;
using PathPoint = FSD::PathPoint;
using SearchTree = FSD::SearchTree;
using Cost_index = FSD::Cost_index;
using Cost_n = FSD::Cost_n;
using Cost_p = FSD::Cost_p;

class BoundaryDetector 
{

 public:
  // Constructor
  BoundaryDetector(rclcpp::Node::SharedPtr nh);

	// Getters
  cqut_msg::msg::Map getboundaryDetections();
  visualization_msgs::msg::Marker getVisualTriangles();
  visualization_msgs::msg::MarkerArray getVisualBoundary();
  visualization_msgs::msg::MarkerArray getVisualTree();
  visualization_msgs::msg::Marker getVisualPath();

	// Setters
  void setLocalMap(cqut_msg::msg::Map msg);


  void runAlgorithm();

private:

	rclcpp::Node::SharedPtr nh_;
	
	cqut_msg::msg::Map map_current;
  cqut_msg::msg::Map map;

  visualization_msgs::msg::Marker visualTriangles;
  visualization_msgs::msg::MarkerArray visualTree;
  visualization_msgs::msg::MarkerArray visualBoundary;
  visualization_msgs::msg::Marker visualPath;
  cqut_msg::msg::Map boundaryDetections;


  double max_beam_cost_;
  int max_iter_num_, max_search_num_;
  Cost_n beam_weight_;
  Cost_p path_weight_;

  void loadParameters();
  bool filter(cqut_msg::msg::Map &init_map);
  void initSet(cqut_msg::msg::Map map, cv::Subdiv2D &coneSet, std::map<ConePos, char> &colorMap);
  void getMidPoint(cv::Subdiv2D coneSet, std::map<ConePos, char> colorMap, std::map<int, PathPoint> &MidSet);
  void searchPath(std::map<int, PathPoint> MidSet, SearchTree &Path);
  void selectBestPath(SearchTree Path, std::vector<PathPoint> &BestPath);
  void generateBoundary(std::vector<PathPoint> BestPath, cqut_msg::msg::Map &Boundary);
};
}
