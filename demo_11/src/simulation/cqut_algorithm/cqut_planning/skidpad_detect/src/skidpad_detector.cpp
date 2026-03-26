#include "skidpad_detect/skidpad_detector.hpp"
#include <sstream>

namespace ns_skidpad_detector 
{

SkidpadDetector::SkidpadDetector(rclcpp::Node::SharedPtr nh) : nh_(nh) 
{
    getClusterFlag = false;
    matchFlag = false;
    loadParameters();
    loadFiles();
};

std_msgs::msg::Float64MultiArray SkidpadDetector::getTransMatrix() 
{
    trans_matrix_in_1D.data.clear();
    for (int i = 0; i < transformation.rows(); i++)
    {
        for (int j = 0; j < transformation.cols(); j++) 
        {
            trans_matrix_in_1D.data.push_back(transformation(i, j));
        }
    }
    return trans_matrix_in_1D;
}

// Setters
void SkidpadDetector::setclusterFiltered(sensor_msgs::msg::PointCloud msg) 
{
    getClusterFlag = true;
    cluster = msg;
}
void SkidpadDetector::loadParameters() 
{
    RCLCPP_INFO(nh_->get_logger(), "loading parameters");

    nh_->declare_parameter("path/skidpad_map", "/home/robot/ros2_ws/src/simulation/cqut_algorithm/cqut_planning/skidpad_detect/config/skidpad.pcd");
    nh_->declare_parameter("path/path_x", "/home/robot/ros2_ws/src/simulation/cqut_algorithm/cqut_planning/skidpad_detect/config/path_x.txt");
    nh_->declare_parameter("path/path_y", "/home/robot/ros2_ws/src/simulation/cqut_algorithm/cqut_planning/skidpad_detect/config/path_y.txt");
    nh_->declare_parameter("length/start", 15.0);
    nh_->declare_parameter("length/lidar2imu", 1.87);
    nh_->declare_parameter("length/threshold", 0.7);

    if (!nh_->get_parameter("path/skidpad_map", path_pcd_)) 
    {
        RCLCPP_WARN(nh_->get_logger(), "Did not load path/skidpad_map. Standard value is: %s", path_pcd_.c_str());
    }
    
    if (!nh_->get_parameter("path/path_x", path_x_)) 
    {
        RCLCPP_WARN(nh_->get_logger(),"Did not load path/path_x. Standard value is: %s", path_x_.c_str());
    }

    if (!nh_->get_parameter("path/path_y", path_y_)) 
    {
        RCLCPP_WARN(nh_->get_logger(),"Did not load path/path_y. Standard value is: %s", path_y_.c_str());
    }

    if (!nh_->get_parameter("length/start", start_length_)) 
    {
        RCLCPP_WARN(nh_->get_logger(),"Did not load start_length. Standard value is: %f", start_length_);
    }

    if (!nh_->get_parameter("length/lidar2imu", lidar2imu_)) 
    {
        RCLCPP_WARN(nh_->get_logger(),"Did not load lidar2imu. Standard value is: %s", lidar2imu_);
    }

    if (!nh_->get_parameter("length/threshold", threshold_)) 
    {
        RCLCPP_WARN(nh_->get_logger(),"Did not load length/threshold. Standard value is: %s", threshold_);
    }
}

void SkidpadDetector::loadFiles() 
{
    /* load pcd skidpad map */
    pcl::PointCloud<pcl::PointXYZ> source_cloud;
    geometry_msgs::msg::Point32 tmp_cloud;
    pcl::io::loadPCDFile (path_pcd_, source_cloud);
    RCLCPP_INFO(nh_->get_logger(), "load files");
    // The front is the x-axis, and the left is the y-axis
    for(int i = 0; i < source_cloud.points.size(); i++)
    {
        tmp_cloud.x = source_cloud.points[i].y + start_length_ + lidar2imu_;
        tmp_cloud.y = -source_cloud.points[i].x;
        skidpad_map.points.push_back(tmp_cloud);
    }

    /* load skidpad path */
    std::ifstream infile_x,infile_y;
    infile_x.open(path_x_);
    infile_y.open(path_y_);
    double path_x,path_y;

    while(!infile_x.eof() && !infile_y.eof())
    {
        infile_x>>path_x;
        infile_y>>path_y;
        geometry_msgs::msg::PoseStamped temp;
        temp.pose.position.x = path_x + lidar2imu_;
        temp.pose.position.y = path_y;
        standard_path.poses.push_back(temp);
    }
    infile_x.close();
    infile_y.close();
}

void SkidpadDetector::runAlgorithm() 
{
    if(!getClusterFlag || matchFlag)
        return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> Final;
    pcl::PointXYZ in_temp;
    pcl::PointXYZ out_temp;

    // find match points between skidpad map and cluster, match distance < threshold_
    for(int i = 0; i < skidpad_map.points.size(); i++)
    {
        double min_dist = std::numeric_limits<double>::infinity();
        int index = -1;
        for(int j = 0; j < cluster.points.size(); j++)
        {
            double dist = std::hypot(skidpad_map.points[i].x - cluster.points[j].x, skidpad_map.points[i].y - cluster.points[j].y);
            if(min_dist > dist) 
            {
                min_dist = dist;
                index = j;
            }
        }
        if(min_dist < threshold_) 
        {
            in_temp.x = skidpad_map.points[i].x;
            in_temp.y = skidpad_map.points[i].y;
            in_temp.z = 0;
            cloud_in->points.push_back(in_temp);

            out_temp.x = cluster.points[index].x;
            out_temp.y = cluster.points[index].y;
            out_temp.z = 0;
            cloud_out->points.push_back(out_temp);
        }
    }
    
    if(cloud_in->size() == 0 || cloud_out->size() == 0)
    {
        RCLCPP_WARN(nh_->get_logger(), "cloud size is zero!");
        return;
    }

    // icp match
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setMaxCorrespondenceDistance(1);  
    icp.setTransformationEpsilon(1e-10); 
    icp.setEuclideanFitnessEpsilon(0.001); 
    icp.setMaximumIterations(100);
    icp.align(Final);

    RCLCPP_INFO(nh_->get_logger(), "icp finish");

    RCLCPP_INFO_STREAM(nh_->get_logger(), "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore());
    transformation = icp.getFinalTransformation();
    RCLCPP_INFO_STREAM(nh_->get_logger(), transformation);
    RCLCPP_INFO_STREAM(nh_->get_logger(), "------------------------------------------------");

    matchFlag = true;
}

}
