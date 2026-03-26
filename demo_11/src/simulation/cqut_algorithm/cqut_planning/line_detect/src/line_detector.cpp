#include "line_detect/line_detector.hpp"
#include <sstream>

namespace ns_line_detector 
{
// Constructor
LineDetector::LineDetector(rclcpp::Node::SharedPtr nh) : nh_(nh) 
{
    nh_->declare_parameter("path_length", 80.0);
    nh_->declare_parameter("allow_angle_error", 1.0);

    if (!nh_->get_parameter("path_length", path_length)) 
    {
        RCLCPP_WARN(nh_->get_logger(), "Did not load path_length. Standard value is: %f", path_length);
    }
    if (!nh_->get_parameter("allow_angle_error", allow_angle_error)) 
    {
        RCLCPP_WARN(nh_->get_logger(), "Did not load allow_angle_error. Standard value is: %f", allow_angle_error);
    }
};

// Getters
geometry_msgs::msg::Point LineDetector::getendPoint() 
{ 
    return end_point; 
}

// Setters
void LineDetector::setlidarCluster(sensor_msgs::msg::PointCloud msgs) 
{
    cluster = msgs;
}

void LineDetector::runAlgorithm()
 {
    if(!getPath)
        createPath();
    else
        return;
}

void LineDetector::createPath() 
{
    if(cluster.points.size() == 0)
        return;
    int accumulator[180][201]={0};
    double p,p1,p2,Y_right,Y_left;
    int theta1,theta2;
    for(int i=0; i<cluster.points.size();i++)
    {
        if(cluster.points[i].y > 2 || cluster.points[i].y < -2)
            continue;
        for (int j=0; j<180; j++)
        {
            p=(cluster.points[i].x * cos(j * M_PI / 180)+cluster.points[i].y*sin(j * M_PI / 180))*5;
            if(p > 100)
                p = 100;
            accumulator[j][(int)p+100]+=1;            
       }
    }

    int max1 = 0;
    int max2 = 0;

    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)
    {
        for(int j = 0; j < 100; j++)
        {
            if(accumulator[i][j] >= max1)
            {
                max1 = accumulator[i][j];
                p1=((float)j-100)/5;
                theta1=i;
            }
        }
    }
   
    for(int i = 90 - allow_angle_error; i < 90 + allow_angle_error; i++)
    {
        for(int j = 100; j < 200; j++)
        {
            if(accumulator[i][j] >= max2)
            {
                max2 = accumulator[i][j];
                p2=((float)j-100)/5;
                theta2=i;
            }
        }
    }

    if (theta1==theta2)
	{
		if  (fabs(p1)<3 && fabs(p2)<3 )
        {
            getPath=true;
            RCLCPP_INFO(nh_->get_logger(), "find ideal path");
        }
	}
    else
    {
        double check_x=(p1*cos((float)theta2*M_PI/180.0)-p2*cos((float)theta1*M_PI/180.0))/(sin((float)theta1*M_PI/180.0)*cos((float)theta2*M_PI/180.0)-sin((float)theta2*M_PI/180.0)*cos((float)theta1*M_PI/180.0));
        if ((check_x > 200 || check_x < -200) && (fabs(p1) < 3 && fabs(p2) < 3))//直线距离车小鱼于3米
        {
            getPath=true;
			RCLCPP_INFO(nh_->get_logger(), "find path");
        }
        else
        {
            getPath=false;
            return;
        }
    }

    Y_right = (p1 - path_length * cos((float)theta1 * M_PI / 180.0)) / sin((float)theta1 * M_PI / 180.0);
    Y_left = (p2 - path_length * cos((float)theta2 * M_PI / 180.0)) / sin((float)theta2 * M_PI / 180.0);

    end_point.x = path_length;
    end_point.y = (Y_left + Y_right)/2;
}

}
