#include "line_detect/line_detector_handle.hpp"
//line检测
typedef ns_line_detector::LineDetectorHandle LineDetectorHandle;//命名空间ns_line_detector
//
int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nodeHandle = rclcpp::Node::make_shared("line_detector");
    LineDetectorHandle myLineDetectorHandle(nodeHandle);
    rclcpp::Rate loop_rate(myLineDetectorHandle.getNodeRate());//获取系统时间
    while (rclcpp::ok()) //检查ros节点是否可用
    {
        myLineDetectorHandle.run();//判读是否读到path,然后creatpath

        rclcpp::spin_some(nodeHandle);  //处理消息    
        loop_rate.sleep();          
    }
  return 0;
}

