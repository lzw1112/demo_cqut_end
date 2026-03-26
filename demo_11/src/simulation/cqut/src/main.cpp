#include "cqut/automated_res.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("automated_res");

    nh->declare_parameter("config", ament_index_cpp::get_package_share_directory("cqut") + "/config/simulation.yaml");
    nh->declare_parameter("sim_id", 0);

    cqut::AutomatedRes automated_res(nh, nh->get_parameter("config").as_string(), nh->get_parameter("sim_id").as_int());

    automated_res.run();

    
    rclcpp::shutdown();
    return 0;
}
