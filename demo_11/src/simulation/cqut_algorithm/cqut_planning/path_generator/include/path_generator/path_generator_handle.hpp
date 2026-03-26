#pragma once

#include "path_generator/path_generator.hpp"

namespace ns_path_generator 
{

    class PathGeneratorHandle 
    {

    public:
        PathGeneratorHandle(rclcpp::Node::SharedPtr nodeHandle);

        int getNodeRate() const;

        void loadParameters();

        void subscribeToTopics();

        void publishToTopics();

        void run();

        void sendMsg();

    private:
        rclcpp::Node::SharedPtr nodeHandle_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr endPointSubscriber_{nullptr};
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr transMatSubscriber_{nullptr};
        rclcpp::Subscription<cqut_msg::msg::Map>::SharedPtr localMapSubscriber_{nullptr};
        rclcpp::Subscription<cqut_msg::msg::CarState>::SharedPtr carStateSubscriber_{nullptr};

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr refPathVisualPublisher_{nullptr};
        rclcpp::Publisher<cqut_msg::msg::Trajectory>::SharedPtr refPathPublisher_{nullptr};

        void endPointCallback(const geometry_msgs::msg::Point::SharedPtr msg);
        void transMatCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
        void carStateCallback(const cqut_msg::msg::CarState::SharedPtr msg);
        void localMapCallback(const cqut_msg::msg::Map::SharedPtr msg);


        std::string car_state_topic_name_;
        std::string transform_matrix_topic_name_;
        std::string end_point_topic_name_;
        std::string map_topic_name_;
        std::string ref_path_topic_name_;
        std::string path_generate_topic_name_;

        PathGenerator path_generator_;
        int node_rate_;
    };

}
