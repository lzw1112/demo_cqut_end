#pragma once

#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo_plugins
{

struct ConeColourProbability
{
    float probability_blue;
    float probability_yellow;
    float probability_orange;
    float probability_other;
};

struct ConePositionUncertainty
{
    float covariance_xx;
    float covariance_xy;
    float covariance_yx;
    float covariance_yy;
};

struct Cone
{
    PCL_ADD_POINT4D;
    float intensity;
    ConePositionUncertainty cone_position_uncertainty;
    ConeColourProbability cone_colour_probability;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

inline Cone constructCone(const float x, const float y, const float z, const float intensity,
                          const float covariance_xx, const float covariance_xy, const float covariance_yx, const float covariance_yy,
                          const float probability_blue, const float probability_yellow, const float probability_orange, const float probability_other)
{
    Cone cone;

    cone.x = x;
    cone.y = y;
    cone.z = z;

    cone.intensity = intensity;

    cone.cone_position_uncertainty.covariance_xx = covariance_xx;
    cone.cone_position_uncertainty.covariance_xy = covariance_xy;
    cone.cone_position_uncertainty.covariance_yx = covariance_yx;
    cone.cone_position_uncertainty.covariance_yy = covariance_yy;

    cone.cone_colour_probability.probability_blue = probability_blue;
    cone.cone_colour_probability.probability_yellow = probability_yellow;
    cone.cone_colour_probability.probability_orange = probability_orange;
    cone.cone_colour_probability.probability_other = probability_other;

    return cone;
}

}

POINT_CLOUD_REGISTER_POINT_STRUCT(gazebo_plugins::Cone,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, cone_position_uncertainty.covariance_xx, covariance_xx)
                                          (float, cone_position_uncertainty.covariance_xy, covariance_xy)
                                          (float, cone_position_uncertainty.covariance_yx, covariance_yx)
                                          (float, cone_position_uncertainty.covariance_yy, covariance_yy)
                                          (float, cone_colour_probability.probability_blue, probability_blue)
                                          (float, cone_colour_probability.probability_yellow, probability_yellow)
                                          (float, cone_colour_probability.probability_orange, probability_orange)
                                          (float, cone_colour_probability.probability_other, probability_other)
)


namespace gazebo_plugins
{
class ConeSensorModel
{
public:

    enum SIDE 
    {
        LEFT   = 0,
        RIGHT  = 1,
        ORANGE = 2
    };

    struct Config 
    {
        double observation_radius;
        std::string topic_name;
        std::string transfer_to_frame;
        bool overwrite_transfer_to_frame;

        double observation_likelihood_left;
        double observation_likelihood_right;
        double observation_likelihood_orange;
        double distance_dependent_observation;

        double likelihood_yellow;
        double likelihood_blue;
        double likelihood_orange;

        double distance_dependent_misclass;

        double gaussian_noise_xy_mu;
        double gaussian_noise_xy_sigma;

        double gaussian_noise_mu_angular;
        double gaussian_noise_sigma_angular;

        double gaussian_noise_mu_radial;
        double gaussian_noise_sigma_radial;

        double color_observation_radius;

        double rate;

        double cut_cones_below_x;

        double delay;
        double delay_noise_sigma;

        void load(const sdf::ElementPtr &_sdf) 
        {
            topic_name = _sdf->Get<std::string>("topic_name");

            observation_radius = _sdf->Get<double>("observation_radius");

            observation_likelihood_left   = _sdf->Get<double>("observation_likelihood_left");
            observation_likelihood_right  = _sdf->Get<double>("observation_likelihood_right");
            observation_likelihood_orange = _sdf->Get<double>("observation_likelihood_orange");

            gaussian_noise_xy_mu    = _sdf->Get<double>("gaussian_noise_xy_mu");
            gaussian_noise_xy_sigma = _sdf->Get<double>("gaussian_noise_sigma");

            likelihood_yellow = _sdf->Get<double>("likelihood_yellow");
            likelihood_blue   = _sdf->Get<double>("likelihood_blue");
            likelihood_orange = _sdf->Get<double>("likelihood_orange");
            
            rate = _sdf->Get<double>("rate"); 
        }

        void print() const 
        {
            RCLCPP_INFO(rclcpp::get_logger("config"), "ConeSensorModel: ");
            RCLCPP_INFO(rclcpp::get_logger("config"), "topic_name: %s", topic_name.c_str());
            RCLCPP_INFO(rclcpp::get_logger("config"), "transfer_to_frame: %s", transfer_to_frame.c_str());

            RCLCPP_INFO(rclcpp::get_logger("config"), "observation_radius: %f", observation_radius);

            RCLCPP_INFO(rclcpp::get_logger("config"), "observation_likelihood_left: %f", observation_likelihood_left);
            RCLCPP_INFO(rclcpp::get_logger("config"), "observation_likelihood_right: %f", observation_likelihood_right);
            RCLCPP_INFO(rclcpp::get_logger("config"), "observation_likelihood_orange: %f", observation_likelihood_orange);

            RCLCPP_INFO(rclcpp::get_logger("config"), "gaussian_noise_xy_mu: %f", gaussian_noise_xy_mu);
            RCLCPP_INFO(rclcpp::get_logger("config"), "gaussian_noise_xy_sigma: %f", gaussian_noise_xy_sigma);

            RCLCPP_INFO(rclcpp::get_logger("config"), "likelihood_yellow: %f", likelihood_yellow);
            RCLCPP_INFO(rclcpp::get_logger("config"), "likelihood_blue: %f", likelihood_blue);
            RCLCPP_INFO(rclcpp::get_logger("config"), "likelihood_orange: %f", likelihood_orange);

            RCLCPP_INFO(rclcpp::get_logger("config"), "rate: %f", rate);
            RCLCPP_INFO(rclcpp::get_logger("config"), "delay: %f", delay);
            RCLCPP_INFO(rclcpp::get_logger("config"), "delay_noise_sigma: %f", delay_noise_sigma);

            RCLCPP_INFO(rclcpp::get_logger("config"), "color_observation_radius: %f", color_observation_radius);
        }

    };

    typedef pcl::PointCloud<Cone> PointCloud;

    ConeSensorModel() {}
    ConeSensorModel(gazebo_ros::Node::SharedPtr _node);

    bool load(const gazebo::physics::ModelPtr &model, const sdf::ElementPtr &_sdf);

    void update();

    double getRate() const { return config.rate; }

 private:

    void updateTrack();

    void findObservedCones(const Eigen::Vector3d &p,
                           const std::vector<boost::shared_ptr<gazebo::physics::Entity>> &cones,
                           double likelihood,
                           double color_likelihood,
                           PointCloud &obs,
                           SIDE side) const;

    ConeColourProbability computeColorProbability(const Eigen::Vector3d &cone,
                                                  double d,
                                                  double color_likelihood,
                                                  double likelihood_factor,
                                                  SIDE side) const;

    void print(const std::vector<Eigen::Vector2d> &vect) const;

    Eigen::Vector3d transform(const std::string &from, const std::string &to, const Eigen::Vector3d &p) const;

    bool getState(const rclcpp::Time &now, Eigen::Vector3d &p) const;
    bool observed(double likelihood) const;

    bool checkInit();

    void addRadialNoise(PointCloud &cloud);

    void addNoise(Cone &c);

 private:
    std::vector<boost::shared_ptr<gazebo::physics::Entity>> left_;
    std::vector<boost::shared_ptr<gazebo::physics::Entity>> right_;
    std::vector<boost::shared_ptr<gazebo::physics::Entity>> orange_;

    std::string vehicle_frame_;

    Config config;

    gazebo_ros::Node::SharedPtr nh_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cones_ = nullptr;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_ = nullptr;

    PointCloud point_cloud_;

    std::map<std::string, int> id_counter;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::ModelPtr parent_model_;

    std::shared_ptr<tf2_ros::Buffer> buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> listener_ = nullptr;

    bool loaded_sacesfully_;
};


class ConeSensor : public gazebo::ModelPlugin
{
public:
    ConeSensor();

    virtual ~ConeSensor();

    virtual void Reset();

    virtual void Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) override;

private:

    void Update();

    bool isLoopTime(const gazebo::common::Time &time, double &dt);

    gazebo_ros::Node::SharedPtr nh;

    std::mutex mutex;

    gazebo::event::ConnectionPtr updateConnection;

    gazebo::transport::NodePtr gznode;

    ConeSensorModel track_;

    gazebo::physics::ModelPtr model_;

    gazebo::common::Time last_sim_time_;
    double dt_required_;

};

}