#include "gazebo_ros_cone_plugin/include/gazebo_ros_cone.hpp"
#include "gazebo_utils/include/gazebo_utils.hpp"

#include <yaml-cpp/yaml.h>
#include <sophus/se3.hpp>
#include <pcl_ros/impl/transforms.hpp>

namespace YAML 
{

template<>
struct convert<gazebo_plugins::ConeSensorModel::Config> 
{
    static bool decode(const Node &node, gazebo_plugins::ConeSensorModel::Config &cType) 
    {
        cType.observation_radius             = node["observation_radius"].as<double>();
        cType.observation_likelihood_left    = node["observation_likelihood_left"].as<double>();
        cType.observation_likelihood_right   = node["observation_likelihood_right"].as<double>();
        cType.observation_likelihood_orange  = node["observation_likelihood_orange"].as<double>();
        cType.distance_dependent_observation = node["distance_dependent_observation"].as<double>();

        cType.topic_name = node["topic_name"].as<std::string>();

        cType.overwrite_transfer_to_frame = node["overwrite"]["enabled"].as<bool>();
        cType.transfer_to_frame           = node["overwrite"]["frame"].as<std::string>();

        cType.gaussian_noise_xy_mu    = node["gaussian_noise_xy_mu"].as<double>();
        cType.gaussian_noise_xy_sigma = node["gaussian_noise_xy_sigma"].as<double>();

        cType.likelihood_yellow = node["likelihood_yellow"].as<double>();
        cType.likelihood_blue   = node["likelihood_blue"].as<double>();
        cType.likelihood_orange = node["likelihood_orange"].as<double>();

        cType.distance_dependent_misclass = node["distance_dependent_misclass"].as<double>();

        cType.gaussian_noise_mu_angular    = node["gaussian_noise_mu_angular"].as<double>();
        cType.gaussian_noise_sigma_angular = node["gaussian_noise_sigma_angular"].as<double>();

        cType.gaussian_noise_mu_radial    = node["gaussian_noise_mu_radial"].as<double>();
        cType.gaussian_noise_sigma_radial = node["gaussian_noise_sigma_radial"].as<double>();

        cType.color_observation_radius = node["color_observation_radius"].as<double>();

        cType.cut_cones_below_x = node["cut_cones_below_x"].as<double>();

        cType.rate              = node["rate"].as<double>();
        cType.delay             = node["delay"]["time"].as<double>();
        cType.delay_noise_sigma = node["delay"]["noise_sigma"].as<double>();

        cType.print();
        return true;
    }
};
}

namespace gazebo_plugins
{

ConeSensorModel::ConeSensorModel(gazebo_ros::Node::SharedPtr _node) : nh_(_node), point_cloud_(), loaded_sacesfully_(false) 
{
    vehicle_frame_ = "base_link";
    buffer_ = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, _node);
}

bool ConeSensorModel::load(const gazebo::physics::ModelPtr &model, const sdf::ElementPtr &_sdf) 
{
    std::string yaml_file = _sdf->Get<std::string>("yaml_config");

    if (!yaml_file.empty()) 
    {
        const std::string node_name = _sdf->Get<std::string>("node_name");
        RCLCPP_INFO(nh_->get_logger(), "ConeSensorModel: Loading Config from YAML file: %s with node_name: %s", yaml_file.c_str(), node_name.c_str());

        YAML::Node yaml_config = YAML::LoadFile(yaml_file);
        config = yaml_config["sensors"][node_name].as<Config>();
    } 
    else 
    {
        RCLCPP_ERROR(nh_->get_logger(), "Loading from SDF");
        config.load(_sdf);
    }

    pub_cones_   = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(config.topic_name, 1);
    // pub_markers_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("/cone_sensor", 1);

    parent_model_ = model;
    model_        = model->GetWorld()->ModelByName("track");
    if (model_ == nullptr) { return false; }

    vehicle_frame_ = _sdf->Get<std::string>("base_link");

    loaded_sacesfully_ = true;
    return true;
}

void ConeSensorModel::update() 
{
    const double     sample_delay_noise = noise::getGaussianNoise(0.0, config.delay_noise_sigma);
    const double     delay_with_noise   = std::max(0.0, sample_delay_noise + config.delay);
    constexpr double eps                = 1e-3;
    const auto       time_now           = rclcpp::Clock().now() -  rclcpp::Duration(std::chrono::duration<double>(eps + delay_with_noise));

    Eigen::Vector3d pos;
    if (!getState(time_now, pos)) { return; }

    pcl_conversions::toPCL(time_now, point_cloud_.header.stamp);

    point_cloud_.points.clear();

    updateTrack();

    if (left_.empty() && right_.empty()) { return; }

    std::vector<Eigen::Vector3d> left_around;
    std::vector<Eigen::Vector3d> right_around;

    findObservedCones(pos, left_, config.observation_likelihood_left, config.likelihood_blue, point_cloud_, LEFT);
    findObservedCones(pos, right_, config.observation_likelihood_right, config.likelihood_yellow, point_cloud_, RIGHT);
    findObservedCones(pos, orange_, config.observation_likelihood_orange, config.likelihood_orange, point_cloud_, ORANGE);

    point_cloud_.header.frame_id = "map";
    point_cloud_.width           = 1;
    point_cloud_.height          = static_cast<uint32_t>(point_cloud_.points.size());

    PointCloud point_cloud_transformed;
    if (!pcl_ros::transformPointCloud(vehicle_frame_, time_now, point_cloud_, "map", point_cloud_transformed, *buffer_)) 
    {
    }

    point_cloud_transformed.header.frame_id = vehicle_frame_; // This is required because of pcl_ros bug

    addRadialNoise(point_cloud_transformed);

    point_cloud_transformed.erase(std::remove_if(point_cloud_transformed.begin(), point_cloud_transformed.end(),
                                                 [&](const Cone &c) {
                                                     return c.x < config.cut_cones_below_x;
                                                 }), point_cloud_transformed.end());

    if (config.overwrite_transfer_to_frame) { point_cloud_transformed.header.frame_id = config.transfer_to_frame; }

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(point_cloud_transformed, msg);
    pub_cones_->publish(msg);
}

bool ConeSensorModel::checkInit() 
{
    if (loaded_sacesfully_) { return true; }

    parent_model_->Reset();
    model_ = parent_model_->GetWorld()->ModelByName("track");
    if (model_ == NULL) 
    {
        RCLCPP_ERROR(nh_->get_logger(), "ConeSensorModel: DID NOT FIND MODEL");
        return false;
    }

    loaded_sacesfully_ = true;
    return true;
}

void ConeSensorModel::updateTrack() 
{
    if (!checkInit()) { return; }

    left_.clear();
    orange_.clear();
    right_.clear();

    for (unsigned int i = 0; i < model_->GetChildCount(); ++i) 
    {
        const auto child  = model_->GetChild(i);
        const auto entity = boost::dynamic_pointer_cast<gazebo::physics::Entity>(child);


        if (std::regex_match(entity->GetName(), std::regex("cone_left\\d+::link"))) 
        {
            left_.push_back(entity);
        } 
        else if (std::regex_match(entity->GetName(), std::regex("cone_right\\d+::link"))) 
        {
            right_.push_back(entity);
        } 
        else if (std::regex_match(entity->GetName(), std::regex("cone_orange\\d+::link")) || std::regex_match(entity->GetName(), std::regex("cone_orange_big\\d+::link"))) 
        {
            orange_.push_back(entity);
        }
    }
}

void ConeSensorModel::addNoise(Cone &c) 
{
    double theta  = std::atan2(c.y, c.x);
    double dr     = noise::getGaussianNoise(config.gaussian_noise_mu_radial, config.gaussian_noise_sigma_radial);
    double dtheta = noise::getGaussianNoise(config.gaussian_noise_mu_angular, config.gaussian_noise_sigma_angular);

    c.x += std::cos(theta) * dr;
    c.y += std::sin(theta) * dr;

    const double d = std::hypot(c.x, c.y);

    theta += dtheta;

    c.x = static_cast<float>(d * std::cos(theta));
    c.y = static_cast<float>(d * std::sin(theta));
}

void ConeSensorModel::addRadialNoise(PointCloud &cloud) 
{
    for (auto &point : cloud.points) 
    {
        addNoise(point);
    }
}

ConeColourProbability ConeSensorModel::computeColorProbability(const Eigen::Vector3d &cone,
                                                               double d,
                                                               double color_likelihood,
                                                               double likelihood_factor,
                                                               SIDE side) const 
{
    double lik_yellow, lik_blue, lik_orange = 0.0, lik_others = 0.0;

    if (d <= config.color_observation_radius) 
    {
        double color_lik;
        const bool right_color = noise::probability(color_likelihood * likelihood_factor, color_lik);
        if (right_color) 
        {
            lik_yellow = color_lik;
            lik_blue   = (1.0 - color_lik);
        } 
        else 
        {
            lik_yellow = lik_blue = 0.5;
        }

        switch (side) 
        {
            case LEFT: 
            {
                std::swap(lik_yellow, lik_blue);
                break;
            }
            case ORANGE: 
            {
                std::swap(lik_yellow, lik_orange);
                break;
            }
            default:
                break;
        }
    } 
    else 
    {
        const double d_after_color            = d - config.color_observation_radius;
        const double d_between_color_no_color = config.observation_radius - config.color_observation_radius;
        lik_others = -1.0 / d_between_color_no_color * d_after_color + 1;
        lik_yellow = lik_blue = lik_orange = (1.0 - lik_others) / 3.0;
    }
    return {static_cast<float>(lik_blue), static_cast<float>(lik_yellow), static_cast<float>(lik_orange),
            static_cast<float>(lik_others)};
}

void ConeSensorModel::findObservedCones(const Eigen::Vector3d &p,
                                        const std::vector<boost::shared_ptr<gazebo::physics::Entity>> &cones,
                                        double likelihood,
                                        double color_likelihood,
                                        PointCloud &obs,
                                        SIDE side) const 
{
    for (const auto &c: cones) 
    {
        const Eigen::Vector3d cone_position         = toVector(c);
        const double          d                     = (p - cone_position).norm();
        const double          obs_likelihood_factor = std::max(-1.0 / config.distance_dependent_observation * d + 1, 0.0);

        if (d <= config.observation_radius && observed(likelihood * obs_likelihood_factor)) 
        {
            const double likelihood_factor = std::max(-1.0 / config.distance_dependent_misclass * d + 1, 0.0);

            Eigen::Vector3d xy_noise = noise::gaussiaNoise3D(config.gaussian_noise_xy_mu, config.gaussian_noise_xy_sigma);
            const auto cone_pos_w_noise = cone_position + xy_noise;

            const auto probs = computeColorProbability(cone_pos_w_noise, d, color_likelihood, likelihood_factor, side);

            const double intensity = probs.probability_yellow;

            Cone cone = constructCone(static_cast<const float>(cone_pos_w_noise.x()),
                                    static_cast<const float>(cone_pos_w_noise.y()),
                                    0.0,
                                    static_cast<const float>(intensity),
                                    static_cast<const float>(config.gaussian_noise_xy_sigma),
                                    0.0,
                                    0.0,
                                    static_cast<const float>(config.gaussian_noise_xy_sigma),
                                    probs.probability_blue,
                                    probs.probability_yellow,
                                    probs.probability_orange,
                                    probs.probability_other);
            obs.points.push_back(cone);
        }
    }
}

void ConeSensorModel::print(const std::vector<Eigen::Vector2d> &vect) const 
{
    for (size_t idx = 0; idx < vect.size(); ++idx) 
    {
        RCLCPP_INFO_STREAM(nh_->get_logger(), idx << " : " << vect[idx].transpose());
    }
}

bool ConeSensorModel::getState(const rclcpp::Time &now, Eigen::Vector3d &p) const 
{
    geometry_msgs::msg::TransformStamped transform;
    try 
    {
        transform = buffer_->lookupTransform("map", vehicle_frame_, now);
    } 
    catch 
    (tf2::TransformException &ex) 
    {
        return false;
    }
    p = {transform.transform.translation.x, transform.transform.translation.y, 0.0};
    return true;

}

Eigen::Vector3d ConeSensorModel::transform(const std::string &from,
                                           const std::string &to,
                                           const Eigen::Vector3d &p) const {
    geometry_msgs::msg::TransformStamped transform;
    try 
    {
        transform = buffer_->lookupTransform(to, from, tf2::TimePointZero);
    }
    catch 
    (tf2::TransformException &ex) 
    {
        return {};
    }

    Sophus::SE3d T;
    T.setQuaternion(Eigen::Quaterniond(transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z));
    T.translation() = Eigen::Vector3d(transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    return T * p;
}

bool ConeSensorModel::observed(const double likelihood) const 
{
    double res;
    return noise::probability(likelihood, res);
}

ConeSensor::ConeSensor()
{
}

ConeSensor::~ConeSensor() 
{
}

void ConeSensor::Reset() 
{
    ModelPlugin::Reset();
}

void ConeSensor::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
{
    nh = gazebo_ros::Node::Get(_sdf);

    RCLCPP_INFO(nh->get_logger(), "Loading ConeSensor");
    model_ = _parent;

    track_ = ConeSensorModel(nh);

    gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gznode->Init();
    updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&ConeSensor::Update, this));

    track_.load(_parent, _sdf);
    dt_required_ = 1.0 / track_.getRate();
}
void ConeSensor::Update() 
{
    std::lock_guard<std::mutex> lock(mutex);
    gazebo::common::Time cur_time = model_->GetWorld()->SimTime();
    double dt;

    if (!isLoopTime(cur_time, dt)) 
    {
        return;
    }

    last_sim_time_ = cur_time;

    track_.update();
}

bool ConeSensor::isLoopTime(const gazebo::common::Time &time, double &dt) 
{
    dt = (time - last_sim_time_).Double();
    if (dt < 0) 
    {
        this->Reset();
        return false;
    } 
    else if (dt >= dt_required_) 
    {
        return true;
    }
    return false;
}

GZ_REGISTER_MODEL_PLUGIN(ConeSensor)

}