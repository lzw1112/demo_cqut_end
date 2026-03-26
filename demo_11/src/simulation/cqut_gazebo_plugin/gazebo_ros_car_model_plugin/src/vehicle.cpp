#include "gazebo_ros_car_model_plugin/include/vehicle.hpp"
#include "gazebo_utils/include/gazebo_utils.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace gazebo_plugins 
{

namespace cqut 
{

Vehicle::Vehicle(gazebo::physics::ModelPtr &_model,
                 sdf::ElementPtr &_sdf,
                 rclcpp::Node::SharedPtr nh,
                 gazebo::transport::NodePtr &gznode)
    : nh_(nh),
      model(_model),
      front_axle_(_model, _sdf, "front", gznode, nh),
      rear_axle_(_model, _sdf, "rear", gznode, nh),
      aero_(param_.aero) {

    // ROS Publishers
    pub_ground_truth_ = nh->create_publisher<cqut_msg::msg::State>("/cqut/base_pose_ground_truth", 1);
    pub_car_info_     = nh->create_publisher<cqut_msg::msg::CarInfo>("/cqut/car_info", 1);

    // ROS Subscribers
    sub_res_          = nh->create_subscription<cqut_msg::msg::ResState>("/cqut/res_state", 1, std::bind(&Vehicle::onRes, this, std::placeholders::_1));
    sub_cmd_          = nh->create_subscription<cqut_msg::msg::Cmd>("/cqut/cmd", 1, std::bind(&Vehicle::onCmd, this, std::placeholders::_1));
    sub_initial_pose_ = nh->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&Vehicle::onInitialPose, this, std::placeholders::_1));

    // TF
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

    // Initializatoin
    initModel(_sdf);
    initVehicleParam(_sdf);

    // Set Axle parameters
    front_axle_.setLeverArm(param_.kinematic.l, 1.0 - param_.kinematic.w_front, param_.kinematic.b_F);
    rear_axle_.setLeverArm(param_.kinematic.l, param_.kinematic.w_front, param_.kinematic.b_R);
    front_axle_.setParam(param_);
    rear_axle_.setParam(param_);

    setPositionFromWorld();

    time_last_cmd_ = 0.0;
}

void Vehicle::setPositionFromWorld() 
{
    auto       pos   = model->WorldPose();
    const auto vel   = model->WorldLinearVel();
    const auto accel = model->WorldLinearAccel();
    const auto r     = model->WorldAngularVel();

    state_.x   = -1;
    state_.y   = pos.Pos().Y();
    state_.yaw = pos.Rot().Yaw();
    state_.v_x = 0.0;
    state_.v_y = 0.0;
    state_.r   = 0.0;
    state_.a_x = 0.0;
    state_.a_y = 0.0;
}

void Vehicle::initModel(sdf::ElementPtr &_sdf) 
{

    std::string chassisLinkName = model->GetName() + "::" + _sdf->Get<std::string>("chassis");
    getLink(chassisLink, model, chassisLinkName);

    std::string baseLinkName = model->GetName() + "::" + _sdf->Get<std::string>("base_link");
    getLink(base_link_, model, baseLinkName);

    // then the wheelbase is the distance between the axle centers
    auto vec3 = front_axle_.getAxlePos() - rear_axle_.getAxlePos();
    param_.kinematic.l = vec3.Length();
}

void Vehicle::initVehicleParam(sdf::ElementPtr &_sdf) 
{
    robot_name_ = getParam<std::string>(_sdf, "robot_name");

    std::string yaml_name = "config.yaml";

    yaml_name = getParam(_sdf, "yaml_config", yaml_name);
    initParam(param_, yaml_name);

    yaml_name = getParam(_sdf, "yaml_sensors", yaml_name);
    initParamSensors(param_, yaml_name);
}

void Vehicle::publish(const double sim_time) 
{

}

void Vehicle::update(const double dt) 
{
    input_.dc = car_info_.torque_ok && rclcpp::Clock().now().seconds() - time_last_cmd_ < 1.0 ? input_.dc : -1.0;
    double Fz = getNormalForce(state_);

    // Tire Forces
    AxleTires FyF{}, FyR{}, alphaF{}, alphaR{};
    front_axle_.getFy(state_, input_, Fz, FyF, &alphaF);
    rear_axle_.getFy(state_, input_, Fz, FyR, &alphaR);
    front_axle_.setSteering(input_.delta);

    // Drivetrain Model
    const double Fx   = getFx(state_, input_);
    const double M_Tv = getMTv(state_, input_);

    // Dynamics
    const auto x_dot_dyn  = f(state_, input_, Fx, M_Tv, FyF, FyR);
    const auto x_next_dyn = state_ + x_dot_dyn * dt;
    state_ = f_kin_correction(x_next_dyn, state_, input_, Fx, M_Tv, FyF, FyR, dt);
    state_.validate();

    // Publish Everything
    setModelState(state_);
    publishTf(state_);

    // Overlay Noise on Velocities
    auto state_pub = state_.toRos(rclcpp::Clock().now());
    state_pub.vx += noise::getGaussianNoise(0.0, param_.sensors.noise_vx_sigma);
    state_pub.vy += noise::getGaussianNoise(0.0, param_.sensors.noise_vy_sigma);
    state_pub.r += noise::getGaussianNoise(0.0, param_.sensors.noise_r_sigma);
    pub_ground_truth_->publish(state_pub);
    publishCarInfo(alphaF, alphaR, FyF, FyR, Fx);
}

void Vehicle::onRes(const cqut_msg::msg::ResState::SharedPtr msg) 
{
    res_state_ = *msg;
    if (res_state_.push_button) { car_info_.torque_ok = static_cast<unsigned char>(true); }
    if (res_state_.emergency) { car_info_.torque_ok = static_cast<unsigned char>(false); }
    RCLCPP_INFO(rclcpp::get_logger("Res"), "torque_ok: %s", car_info_.torque_ok ? "true" : "false");
}

void Vehicle::printInfo() 
{
    front_axle_.printInfo();
    rear_axle_.printInfo();
}

State Vehicle::f(const State &x,
                 const Input &u,
                 const double Fx,
                 const double M_TV,
                 const AxleTires &FyF,
                 const AxleTires &FyR) 
{
    const double FyF_tot = FyF.left + FyF.right;
    const double FyR_tot = FyR.left + FyR.right;
    const double v_x     = std::max(1.0, x.v_x);

    const double m_lon = param_.inertia.m + param_.driveTrain.m_lon_add;

    State x_dot{};
    x_dot.x   = std::cos(x.yaw) * x.v_x - std::sin(x.yaw) * x.v_y;
    x_dot.y   = std::sin(x.yaw) * x.v_x + std::cos(x.yaw) * x.v_y;
    x_dot.yaw = x.r;
    x_dot.v_x = (x.r * x.v_y) + (Fx - std::sin(u.delta) * (FyF_tot)) / m_lon;
    x_dot.v_y = ((std::cos(u.delta) * FyF_tot) + FyR_tot) / param_.inertia.m - (x.r * v_x);
    x_dot.r   = ((std::cos(u.delta) * FyF_tot * param_.kinematic.l_F
                  + std::sin(u.delta) * (FyF.left - FyF.right) * 0.5 * param_.kinematic.b_F)
                 - ((FyR_tot) * param_.kinematic.l_R)
                 + M_TV) / param_.inertia.I_z;
    x_dot.a_x = 0;
    x_dot.a_y = 0;

    return x_dot;
}

std::ostream &operator<<(std::ostream &os, const State s) 
{
    os << s.getString();
    return os;
}

State Vehicle::f_kin_correction(const State &x_in,
                                const State &x_state,
                                const Input &u,
                                const double Fx,
                                const double M_TV,
                                const AxleTires &FyF,
                                const AxleTires &FyR,
                                const double dt) 
{
    State        x       = x_in;
    const double v_x_dot = Fx / (param_.inertia.m + param_.driveTrain.m_lon_add);
    const double v       = std::hypot(state_.v_x, state_.v_y);
    const double v_blend = 0.5 * (v - 1.5);
    const double blend   = std::fmax(std::fmin(1.0, v_blend), 0.0);

    x.v_x = blend * x.v_x + (1.0 - blend) * (x_state.v_x + dt * v_x_dot);

    const double v_y = std::tan(u.delta) * x.v_x * param_.kinematic.l_R / param_.kinematic.l;
    const double r   = std::tan(u.delta) * x.v_x / param_.kinematic.l;

    x.v_y = blend * x.v_y + (1.0 - blend) * v_y;
    x.r   = blend * x.r + (1.0 - blend) * r;

    x.a_x = (std::max(0.0, x.v_x) - std::max(0.0, x_state.v_x))/dt;
    x.a_y = (x.v_y - x_state.v_y)/dt;
    x.theta = (x.r - x_state.r)/dt;
    return x;
}

void Vehicle::publishTf(const State &x) 
{
    // Position
    geometry_msgs::msg::TransformStamped transform;
    transform.transform.translation.set__x(x.x);
    transform.transform.translation.set__y(x.y);
    transform.transform.translation.set__z(0.0);

    // Orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, x.yaw);
    transform.transform.rotation = tf2::toMsg(q);

    // Stamp
    transform.header.stamp = rclcpp::Clock().now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";

    // Send TF
    tf_br_->sendTransform(transform);
}

double Vehicle::getFx(const State &x, const Input &u) 
{
    const double dc = x.v_x <= 0.0 && u.dc < 0.0 ? 0.0 : u.dc;
    const double Fx = dc * param_.driveTrain.cm1 - aero_.getFdrag(x) - param_.driveTrain.cr0;
    return Fx;
}

double Vehicle::getMTv(const State &x, const Input &u) const 
{
    // const double shrinkage = param_.torqueVectoring.shrinkage;
    // const double K_stab    = param_.torqueVectoring.K_stability;
    // const double l         = param_.kinematic.l;

    // const double delta = u.delta;
    // const double v_x   = x.v_x;
  
    return 0.0;
}

void Vehicle::onCmd(const cqut_msg::msg::Cmd::SharedPtr msg) 
{
    input_.delta = msg->delta;
    input_.dc    = msg->dc;
    time_last_cmd_ = rclcpp::Clock().now().seconds();
}

void Vehicle::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
{
    state_.x   = msg->pose.pose.position.x;
    state_.y   = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double r{}, p{}, y{};
    tf2::Matrix3x3 m(q);
    m.getRPY(r, p, y);
    state_.yaw = y;

    state_.v_x = state_.v_y = state_.r = state_.a_x = state_.a_y = 0.0;
}

double Vehicle::getNormalForce(const State &x) 
{
    return param_.inertia.g * param_.inertia.m + aero_.getFdown(x);
}

void Vehicle::setModelState(const State &x) 
{
    const ignition::math::Pose3<double> pose(x.x, x.y, 0.0, 0, 0.0, x.yaw);
    const ignition::math::Vector3<double> vel(x.v_x, x.v_y, 0.0);
    const ignition::math::Vector3<double> angular(0.0, 0.0, x.r);
    model->SetWorldPose(pose);
    model->SetAngularVel(angular);
    model->SetLinearVel(vel);
}

double Vehicle::getGaussianNoise(double mean, double var) const 
{
    std::normal_distribution<double> distribution(mean, var);
    // construct a trivial random generator engine from a time-based seed:
    long                             seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine       generator(seed);
    return distribution(generator);
}

void Vehicle::publishCarInfo(const AxleTires &alphaF,
                             const AxleTires &alphaR,
                             const AxleTires &FyF,
                             const AxleTires &FyR,
                             const double Fx) const 
{
    // Publish Car Info
    cqut_msg::msg::CarInfo car_info;
    car_info.header.stamp = rclcpp::Clock().now();

    car_info.alpha_f   = alphaF.avg();
    car_info.alpha_f_l = alphaF.left;
    car_info.alpha_f_r = alphaF.right;

    car_info.fy_f   = FyF.avg();
    car_info.fy_f_l = FyF.left;
    car_info.fy_f_r = FyF.right;

    car_info.alpha_r   = alphaR.avg();
    car_info.alpha_r_l = alphaR.left;
    car_info.alpha_r_r = alphaR.right;

    car_info.fy_r   = FyR.avg();
    car_info.fy_r_l = FyR.left;
    car_info.fy_r_r = FyR.right;

    car_info.fx = Fx;
    pub_car_info_->publish(car_info);
}

} // namespace cqut
} // namespace gazebo_plugins
