#include "fsd_control/Solver/mpc_solver.h"
#include "fsd_control/Utils/param.h"
#include <rclcpp/rclcpp.hpp>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cmath>

namespace ns_control 
{

void MPC_Solver::solve() 
{
    const size_t N = control_param_.N;
    const double dt = control_param_.dt;
    const double Lf = control_param_.car_length;
    const int px_range_begin       = 0;
    const int py_range_begin       = px_range_begin    + N;
    const int psi_range_begin      = py_range_begin    + N;
    const int v_range_begin        = psi_range_begin   + N;
    const int cte_range_begin      = v_range_begin     + N;
    const int epsi_range_begin     = cte_range_begin   + N;
    const int steering_range_begin = epsi_range_begin  + N;
    const int throttle_range_begin = steering_range_begin + (N - 1);

    RCLCPP_INFO(rclcpp::get_logger("MPC"), "begin solve");
    if(trajectory_.empty()) 
    {
        RCLCPP_INFO(rclcpp::get_logger("MPC"), "trajectory empty");
        return;
    }

    double v      = state_.v;
    double delta  = state_.Delta;
    double a      = state_.a;

    double actuator_latency = 0;
    const double px = 0.0 + v * actuator_latency;
    const double py = 0.0;
    const double psi = 0.0 + v * (-delta) / Lf * actuator_latency;
    v = v + a * actuator_latency;


    int state_size = 6;
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    size_t n_vars = state_size * N + 2 * (N-1);

    size_t n_constraints = state_size * N;

    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++)
    {
        vars[i] = 0;
    }

    vars[px_range_begin]        = px;
    vars[py_range_begin]        = py;
    vars[psi_range_begin]       = psi;
    vars[v_range_begin]         = v;
    vars[cte_range_begin]       = cte;
    vars[epsi_range_begin]      = epsi;
    vars[steering_range_begin]  = 0.0;
    vars[throttle_range_begin]  = 0.0;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    for (int i = 0; i < steering_range_begin; i++)
    {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    for (int i = steering_range_begin; i < throttle_range_begin; i++)
    {
        vars_lowerbound[i] = -0.8;
        vars_upperbound[i] = 0.8;
    }

    for (int i = throttle_range_begin; i < n_vars; i++)
    {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = +1.0;
    }

    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0.0;
        constraints_upperbound[i] = 0.0;
    }

    constraints_lowerbound[px_range_begin]    = px;
    constraints_upperbound[px_range_begin]    = px;

    constraints_lowerbound[py_range_begin]    = py;
    constraints_upperbound[py_range_begin]    = py;

    constraints_lowerbound[psi_range_begin]  = psi;
    constraints_upperbound[psi_range_begin]  = psi;

    constraints_lowerbound[v_range_begin]    = v;
    constraints_upperbound[v_range_begin]    = v;

    constraints_lowerbound[cte_range_begin]  = cte;
    constraints_upperbound[cte_range_begin]  = cte;

    constraints_lowerbound[epsi_range_begin] = epsi;
    constraints_upperbound[epsi_range_begin] = epsi;

    std::string options;
    options += "Integer print_level  0\n";
    
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";

    options += "Numeric max_cpu_time          0.5\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    FG_eval fg_eval(trajectory_);
    
    CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    double cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;

    for (int i = 0; i < N; i++)
    {
        geometry_msgs::msg::Point32 p;
        p.x = solution.x[px_range_begin + i];
        p.y = solution.x[py_range_begin + i];
    }
    control_command_.steering_angle.data = solution.x[steering_range_begin];
    control_command_.throttle.data = solution.x[throttle_range_begin];
    RCLCPP_INFO(rclcpp::get_logger("MPC"), "throttle: %lf, steering: %lf", control_command_.steering_angle.data, control_command_.throttle.data);

    predictive_path.clear();
    TrajectoryPoint p_tmp;
    for (int i = 0; i < N; i++)
    {
        geometry_msgs::msg::Point32 p;
        p_tmp.pts.x = solution.x[px_range_begin + i];
        p_tmp.pts.y = solution.x[py_range_begin + i];
        p_tmp.velocity = solution.x[v_range_begin + i];
        predictive_path.push_back(p_tmp);
    }
}

}