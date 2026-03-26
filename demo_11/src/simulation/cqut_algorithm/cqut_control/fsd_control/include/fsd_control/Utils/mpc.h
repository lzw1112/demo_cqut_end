#pragma once

#include "fsd_control/Utils/types.h"
#include <rclcpp/rclcpp.hpp>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cmath>

namespace ns_control 
{
  
using CppAD::AD;

class FG_eval 
{
    public:

    Trajectory refline_;

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

    FG_eval(Trajectory refline)
    {
        this->refline_ = refline;
    }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector& fg, const ADvector& vars)
    {

        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPC"), "fg: " << fg.size());
        RCLCPP_INFO_STREAM(rclcpp::get_logger("MPC"), "vars: " << vars.size());

        const double desired_cte = 0.0;
        const double desired_epsi = 0.0;

        const double cost_func_px_weight = control_param_.weight.px;
        const double cost_func_py_weight = control_param_.weight.py;
        const double cost_func_pyaw_weight = control_param_.weight.pyaw;

        const double cost_func_cte_weight = control_param_.weight.cte;
        const double cost_func_epsi_weight = control_param_.weight.epsi;
        const double cost_func_v_weight = control_param_.weight.v;
        const double cost_func_steer_weight = control_param_.weight.steer;
        const double cost_func_throttle_weight = control_param_.weight.throttle;
        const double cost_func_steer_rate_weight = control_param_.weight.steer_rate;
        const double cost_func_throttle_rate_weight = control_param_.weight.throttle_rate;

        fg[0] = 0.0;
        for (int t = 0; t < N; t++)
        {
            fg[0] += cost_func_px_weight * CppAD::pow(vars[px_range_begin + t] - refline_[t].pts.x, 2);
            fg[0] += cost_func_py_weight * CppAD::pow(vars[py_range_begin + t] - refline_[t].pts.y, 2);

            fg[0] += cost_func_cte_weight * CppAD::pow(vars[cte_range_begin + t]  - desired_cte,  2);
            fg[0] += cost_func_epsi_weight * CppAD::pow(vars[epsi_range_begin + t] - desired_epsi, 2);
            fg[0] += cost_func_v_weight * CppAD::pow(vars[v_range_begin + t] - refline_[t].velocity, 2);
        }

        for (int t = 0; t < N - 1; t++)
        {
            fg[0] += cost_func_steer_weight * cost_func_steer_weight * CppAD::pow(vars[steering_range_begin + t], 2);
            fg[0] += cost_func_throttle_weight * CppAD::pow(vars[throttle_range_begin + t], 2);
        }

        for (int t = 0; t < N - 2; t++)
        {
            fg[0] += cost_func_steer_rate_weight * (CppAD::pow(vars[steering_range_begin + t + 1] - vars[steering_range_begin + t], 2));
            fg[0] += cost_func_throttle_rate_weight * CppAD::pow(vars[throttle_range_begin + t + 1] - vars[throttle_range_begin + t], 2);
        }

        fg[1 + px_range_begin] = vars[px_range_begin];
        fg[1 + py_range_begin] = vars[py_range_begin];
        fg[1 + psi_range_begin] = vars[psi_range_begin];
        fg[1 + v_range_begin] = vars[v_range_begin];
        fg[1 + cte_range_begin] = vars[cte_range_begin];
        fg[1 + epsi_range_begin] = vars[epsi_range_begin];

        for (int i = 0; i < N - 1; i++) 
        {
          
            AD<double> x1 = vars[px_range_begin + i + 1];
            AD<double> y1 = vars[py_range_begin + i + 1];
            AD<double> psi1 = vars[psi_range_begin + i + 1];
            AD<double> v1 = vars[v_range_begin + i + 1];
            AD<double> cte1 = vars[cte_range_begin + i + 1];
            AD<double> epsi1 = vars[epsi_range_begin + i + 1];

            AD<double> x0 = vars[px_range_begin + i];
            AD<double> y0 = vars[py_range_begin + i];
            AD<double> psi0 = vars[psi_range_begin + i];
            AD<double> v0 = vars[v_range_begin + i];
            AD<double> cte0 = vars[cte_range_begin + i];
            AD<double> epsi0 = vars[epsi_range_begin + i];

            AD<double> delta0 = vars[steering_range_begin + i];
            AD<double> a0 = vars[throttle_range_begin + i];
            AD<double> f0 = refline_[i].pts.y;
            AD<double> psides0 = refline_[i].yaw;
            
            fg[2 + px_range_begin + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[2 + py_range_begin + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[2 + psi_range_begin + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[2 + v_range_begin + i] = v1 - (v0 + a0 * dt);
            fg[2 + cte_range_begin + i] =
            cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[2 + epsi_range_begin + i] =
            epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};
}