#include "cqut/ecu.h"
#include <rclcpp/rclcpp.hpp>

namespace cqut
{
    
Ecu::Ecu() : state_(EcuState::NOT_STARTED), car_state_stoped_(0.0)
{
}

void Ecu::mission_finished(const cqut_msg::msg::Mission &mission)
{
    if(mission.finished)
    {
        RCLCPP_INFO(rclcpp::get_logger("ECU"), "Discipline has been Finnished");
        state_ = EcuState::FINNISHED_DISCIPLINE;
    }
}

bool Ecu::request_stop()
{
    return state_ == EcuState::EMERGENCY_STATE;
}

void Ecu::start()
{
    state_ = EcuState::READY_TO_DRIVE;
}

void Ecu::update_state(const cqut_msg::msg::State &state)
{
    double cur_time = rclcpp::Clock().now().seconds();
    if(state.vx < 0.1)
    {
        if (car_state_stoped_ == 0.0)
        {
            car_state_stoped_ = cur_time;
        }
        else if (cur_time - car_state_stoped_ > 5.0 && state_ == EcuState::READY_TO_DRIVE)
        {
            state_ = EcuState::EMERGENCY_STATE;
            RCLCPP_WARN(rclcpp::get_logger("ECU"), "Emergency State, too long standing still");
        }
    }
    else
        car_state_stoped_ = 0.0;
}

} // namespace cqut
