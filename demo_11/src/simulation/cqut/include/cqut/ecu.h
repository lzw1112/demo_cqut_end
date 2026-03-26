#pragma once

#include <cqut_msg/msg/mission.hpp>
#include <cqut_msg/msg/state.hpp>

namespace cqut
{
    
class Ecu
{
public:
    enum class EcuState
    {
        NOT_STARTED = 0,
        READY_TO_DRIVE = 1,
        FINNISHED_DISCIPLINE = 2,
        EMERGENCY_STATE = 3
    };

    Ecu();

    void mission_finished(const cqut_msg::msg::Mission &mission);
    bool request_stop();
    void start();
    void update_state(const cqut_msg::msg::State &state);
private:
    EcuState state_;
    double car_state_stoped_ = 0.0;
};

} // namespace cqut
