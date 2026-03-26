#pragma once

#include "fsd_control/Solver/solver_base.h"

namespace ns_control 
{

class Pure_Pursuit_Solver : public Solver 
{
public:
    void solve();
};

}; // namespace ns_control
