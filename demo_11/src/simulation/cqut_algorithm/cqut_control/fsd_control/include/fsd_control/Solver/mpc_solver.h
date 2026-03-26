#pragma once

#include "fsd_control/Solver/solver_base.h"
#include "fsd_control/Utils/mpc.h"

namespace ns_control 
{

class MPC_Solver : public Solver 
{
public:
  void solve();

  double cte = 0.0;
	double epsi = 0.0;
};

}; // namespace ns_control
