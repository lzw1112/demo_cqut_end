#pragma once

#include "path_generator/Track/track_base.h"

namespace ns_path_generator 
{

class Skidpad_Track : public Track 
{
public:
  bool genTraj();
  bool CalculateTraj(Trajectory &refline);
  int now_state = 1;
};

} // namespace ns_path_generator