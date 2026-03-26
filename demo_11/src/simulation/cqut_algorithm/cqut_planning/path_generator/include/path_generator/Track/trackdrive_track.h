#pragma once

#include "path_generator/Track/track_base.h"

namespace ns_path_generator 
{

class Autox_Track : public Track 
{
public:
  bool genTraj();
  bool CalculateTraj(Trajectory &refline);
};

} // namespace ns_path_generator