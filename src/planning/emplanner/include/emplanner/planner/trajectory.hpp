#pragma once
#include "emplanner/common/pnc_point.hpp"
#include <vector>

class Trajectory {

public:
  Trajectory(/* args */);
  ~Trajectory();
  
  const std::vector<TrajectoryPoint> trajectory_points() const;
  void set_trajectory_points(const std::vector<TrajectoryPoint> &trajectory_points);

private:
  /* data */
  std::vector<TrajectoryPoint> trajectory_points_;
};

