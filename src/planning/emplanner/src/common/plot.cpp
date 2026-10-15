#include "emplanner/common/plot.hpp"

namespace plt = matplotlibcpp;

Plot::Plot(){}

Vector2D Plot::getRoutingPath(std::vector<MapPoint> routing_path_points) {
  
  std::vector<double> x, y;
  for (const auto point : routing_path_points) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  return{x,y};
}

Vector2D Plot::getReferenceLine(ReferenceLine reference_line) {
  std::vector<double> x, y;
  for (const auto point : reference_line.get_reference_points()) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
  return{x, y};
}

Vector2D Plot::getTrajectoryPoints(Trajectory trajectory){
  std::vector<double> x, y;
  for (const auto &point : trajectory.trajectory_points()) {
    x.push_back(point.x);
    y.push_back(point.y);
  }
    return{x, y};
}