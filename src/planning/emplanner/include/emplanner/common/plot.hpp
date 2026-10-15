#include "matplotlibcpp.h"
#include "emplanner/reference_line/reference_line.hpp"
#include "emplanner/planner/trajectory.hpp"

#include <map>
#include <vector>

namespace plt = matplotlibcpp;


class Plot{
    
public:
    Plot();
    ~Plot() = default;

    // 获取路径点
    Vector2D getRoutingPath(std::vector<MapPoint> routing_path_points);

    // 提取参考线
    Vector2D getReferenceLine(ReferenceLine reference_line);

    // 提取轨迹点
    Vector2D getTrajectoryPoints(Trajectory trajectory);

};