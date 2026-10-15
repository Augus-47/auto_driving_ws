#pragma once

#include "EMPlanner_config.h"
#include "emplanner/reference_line/reference_line.hpp"
#include "emplanner/planner/trajectory.hpp"
#include <memory>

#include "emplanner/planner/path_time_graph.hpp"
#include "emplanner/planner/speed_time_graph.hpp"

class EMPlanner {
public:
  EMPlanner(const EMPlannerConfig &conf);
  ~EMPlanner() = default;

  void Plan(const TrajectoryPoint &planning_init_point, //规划起点
            const ReferenceLine &reference_line,        //参考线
            const LocalizationInfo &localization,       //定位信息
            const std::vector<ObstacleInfo> &obstacle,  //障碍物信息
            ReferenceLine *reference_line_planed); //虚拟障碍物

  // 1.计算规划起点
  void CalPlaningStartPoint(const Trajectory &pre_traj,
                            const LocalizationInfo &local_info,
                            TrajectoryPoint *start_plan_point,
                            Trajectory *stitch_traj);
  // 计算规划起点(简化版)
  void CalPlaningStartPoint(const std::vector<ReferencePoint>& pre_path,
                            const LocalizationInfo& local_info,
                            TrajectoryPoint* plan_start_point);

  void StitchTrajectory(const Trajectory &cur_traj,
                        const Trajectory &stitch_traj, Trajectory &final_traj);

  std::unique_ptr<PathTimeGraph> sl_graph_;
  std::unique_ptr<SpeedTimeGraph> st_graph_;

private:
  EMPlannerConfig config_;

  Trajectory pre_trajectory_;
  Trajectory trajectory_;

  // 1.参考线
  // 2.车辆的sl
  // 3.障碍物的sl
  // 4.采样点的sl
};
