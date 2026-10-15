#pragma once

#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Eigen"
#include <algorithm>
#include <float.h>
#include <vector>

#include "emplanner/common/pnc_point.hpp"
#include "emplanner/planner/EMPlanner_config.h"
#include "emplanner/reference_line/reference_line_provider.hpp"

class PathTimeGraph {
private:
  /* data */
public:
  // 1.初始化以参考线和车辆位置信息构建，sl坐标系。
  PathTimeGraph(const ReferenceLine &reference_line,  const EMPlannerConfig &emplaner_conf);
  ~PathTimeGraph() = default;

  static void Cartesian2Frenet(
    const ReferenceLine &reference_line,             //参考线
    const std::vector<SLPoint> &sl_reference_line,   //参考线的sl值
    TrajectoryPoint &points_wcs,  //待转换的点
    ReferencePoint &match_points, //匹配点
    ReferencePoint &project_points, //投影点
    SLPoint &points_fcs);

  // 自然坐标系转笛卡尔坐标系
  static void Frenet2Cartesian(const ReferenceLine &reference_line, //参考线
                   const std::vector<SLPoint> &sl_reference_line, //参考线的sl值
                   const std::vector<SLPoint> &points_fcs, //待转换的点
                   std::vector<ReferencePoint> *points_wcs);

  // 1.根据参考线和自车投影点生成s轴
  void InitSAxis(const ReferenceLine &reference_line, std::vector<SLPoint> *sl_reference_line);

  // 2.自车sl，记录自车位置。自车投影
  void SetStartPointSl(TrajectoryPoint plan_start_point); //存在问题
  
  // 3.障碍物sl，记录障碍物位置。障碍物投影
  void SetStaticObstaclesSl(const std::vector<ObstacleInfo> static_obstacles); //存在问题

  // 4.dp主算法相关生成采样点
  void CreateSamplingPoint(const int row, const int col, const double sample_s, const double sample_l);
  
  double CalcPathCost(SLPoint point1, SLPoint point2);
  
  void CalcQuinticCoeffient(const SLPoint &point1, const SLPoint &point2,
                            std::vector<double> *QuinticCoeffient);
  
  void PathDynamicPlanning();
  
  // 5.dp路径插值
  void DpPathInterpolation(double ds);

  const std::vector<SLPoint> dp_path_points() const; //动态规划路径点
  const std::vector<SLPoint> dp_path_points_dense() const; //动态规划加密路径点

  const std::vector<SLPoint> qp_path_points() const; //动态规划路径点
  const std::vector<SLPoint> qp_path_points_dense() const; //动态规划加密路径点

  // 二次规划相关，生成凸空间(上下边界)
  void GenerateConvexSpace(); //根据dp_path,输出l_minx,l_max
  
  int FindNearIndex(const std::vector<SLPoint> &dp_path_points_dense, double s);

  bool PathQuadraticProgramming();
  
  void QpPathInterpolation(int interpolation_num, double ds);

  void GeneratePlaningPath();

  void CalcProjPoint(const SLPoint sl_point,
                     std::vector<SLPoint> sl_reference_line,
                     const std::vector<ReferencePoint> reference_line,
                     ReferencePoint &project_point);

  const ReferenceLine planning_path() const;
  const SLPoint sl_plan_start() const;

private:
  EMPlannerConfig config_;

  ReferenceLine reference_line_;      //参考线，世界坐标系
  ReferencePoint host_match_point_;   //车辆在参考线的投影点
  ReferencePoint host_project_point_; //车辆在参考线的投影点
  std::vector<SLPoint> sl_reference_line_; // l均为0，s由车辆定位决定

  SLPoint sl_plan_start_;
  SLPoint sl_host_;                                 //自车的sl
  // TODO对于障碍物，可以添加障碍物的长宽等类型属性,或者将sl属性添加到原来obs的info中
  std::vector<SLPoint> sl_static_obstacles_;        //静态障碍物的sl
  std::vector<std::vector<SLPoint>> sample_points_; //采样点矩阵
  std::vector<SLPoint> dp_path_points_;             //动态规划路径点
  std::vector<SLPoint> dp_path_points_dense_; //动态规划加密路径点
  std::vector<SLPoint> qp_path_points_;       //二次规划路径点
  std::vector<SLPoint> qp_path_points_dense_; //增密后的二次规划路径点

  ReferenceLine planning_path_; //转换后的世界坐标系轨迹

  Eigen::VectorXd l_min_;
  Eigen::VectorXd l_max_;
};
