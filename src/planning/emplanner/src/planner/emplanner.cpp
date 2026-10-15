#include "emplanner/planner/emplanner.hpp"
#include <math.h>


EMPlanner::EMPlanner(const EMPlannerConfig &conf) : config_(conf) {}

void EMPlanner::Plan(const TrajectoryPoint &planning_init_point,
                     const ReferenceLine &reference_line,
                     const LocalizationInfo &localization_info,
                     const std::vector<ObstacleInfo> &static_obstacles,
                     ReferenceLine *reference_line_planed) {

  // 根据当前参考线，匹配点信息初始化sl坐标系
  sl_graph_ = std::make_unique<PathTimeGraph>(reference_line, config_);

  // 设置 自车与障碍物 在sl的投影
  sl_graph_->SetStartPointSl(planning_init_point);
  sl_graph_->SetStaticObstaclesSl(static_obstacles);

  // 创建采样点网格(行数量，列数量，采样间隔s，采样间隔l)
  sl_graph_->CreateSamplingPoint(8, 10, 0.5, 0.5); // 8行5列，s间隔1m*5，l间隔0.5m*8
  
  // 动态规划得到路径点
  sl_graph_->PathDynamicPlanning();
  if(sl_graph_->dp_path_points().empty()) {
    std::cout << "动态规划插值后路径点数量为0，请检查路径规划逻辑。" << std::endl;
    return;
  }
  std::cout << "动态规划得到路径点数量: " << sl_graph_->dp_path_points().size() << std::endl;

  // 插值，共61个点(没有使用)，间隔0.2m
  sl_graph_->DpPathInterpolation(0.2); 
  
  std::cout << "插值后的数量: " << sl_graph_->dp_path_points_dense().size() << std::endl;
  auto diss = sl_graph_->dp_path_points_dense()[1].s - sl_graph_->dp_path_points_dense()[0].s;
  auto disl = sl_graph_->dp_path_points_dense()[1].l - sl_graph_->dp_path_points_dense()[0].l;
  double ds = std::sqrt(diss * diss + disl * disl);
  std::cout << "插值后的路径点间隔" << ds << std::endl;

  // 生成凸空间(静态障碍物)
  sl_graph_->GenerateConvexSpace();  //静态障碍物的长和宽

  // 二次规划优化路径点？
  auto result = sl_graph_->PathQuadraticProgramming();
  if(!result) {
    return;
  }

  // 12m,
  sl_graph_->QpPathInterpolation(30, 0.5);

  // 生成规划路径
  sl_graph_->GeneratePlaningPath();

  // 获取规划路径
  *reference_line_planed = sl_graph_->planning_path();
}

void EMPlanner::CalPlaningStartPoint(const std::vector<ReferencePoint>& pre_path,
                                    const LocalizationInfo& local_info,
                                    TrajectoryPoint* plan_start_point) {
  // 当前位置
  double x_cur = local_info.x;
  double y_cur = local_info.y;
  double heading_cur = local_info.heading;
  double vx_cur =
      local_info.vx * cos(heading_cur) + local_info.vy * sin(heading_cur);
  double vy_cur =
      local_info.vx * sin(heading_cur) + local_info.vy * cos(heading_cur);
  double ax_cur =
      local_info.ax * cos(heading_cur) + local_info.ay * sin(heading_cur);
  double ay_cur =
      local_info.ax * sin(heading_cur) + local_info.ay * cos(heading_cur);
  

  plan_start_point->kappa = 0;
  plan_start_point->v = local_info.v;
  plan_start_point->vx = vx_cur;
  plan_start_point->vy = vy_cur;
  plan_start_point->a = local_info.a;
  plan_start_point->ax = ax_cur;
  plan_start_point->ay = ay_cur;

  // 如果没有前序路径或这是首次规划，直接使用当前位置
  if (pre_path.empty()) {
    plan_start_point->x = x_cur;
    plan_start_point->y = y_cur;
    plan_start_point->heading = heading_cur;
    return;
  }
  
  // 在前序路径上找到最接近当前位置的点
  int closest_point_index = 0;
  double min_distance = std::numeric_limits<double>::max();
  
  for (size_t i = 0; i < pre_path.size(); ++i) {
    double dx = pre_path[i].x - x_cur;
    double dy = pre_path[i].y - y_cur;
    double distance = dx*dx + dy*dy;  // 平方距离，避免开方
    
    if (distance < min_distance) {
      min_distance = distance;
      closest_point_index = i;
    }
  }
  
  // 计算横纵向误差
  double closest_heading = pre_path[closest_point_index].heading;
  // 切向量和法向量
  double tx = cos(closest_heading);
  double ty = sin(closest_heading);
  double nx = -ty;
  double ny = tx;
  
  // 误差向量
  double dx = x_cur - pre_path[closest_point_index].x;
  double dy = y_cur - pre_path[closest_point_index].y;
  
  // 横纵向误差
  double lon_err = std::abs(dx * tx + dy * ty);
  double lat_err = std::abs(dx * nx + dy * ny);
  
  // 如果误差较小，选择前序路径上稍前方的点作为规划起点
  if (lon_err <= 2.0 && lat_err <= 0.5) {
    // 找前序路径上稍前方的点（例如前方0.5米-1米处）
    int forward_index = closest_point_index;
    double forward_distance = 0.0;
    const double target_forward_distance = 0.5;  // 前方0.5米
    
    for (size_t i = closest_point_index + 1; i < pre_path.size(); ++i) {
      double segment_dx = pre_path[i].x - pre_path[i-1].x;
      double segment_dy = pre_path[i].y - pre_path[i-1].y;
      double segment_length = std::sqrt(segment_dx*segment_dx + segment_dy*segment_dy);
      
      forward_distance += segment_length;
      if (forward_distance >= target_forward_distance) {
        forward_index = i;
        break;
      }
    }
    
    // 使用前方点作为规划起点
    plan_start_point->x = pre_path[forward_index].x;
    plan_start_point->y = pre_path[forward_index].y;
    plan_start_point->heading = pre_path[forward_index].heading;
    
  } else {
    // 误差较大，直接用当前位置作为规划起点
    plan_start_point->x = x_cur;
    plan_start_point->y = y_cur;
    plan_start_point->heading = heading_cur;
  }
}


// 计算规划起点，在plan之前就得到了
void EMPlanner::CalPlaningStartPoint(const Trajectory &pre_traj,
                                     const LocalizationInfo &local_info,
                                     TrajectoryPoint *plan_start_point,
                                     Trajectory *stitch_traj) {
  double x_cur = local_info.x;
  double y_cur = local_info.y;
  double heading_cur = local_info.heading;
  double kappa_cur = 0;
  //车身坐标系转换为世界坐标系
  double vx_cur =
      local_info.vx * cos(heading_cur) + local_info.vy * sin(heading_cur);
  double vy_cur =
      local_info.vx * sin(heading_cur) + local_info.vy * cos(heading_cur);
  double ax_cur =
      local_info.ax * cos(heading_cur) + local_info.ay * sin(heading_cur);
  double ay_cur =
      local_info.ax * sin(heading_cur) + local_info.ay * cos(heading_cur);
  //  %由于不是首次运行了，有上一时刻的轨迹了，找到上一周期轨迹规划的本周期车辆应该在的位置
  double cur_time = local_info.t; //当前时间等于定位信息的时间

  // 如果是首次运行，则为自车位置
  if (pre_traj.trajectory_points().empty()) {
    plan_start_point->x = local_info.x;
    plan_start_point->y = local_info.y;
    plan_start_point->heading = local_info.heading;
    plan_start_point->kappa = local_info.kappa;
    plan_start_point->v = local_info.v;
    plan_start_point->vx = local_info.vx;
    plan_start_point->vy = local_info.vy;
    plan_start_point->a = local_info.a;
    plan_start_point->ax = local_info.ax;
    plan_start_point->ay = local_info.ay;
    plan_start_point->t = local_info.t;
  } else {
    auto pre_traj_points = pre_traj.trajectory_points();
    int pre_index_desire = 0;
    for (int i = 0; i < pre_traj_points.size() - 1; i++) {
      if (cur_time >= pre_traj_points[i].t &&
          cur_time < pre_traj_points[i + 1].t) {
        pre_index_desire = i;
        break;
      }
    }
    //% 上一周期规划的本周期的车应该在的位置
    double pre_x_desire = pre_traj_points[pre_index_desire].x;
    double pre_y_desire = pre_traj_points[pre_index_desire].y;
    double pre_heading_desire = pre_traj_points[pre_index_desire].heading;
    //计算横纵向误差
    // 切向量
    auto tor = std::make_pair(cos(pre_heading_desire), sin(pre_heading_desire));
    //法向量
    auto nor =
        std::make_pair(-sin(pre_heading_desire), cos(pre_heading_desire));
    //% 误差向量
    auto d_err = std::make_pair(local_info.x - pre_x_desire,
                                local_info.y - pre_y_desire);
    //%纵向误差
    double lon_err = abs(d_err.first * tor.first + d_err.second * tor.second);
    //%横向误差
    double lat_err = abs(d_err.first * nor.first + d_err.second * nor.second);
    //%纵向误差大于2.5 横向误差大于0.5 认为控制没跟上
    double dt = 0.1; //时间间隔为0.1s一个点
    if ((lon_err > 2.5) || (lat_err > 0.5)) {
      // %此分支处理控制未跟上的情况，规划起点通过运动学递推，cur_time+0.1(下一时刻的车辆位置)
      //定位模块传递的速度是以车身为坐标系的，轨迹传递的速度是以世界为坐标系
      plan_start_point->x = x_cur + vx_cur * dt + 0.5 * ax_cur * dt * dt;
      plan_start_point->y = y_cur + vy_cur * dt + 0.5 * ay_cur * dt * dt;
      plan_start_point->vx = vx_cur + ax_cur * dt;
      plan_start_point->vy = vy_cur + ay_cur * dt;
      plan_start_point->heading =
          atan2(plan_start_point->vy, plan_start_point->vx);
      plan_start_point->ax = ax_cur;
      plan_start_point->ay = ay_cur;
      plan_start_point->kappa = kappa_cur;
      plan_start_point->t = cur_time + dt;
    } else {
      //找到cur+dt对应的轨迹点
      for (int i = 0; i < pre_traj_points.size() - 1; i++) {
        if (cur_time + dt >= pre_traj_points[i].t &&
            cur_time + dt < pre_traj_points[i + 1].t) {
          pre_index_desire = i;
          break;
        }
      }
      plan_start_point->x = pre_traj_points[pre_index_desire].x;
      plan_start_point->y = pre_traj_points[pre_index_desire].y;
      plan_start_point->heading = pre_traj_points[pre_index_desire].heading;
      plan_start_point->kappa = pre_traj_points[pre_index_desire].kappa;

      plan_start_point->vx =
          pre_traj_points[pre_index_desire].v * cos(plan_start_point->heading);
      plan_start_point->vy =
          pre_traj_points[pre_index_desire].v * sin(plan_start_point->heading);
      plan_start_point->heading =
          atan2(plan_start_point->vy, plan_start_point->vx);

      // 切向量
      tor = std::make_pair(cos(plan_start_point->heading),
                           sin(plan_start_point->heading));
      //法向量
      nor = std::make_pair(-sin(plan_start_point->heading),
                           cos(plan_start_point->heading));
      auto a_tor =
          std::make_pair(pre_traj_points[pre_index_desire].a * tor.first,
                         pre_traj_points[pre_index_desire].a * tor.second);
      auto a_nor = std::make_pair(
          pow(pre_traj_points[pre_index_desire].v, 2) *
              pre_traj_points[pre_index_desire].kappa * nor.first,
          pow(pre_traj_points[pre_index_desire].v, 2) *
              pre_traj_points[pre_index_desire].kappa * nor.second);
      plan_start_point->ax = a_nor.first + a_nor.first;
      plan_start_point->ay = a_nor.second + a_nor.second;

      plan_start_point->t = pre_traj_points[pre_index_desire].t;
      pre_index_desire--;
      if (pre_index_desire >= 20) {
        auto it_start = pre_traj_points.begin() + pre_index_desire - 19;
        auto it_end = pre_traj_points.begin() + pre_index_desire;
        std::vector<TrajectoryPoint> stitch_traj_points(it_start, it_end);
        stitch_traj->set_trajectory_points(stitch_traj_points);
      } else {
        stitch_traj->set_trajectory_points(pre_traj_points);
      }
    }
  }
}

// 2. 拼接轨迹
void EMPlanner::StitchTrajectory(const Trajectory &cur_traj,
                                 const Trajectory &stitch_traj,
                                 Trajectory &final_traj) {
  std::vector<TrajectoryPoint> cur_traj_points = cur_traj.trajectory_points();
  std::vector<TrajectoryPoint> stitch_traj_points =
      stitch_traj.trajectory_points();
  auto final_traj_points = stitch_traj_points.insert(
      stitch_traj_points.end(), cur_traj_points.begin(), cur_traj_points.end());
  final_traj.set_trajectory_points(stitch_traj_points);
}