#include "emplanner/planner/path_time_graph.hpp"

// 初始化,根据参考线生成sl
PathTimeGraph::PathTimeGraph(const ReferenceLine &reference_line,
                             const EMPlannerConfig &emplaner_conf) {
  reference_line_ = reference_line;
  config_ = emplaner_conf;

  //初始化根据参考线，以自车的投影点为原点，生产sl
  InitSAxis(reference_line, &sl_reference_line_);
}

// 初始化s轴，以投影点为原点
void PathTimeGraph::InitSAxis(const ReferenceLine &reference_line,
                              std::vector<SLPoint> *sl_reference_line) {

  // 获取参考线的相关信息(参考点、匹配点、投影点等)
  auto reference_points = reference_line.get_reference_points();
  auto host_match_index = reference_line.get_match_point_index();
  auto host_match_point = reference_line.get_host_match_point();
  auto host_project_point = reference_line.get_host_project_point();
  
  // 参数检查,防止匹配点越界
  int len = reference_points.size();
  if (len < 2 || host_match_index >= len - 1) {
    return;
  }
  
  sl_reference_line->resize(len);

  // 1. 计算参考线上每个点的累积弧长，为s(以路径起点为原点)
  (*sl_reference_line)[0].s = 0.0;
  for (int i = 1; i < len; i++) { // 注意这里修正了循环边界
    double dx = reference_points[i].x - reference_points[i - 1].x;
    double dy = reference_points[i].y - reference_points[i - 1].y;
    (*sl_reference_line)[i].s = std::hypot(dx, dy) + (*sl_reference_line)[i - 1].s;
  }
  
  // 2. 获取匹配点的s值
  double s_host_match = (*sl_reference_line)[host_match_index].s;
  
  // 3. 获取匹配点的下一个参考点
  ReferencePoint host_match_point_next = reference_points[host_match_index + 1];
  
  // 4. 计算投影点到匹配点的向量
  double dx_proj_match = host_project_point.x - host_match_point.x;
  double dy_proj_match = host_project_point.y - host_match_point.y;
  
  // 5. 计算匹配点到下一点的向量
  double dx_match_next = host_match_point_next.x - host_match_point.x;
  double dy_match_next = host_match_point_next.y - host_match_point.y;
  
  // 6. 通过点积判断投影点在匹配点的前后
  double dot_product = dx_proj_match * dx_match_next + dy_proj_match * dy_match_next;
  
  // 7. 计算投影点到匹配点的距离
  double proj_match_distance = std::hypot(dx_proj_match, dy_proj_match);
  
  // 8. 计算s0，即投影点的s坐标
  double s0;
  if (dot_product > 0) {
    // 投影点在匹配点前方，s增加
    s0 = s_host_match + proj_match_distance;
  } else {
    // 投影点在匹配点后方，s减少
    s0 = s_host_match - proj_match_distance;
  }

  // 9. 平移s坐标，使投影点为原点
  for (int i = 0; i < len; i++) {
    (*sl_reference_line)[i].s -= s0;
  }
}

/*-------------------------------------设置自车位置和障碍物部分-------------------------------------------------------------*/

// 设置规划起点的sl(并不一定是自车位置？？)
void PathTimeGraph::SetStartPointSl(TrajectoryPoint plan_start_point) {

  MapPoint map_point;
  map_point.x = plan_start_point.x;
  map_point.y = plan_start_point.y;

  //定义变量，用于存储计算结果
  ReferencePoint start_match_point, start_project_point;

  // 计算匹配点和投影点
  std::vector<MapPoint> map_points;
  ReferenceLineProvider::referce_to_map_point(reference_line_, map_points);
  int index = ReferenceLineProvider::FindMatchIndex(map_points, map_point, 0, 3);
  start_match_point =  reference_line_.get_reference_points()[index];
  ReferenceLineProvider::CalculateProjectPoint(reference_line_.get_reference_points(), index, map_point, start_project_point);

  // 将TrajectoryPoint转换为SL坐标
  auto start_point = plan_start_point;
  Cartesian2Frenet(reference_line_, sl_reference_line_, start_point,
                   start_match_point, start_project_point, sl_plan_start_);
}

// 设置静态障碍物sl(将障碍物点转换为SL坐标系)
void PathTimeGraph::SetStaticObstaclesSl(const std::vector<ObstacleInfo> static_obstacles) {
  std::vector<MapPoint> object_map_points;
  int size = static_obstacles.size();

  object_map_points.resize(size);
  for (int i = 0; i < size; i++) {
    object_map_points[i].x = static_obstacles[i].x;
    object_map_points[i].y = static_obstacles[i].y;
  }
  //定义变量，用于存储计算结果
  std::vector<ReferencePoint> obstacles_match_points, obstacles_project_points;
  obstacles_match_points.resize(size);
  obstacles_project_points.resize(size);

  // 参考线转化
  std::vector<MapPoint> map_points;
  ReferenceLineProvider::referce_to_map_point(reference_line_, map_points);

  for(int i = 0; i < size; i++) {
    // 计算匹配点和投影点
    int index = ReferenceLineProvider::FindMatchIndex(map_points, object_map_points[i], 0, 3);
    obstacles_match_points[i] = reference_line_.get_reference_points()[index];
    ReferenceLineProvider::CalculateProjectPoint(reference_line_.get_reference_points(), index, object_map_points[i], obstacles_project_points[i]);
  }

  //此部分比较妥协，暂时使用这种赋值循环，解决障碍物信息和参数不一致得到的问题
  std::vector<TrajectoryPoint> traj_points;
  traj_points.resize(size);
  for (int i = 0; i < size; i++) {
    traj_points[i].x = static_obstacles[i].x;
    traj_points[i].y = static_obstacles[i].y;
    traj_points[i].vx = static_obstacles[i].vx;
    traj_points[i].vy = static_obstacles[i].vy;
    traj_points[i].ax = static_obstacles[i].ax;
    traj_points[i].ay = static_obstacles[i].ay;
  }

  // 将障碍物点转换为SL坐标
  for (int i = 0; i < size; i++) {
    Cartesian2Frenet(reference_line_, sl_reference_line_,
                     traj_points[i], obstacles_match_points[i],
                      obstacles_project_points[i],
                     sl_static_obstacles_[i]);
  }
}

//--------------------------------------坐标变换-------------------------------------------------
void PathTimeGraph::Cartesian2Frenet(
    const ReferenceLine &reference_line,             //参考线
    const std::vector<SLPoint> &sl_reference_line,   //参考线的sl值
    TrajectoryPoint &points_wcs,  //待转换的点
    ReferencePoint &match_points, //匹配点
    ReferencePoint &project_points, //投影点
    SLPoint &points_fcs) //转换点对应SL
{
  // 获取参考线点
  auto reference_line_points = reference_line.get_reference_points();

    
  // 1. 匹配点与投影点的 差
  double dx_proj_match = project_points.x - match_points.x;
  double dy_proj_match = project_points.y - match_points.y;
    
  // 获取匹配点的下一个点的方向向量
  double dx_next = 0, dy_next = 0;
  if (match_points.index < reference_line_points.size() - 1) {
    dx_next = reference_line_points[match_points.index + 1].x - match_points.x;
    dy_next = reference_line_points[match_points.index + 1].y - match_points.y;
  } else {
    // 如果是最后一个点，使用前一个方向
    dx_next = match_points.x - reference_line_points[match_points.index - 1].x;
    dy_next = match_points.y - reference_line_points[match_points.index - 1].y;
  }
    
  // 点积判断投影点相对于匹配点的位置
  double dot_product = dx_proj_match * dx_next + dy_proj_match * dy_next;
    
  // 计算投影点到匹配点的距离
  double distance = std::hypot(dx_proj_match, dy_proj_match);
    
  // 根据方向确定 最终s 值
  if (dot_product > 0) {
    // 投影点在匹配点前方
    points_fcs.s = sl_reference_line[match_points.index].s + distance;
  } else {
    // 投影点在匹配点后方
    points_fcs.s = sl_reference_line[match_points.index].s - distance;
  }

  // 2. 计算横向偏移l
  double nx = -std::sin(project_points.heading); 
  double ny = std::cos(project_points.heading);
    
  // 计算当前位置点到投影点的向量
  double dx = points_wcs.x - project_points.x;
  double dy = points_wcs.y - project_points.y;
    
  // 横向偏移 = 点到投影点的向量 点乘 法向量
  points_fcs.l = dx * nx + dy * ny;

  // 3. 计算速度投影
  // 切向量 (沿路径方向)
  double tx = std::cos(project_points.heading);
  double ty = std::sin(project_points.heading);
    
  // 横向速度 = 速度向量 点乘 法向量
  points_fcs.dl_dt = points_wcs.vx * nx + points_wcs.vy * ny;
    
  // 计算曲率修正因子
  double curvature_factor = 1.0 - project_points.kappa * points_fcs.l;
  if (std::abs(curvature_factor) < 1e-6) {
    // 防止除以零
    curvature_factor = (curvature_factor > 0) ? 1e-6 : -1e-6;
  }
    
  // 纵向速度 = (速度向量 点乘 切向量) / 曲率修正因子
  points_fcs.ds_dt = (points_wcs.vx * tx + points_wcs.vy * ty) / curvature_factor;
  
  // 计算dl/ds
  if (std::abs(points_fcs.ds_dt) < 1e-6) {
    points_fcs.dl_ds = 0.0;
  } else {
    points_fcs.dl_ds = points_fcs.dl_dt / points_fcs.ds_dt;
  }
    
    // 4. 计算加速度变换
    // 修复之前的计算错误
  points_fcs.ddl_dt = points_wcs.ax * nx + points_wcs.ay * ny - 
                        project_points.kappa * points_fcs.ds_dt * points_fcs.ds_dt * 
                        (1.0 - project_points.kappa * points_fcs.l);
  
  points_fcs.dds_dt = (1.0 / curvature_factor) * 
                      ((points_wcs.ax * tx + points_wcs.ay * ty) + 
                        2.0 * project_points.kappa * points_fcs.dl_dt * points_fcs.ds_dt);
  
  // 计算ddl/ds
  if (std::abs(points_fcs.ds_dt) < 1e-5) {
    points_fcs.ddl_ds = 0.0;
  } else {
    points_fcs.ddl_ds = (points_fcs.ddl_dt - points_fcs.dds_dt * points_fcs.dl_ds) / 
                          (points_fcs.ds_dt * points_fcs.ds_dt);
  }
}

//自然坐标系转笛卡尔坐标系
void PathTimeGraph::Frenet2Cartesian(
    const ReferenceLine &reference_line,           //参考线
    const std::vector<SLPoint> &sl_reference_line, //参考线的sl值
    const std::vector<SLPoint> &points_fcs,        //待转换的点
    std::vector<ReferencePoint> *points_wcs) {}

/*----------------------------------------路径动态规划部分开始------------------------------------------------------------------------------------*/

// 动态规划得到路径点
void PathTimeGraph::PathDynamicPlanning() {
  int rows = sample_points_.size();
  std::cout << "动态规划采样点行数: " << rows << std::endl;
  int cols = sample_points_[0].size();
  std::cout << "动态规划采样点列数: " << cols << std::endl;
  
  // 1.计算起点到第一列所有点的cost
  for (int i = 0; i < rows; i++) {
    sample_points_[i][0].cost2start = CalcPathCost(sl_plan_start_, sample_points_[i][0]);
  }

  // 2.动态规划主结构
  for (int j = 1; j < cols; j++) //第j列
  {
    for (int i = 0; i < rows; i++) //第i行
    {
      // 计算起点到ij点的cost
      sample_points_[i][j].cost2start = DBL_MAX;
      for (int k = 0; k < rows; k++) {
        // 计算前一列每一点到当前点的cost，结合前一列点的cost2start，获得起点到当前点的cost，通过比较获取最小值
        // cost = 前一列当前行点的cost + 当前点-前一列所有点的cost
        double cost = sample_points_[k][j - 1].cost2start +
            CalcPathCost(sample_points_[k][j - 1], sample_points_[i][j]);// 
        
        // 更新最小cost
        if (cost < sample_points_[i][j].cost2start) {
          sample_points_[i][j].cost2start = cost;
          //  记录最优路径上一列的行号
          sample_points_[i][j].pre_mincost_row = k; 
        }
      }
    }
  }

  // 3.回溯路径
  std::vector<SLPoint> dp_path_points; // rows-1个+起点
  dp_path_points.resize(cols + 1);
  dp_path_points[0] = sl_plan_start_; //第一个点是起点
  //找到最后一列
  dp_path_points[cols] = sample_points_[0][cols - 1];
  for (int i = 1; i < rows; i++) {
    if (sample_points_[i][cols - 1].cost2start <
        dp_path_points[cols].cost2start)
      dp_path_points[cols] = sample_points_[i][cols - 1];
  }

  for (int j = cols - 1; j > 0; j--) {
    int pre_mincost_row = dp_path_points[j + 1].pre_mincost_row;
    dp_path_points[j] = sample_points_[pre_mincost_row][j - 1];
  }
  dp_path_points_ = dp_path_points; //赋值私有变量
}

// 计算路径代价(五次多项式连接 开始点-结束点)(碰撞，平滑，参考线)
double PathTimeGraph::CalcPathCost(SLPoint point_satrt, SLPoint point_end) {
  
  //计算两点之间五次多项式连接的代价
  // 1.计算五次多项式系数
  std::vector<double> qc;
  CalcQuinticCoeffient(point_satrt, point_end, &qc);
  
  //中间取num个点计算cost，加上终点num+1个
  int interpolation_num = 5; //插值点数
  //插值
  SLPoint quintic_path_point;
  double cost_smooth = 0;// 平滑代价
  double cost_ref = 0;// 参考线代价
  double cost_collision = 0;// 碰撞代价

  for (int i = 0; i < interpolation_num; i++) { //不包含起点包含终点
    //计算第i和点的s,l,dl,ddl,dddl
    quintic_path_point.s = point_satrt.s + (i + 1) * (point_end.s - point_satrt.s) / interpolation_num;
    quintic_path_point.l = qc[0] + qc[1] * quintic_path_point.s +
                           qc[2] * pow(quintic_path_point.s, 2) +
                           qc[3] * pow(quintic_path_point.s, 3) +
                           qc[4] * pow(quintic_path_point.s, 4) +
                           qc[5] * pow(quintic_path_point.s, 5);
    quintic_path_point.dl_ds = qc[1] + 2 * qc[2] * quintic_path_point.s +
                               3 * qc[3] * pow(quintic_path_point.s, 2) +
                               4 * qc[4] * pow(quintic_path_point.s, 3) +
                               5 * qc[5] * pow(quintic_path_point.s, 4);
    quintic_path_point.ddl_ds = 2 * qc[2] + 6 * qc[3] * quintic_path_point.s +
                                12 * qc[4] * pow(quintic_path_point.s, 2) +
                                20 * qc[5] * pow(quintic_path_point.s, 3);
    quintic_path_point.dddl_ds = 6 * qc[3] + 24 * qc[4] * quintic_path_point.s +
                                 60 * qc[5] * pow(quintic_path_point.s, 2);
    //平滑代价
    cost_smooth = cost_smooth +
                  config_.dp_cost_dl * pow(quintic_path_point.dl_ds, 2) +
                  config_.dp_cost_ddl * pow(quintic_path_point.ddl_ds, 2) +
                  config_.dp_cost_dddl * pow(quintic_path_point.dddl_ds, 2);
    
    //如果ddl或者切向角过大，则增加cost
    ////此处导致代价都很大
    if (abs(quintic_path_point.ddl_ds) > 0.5 || abs(atan(quintic_path_point.dl_ds)) > 0.4 * M_PI) {
      cost_smooth = cost_smooth + 10e7;
    }
    //参考线代价，l越大，代价越大
    cost_ref = config_.dp_cost_ref * pow(quintic_path_point.l, 2);

    //碰撞代价,这里做了非常简化的质点模型，认为障碍物就是一个点
    //求该点到障碍物的最小距离
    double min_d = DBL_MAX;
    for (const auto &obstacle : sl_static_obstacles_) //遍历静态障碍物
    {
      double dlon = quintic_path_point.s - obstacle.s;
      double dlat = quintic_path_point.l - obstacle.l;
      double square_d = pow(dlon, 2) + pow(dlat, 2);
      min_d = std::min(min_d, square_d);
    }

    // 距离大于3m，则碰撞代价为0，距离大于1m，则碰撞代价，距离小于1m，则碰撞代价为500/距离
    if (min_d > 9)
      cost_collision = cost_collision + 0;
    else if (min_d < 9 && min_d > 1)
      cost_collision = cost_collision + config_.dp_cost_collision * 500 / min_d;
    else
      cost_collision = cost_collision + config_.dp_cost_collision * 10e5;
  }
  // 三个代价不是一个数量级
  return cost_smooth + cost_ref + cost_collision;
}

// 计算五次多项式系数
void PathTimeGraph::CalcQuinticCoeffient(
    const SLPoint &point_s, const SLPoint &point_e,
    std::vector<double> *QuinticCoeffient) {
  double start_s = point_s.s;
  double start_s2 = pow(point_s.s, 2);
  double start_s3 = pow(point_s.s, 3);
  double start_s4 = pow(point_s.s, 4);
  double start_s5 = pow(point_s.s, 5);

  double end_s = point_e.s;
  double end_s2 = pow(point_e.s, 2);
  double end_s3 = pow(point_e.s, 3);
  double end_s4 = pow(point_e.s, 4);
  double end_s5 = pow(point_e.s, 5);

  Eigen::MatrixXd A(6, 6);
  A << 1, start_s, start_s2, start_s3, start_s4, start_s5, 0, 1, 2 * start_s,
      3 * start_s2, 4 * start_s3, 5 * start_s4, 0, 0, 2, 6 * start_s,
      12 * start_s2, 20 * start_s3, 1, end_s, end_s2, end_s3, end_s4, end_s5, 0,
      1, 2 * end_s, 3 * end_s2, 4 * end_s3, 5 * end_s4, 0, 0, 2, 6 * end_s,
      12 * end_s2, 20 * end_s3;
  Eigen::VectorXd B(6);
  B << point_s.l, point_s.dl_ds, point_s.ddl_ds, point_e.l, point_e.dl_ds,
      point_e.ddl_ds;

  Eigen::VectorXd coeff(6);
  coeff = A.inverse() * B;
  (*QuinticCoeffient).resize(coeff.size());
  for (int i = 0; i < coeff.size(); i++) {
    (*QuinticCoeffient)[i] = coeff[i];
  }
}

/**
 * 创建采样点网格
 * @param row 行数
 * @param col 列数
 * @param sample_s s方向采样间隔
 * @param sample_l l方向采样间隔
 */
void PathTimeGraph::CreateSamplingPoint(
    const int row, const int col, const double sample_s,
    const double sample_l) { 
  
  sample_points_.resize(row);// 多少行
  for (int j = 0; j < col; j++) {// 列
    for (int i = 0; i < row; i++) {// 行
      sample_points_[i].resize(col);// 每行多少列

      // 横向上s 只有在列j变化时，值才变
      sample_points_[i][j].s = sl_plan_start_.s + (j + 1) * sample_s;
      // 纵向上l，采样结果如上面所示
      sample_points_[i][j].l = ((row + 1) / 2 - (i + 1)) * sample_l;
      // 初始化dl_ds,ddl_ds，横向速度加速度
      sample_points_[i][j].dl_ds = 0;
      sample_points_[i][j].ddl_ds = 0;
    }
  }

}


// 插值，原来path上添加采样点
void PathTimeGraph::DpPathInterpolation(double ds) {
  std::vector<SLPoint> dp_path_points_dense;
  // //调试增加
  int size = dp_path_points_.size();// path路径点数量
  int dense_size = dp_path_points_.back().s / ds + 1; // 采样点数量


  for (int i = 0; i < size - 1; i++) {
    // 最终采样点的数量 小于 要求采样点数量
    if (dp_path_points_dense.size() < dense_size) {
      //对s进行采样,依次对每两点间隔ds进行采样
      SLPoint quintic_path_point;
      std::vector<double> qc;
      CalcQuinticCoeffient(dp_path_points_[i], dp_path_points_[i + 1], &qc);
      
      for (int j = 0; j < dense_size; j++) {
        quintic_path_point.s = dp_path_points_[i].s + ds * j;
        if (quintic_path_point.s < dp_path_points_[i + 1].s) {
          quintic_path_point.l = qc[0] + qc[1] * quintic_path_point.s +
                                 qc[2] * pow(quintic_path_point.s, 2) +
                                 qc[3] * pow(quintic_path_point.s, 3) +
                                 qc[4] * pow(quintic_path_point.s, 4) +
                                 qc[5] * pow(quintic_path_point.s, 5);
          quintic_path_point.dl_ds = qc[1] + 2 * qc[2] * quintic_path_point.s +
                                     3 * qc[3] * pow(quintic_path_point.s, 2) +
                                     4 * pow(quintic_path_point.s, 3) +
                                     5 * qc[5] * pow(quintic_path_point.s, 4);
          quintic_path_point.ddl_ds =
              2 * qc[2] + 6 * qc[3] * quintic_path_point.s +
              12 * qc[4] * pow(quintic_path_point.s, 2) +
              20 * qc[5] * pow(quintic_path_point.s, 3);
          dp_path_points_dense.push_back(quintic_path_point);
        } else
          break;
      }
    } else
      break;
  }
  dp_path_points_dense.push_back(dp_path_points_.back());
  dp_path_points_dense_ = dp_path_points_dense;
}

/*----------------------------------------路径动态规划结束------------------------------------------------------------------------------------*/

// 生成凸空间(根据静态障碍物产生上下边界)
void PathTimeGraph::GenerateConvexSpace() {
  // 0.初始化l_min,l_max为道路边界
  // 1.找到每个障碍物的s_head,s_end
  // 2.找到dp_path_s 中，与s_head s_end最近的s的编号。
  //如果是向左绕行，则设置l_min
  //如果向右绕行，则设置l_max
  int size = dp_path_points_dense_.size(); // 采样后的路径点数量

  // Eigen::VectorXd::Ones(size) 是一个静态成员函数，用于创建一个大小为 size 的向量，并将所有元素初始化为 1.0。
  // size 是向量的长度，表示向量中有多少个元素。
  Eigen::VectorXd l_min = Eigen::VectorXd::Ones(size) * -4;// 初始化l_min为-4
  Eigen::VectorXd l_max = Eigen::VectorXd::Ones(size) * 4; // 初始化l_max为4

  double obs_length = 1.0; // 静态障碍物长度
  double obs_width = 1.0; // 静态障碍物宽度

  // 遍历每个障碍物
  for (const auto &obs : sl_static_obstacles_) {

    // 计算障碍物在s方向的边界
    double obs_s_min = obs.s - obs_length / 2;
    double obs_s_max = obs.s + obs_width / 2;

    // 找到路径点中，与s_head s_end最近的s的编号。
    double start_index = FindNearIndex(dp_path_points_dense_, obs_s_min);
    double end_index = FindNearIndex(dp_path_points_dense_, obs_s_max);

    // 计算路径l方向中心点
    double path_center_l = (dp_path_points_dense_[start_index].l + dp_path_points_dense_[end_index].l) / 2;

    // 判断障碍物在路径中心点左侧还是右侧
    if (path_center_l > obs.l) { //左侧绕行
      for (int i = start_index; i <= end_index; i++) {
        l_min(i) = std::max(l_min(i), obs.l + obs_width / 2);
      }
    } else { //右侧绕行
      for (int i = start_index; i <= end_index; i++) {
        l_max(i) = std::min(l_max(i), obs.l - obs_width / 2);
      }
    }
  }

  l_min_ = l_min;
  l_max_ = l_max;
}

// 找到与障碍物s最近的路径点的索引
int PathTimeGraph::FindNearIndex(const std::vector<SLPoint> &dp_path_points_dense, double obs_s) {
  
  // 如果障碍物在路径起点之前，则返回0
  if (dp_path_points_dense.front().s > obs_s)
    return 0;
  // 如果障碍物在路径终点之后，则返回最后一个点
  if (dp_path_points_dense.back().s < obs_s)
    return dp_path_points_dense.size() - 1;
  double index = 0;
  for (index = 0; index < dp_path_points_dense.size(); index++) {// 找到第一个大于障碍物s的点
    if (dp_path_points_dense[index].s > obs_s)
      break;
  }

  // 如果障碍物在两个点之间，则返回较小的点
  if (dp_path_points_dense[index].s - obs_s > obs_s - dp_path_points_dense[index - 1].s)
    return index - 1;
  else
    return index;
}

// 二次规划优化路径点
bool PathTimeGraph::PathQuadraticProgramming() {
  std::cout << "开始二次规划优化路径点" << std::endl;
  
  int n = dp_path_points_dense_.size(); // 采样后的路径点数量
  double ds = dp_path_points_dense_[1].s - dp_path_points_dense_[0].s;// s采样间隔
  
  double d1 = 0.6; //汽车中心到后前边的距离
  double d2 = 0.8;
  double w = 0.8;

  //输出初始化
  Eigen::VectorXd qp_path_l = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qp_path_dl = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qp_path_ddl = Eigen::VectorXd::Zero(n);
  Eigen::VectorXd qp_path_s = Eigen::VectorXd::Zero(n);

  // Hissen矩阵初始化
  Eigen::SparseMatrix<double> H(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_L(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DL(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDL(3 * n, 3 * n); // n-1,3n
  Eigen::SparseMatrix<double> H_DDDL(3 * n, 3 * n);

  Eigen::SparseMatrix<double> H_CENTRE(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_L_END(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DL_END(3 * n, 3 * n);
  Eigen::SparseMatrix<double> H_DDL_END(3 * n, 3 * n);

  // grident矩阵 f
  Eigen::VectorXd f = Eigen::VectorXd::Zero(3 * n);

  //不等式约束  A*x <= b，dl ddl变化量约束
  Eigen::SparseMatrix<double> A(8 * n, 3 * n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(8 * n);

  //等式约束Aeq*x = beq,连续性约束
  Eigen::SparseMatrix<double> Aeq(2 * n - 2, 3 * n);
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(2 * n - 2);

  // lb ub初始化 lb <= x <= ub;
  Eigen::SparseMatrix<double> A_lu(3 * n, 3 * n);
  A_lu.setIdentity();
  Eigen::VectorXd lb = Eigen::VectorXd::Ones(3 * n) * -DBL_MAX;
  Eigen::VectorXd ub = Eigen::VectorXd::Ones(3 * n) * DBL_MAX;
  //行优先只能进行列操作
  Eigen::SparseMatrix<double> A_merge(8 * n + 2 * n - 2 + 3 * n, 3 * n);
  Eigen::VectorXd lb_merge(8 * n + 2 * n - 2 + 3 * n);
  Eigen::VectorXd ub_merge(8 * n + 2 * n - 2 + 3 * n);
  //---------------------------------------------H
  //生成H_L,H_DL,H_DDL,H_CENTRE
  // matlab矩阵索引从1开始，eigen矩阵索引从
  for (int i = 0; i < n; i++) {
    H_L.insert(3 * i, 3 * i) = 1;
    H_DL.insert(3 * i + 1, 3 * i + 1) = 1;
    H_DDL.insert(3 * i + 2, 3 * i + 2) = 1;
  }
  H_CENTRE = H_L;
  //生成H_DDDL
  Eigen::SparseMatrix<double> H_dddl_sub(1, 6);
  H_dddl_sub.insert(0, 2) = 1;
  H_dddl_sub.insert(0, 5) = -1;

  for (int i = 0; i < n - 1; i++) {
    int row = i;
    int col = 3 * i;
    //关于读操作，稀疏矩阵支持类似于密集矩阵相同的接口来访问块，列，行。
    //但由于性能的原因，稀疏子矩阵的写操作非常受限。当前仅列（列）优先稀疏矩阵的连续列（或行）可写。此外，在编译时必须知道这些信息。
    //  H_DDDL.block(row, col, 1, 6) = H_dddl_sub; //设置矩阵的块
    // H_DDDL.middleRows(row, col) = H_dddl_sub;
    H_DDDL.insert(row, col + 2) = 1;
    H_DDDL.insert(row, col + 5) = -1;
  }

  //% 生成H_L_END H_DL_END H_DDL_END
  H_L_END.insert(3 * n - 3, 3 * n - 3) = 1;
  H_DL_END.insert(3 * n - 2, 3 * n - 2) = 1;
  H_DDL_END.insert(3 * n - 1, 3 * n - 1) = 1;

  H = config_.qp_cost_l * (H_L.transpose() * H_L) +
      config_.qp_cost_dl * (H_DL.transpose() * H_DL) +
      config_.qp_cost_ddl * (H_DDL.transpose() * H_DDL) +
      config_.qp_cost_dddl * (H_DDDL.transpose() * H_DDDL) / ds +
      config_.qp_cost_centre * (H_CENTRE.transpose() * H_CENTRE) +
      config_.qp_cost_end_l * (H_L_END.transpose() * H_L_END) +
      config_.qp_cost_end_dl * (H_DL_END.transpose() * H_DL_END) +
      config_.qp_cost_end_ddl * (H_DDL_END.transpose() * H_DDL_END);
  H = 2 * H;

  //--------------------------------------生成f
  auto centre_line =
      0.5 * (l_min_ + l_max_); //此时centre line 还是60个点,起点不优化
  for (int i = 0; i < n; i++) {
    f(3 * i) = -2 * centre_line(i);
    //避免centreline权重过大影响轨迹平顺性
    if (abs(f(3 * i)) > 0.3)
      f(3 * i) = config_.qp_cost_centre * centre_line(i);
  }
  //% 期望的终点状态
  double end_l_desire = 0;
  double end_dl_desire = 0;
  double end_ddl_desire = 0;
  f(3 * n - 3) = f(3 * n - 3) - 2 * end_l_desire * config_.qp_cost_end_l;
  f(3 * n - 2) = f(3 * n - 2) - 2 * end_dl_desire * config_.qp_cost_end_dl;
  f(3 * n - 1) = f(3 * n - 1) - 2 * end_ddl_desire * config_.qp_cost_end_ddl;

  // Ax<=b
  //生成A 凸空间约束
  Eigen::MatrixXd A_sub(8, 3);

  A_sub << 1, d1, 0, 1, d1, 0, 1, -d2, 0, 1, -d2, 0, -1, -d1, 0, -1, -d1, 0, -1,
      d2, 0, -1, d2, 0;

  double row_index_start = 0;
  for (int i = 0; i < n - 1; i++) {
    int row = 8 * i;
    int col = 3 * i;
    //   A.block(row, col, 8, 3) = A_sub;
    A_merge.insert(row + 0, col + 0) = 1;
    A_merge.insert(row + 1, col + 0) = 1;
    A_merge.insert(row + 2, col + 0) = 1;
    A_merge.insert(row + 3, col + 0) = 1;
    A_merge.insert(row + 5, col + 0) = -1;
    A_merge.insert(row + 6, col + 0) = -1;
    A_merge.insert(row + 7, col + 0) = -1;
    A_merge.insert(row + 8, col + 0) = -1;

    A_merge.insert(row + 0, col + 1) = d1;
    A_merge.insert(row + 1, col + 1) = d1;
    A_merge.insert(row + 2, col + 1) = -d2;
    A_merge.insert(row + 3, col + 1) = -d2;
    A_merge.insert(row + 4, col + 1) = -d1;
    A_merge.insert(row + 5, col + 1) = -d1;
    A_merge.insert(row + 6, col + 1) = d2;
    A_merge.insert(row + 7, col + 1) = d2;
  }

  //生成b
  int front_index = ceil(d1 / ds);
  int back_index = ceil(d2 / ds);
  for (int i = 0; i < n; i++) {
    int index1 = std::min(i + front_index, n - 1); //车头索引
    int index2 = std::max(i - back_index, 0);      //车位索引
    Eigen::VectorXd b_sub(8);
    b_sub << l_max_(index1) - w / 2, l_max_(index1) + w / 2,
        l_max_(index2) - w / 2, l_max_(index2) + w / 2, -l_min_(index1) + w / 2,
        -l_min_(index1) - w / 2, -l_min_(index2) + w / 2,
        -l_min_(index2) - w / 2;
    b.block(8 * i, 0, 8, 1) = b_sub;
  }

  // Aeqx<=beq
  //生成Aeq,连续性约束
  Eigen::MatrixXd Aeq_sub(2, 6);
  Aeq_sub << 1, ds, pow(ds, 2) / 3, -1, 0, pow(ds, 2) / 6, 0, 1, ds / 2, 0, -1,
      ds / 2;

  row_index_start = 8 * n;
  for (int i = 0; i < n - 1; i++) {
    int row = row_index_start + 2 * i;
    int col = 3 * i;
    // Aeq.block(row, col, 2, 6) = A_sub;
    A_merge.insert(row, col) = 1;
    A_merge.insert(row, col + 1) = ds;
    A_merge.insert(row, col + 2) = pow(ds, 2) / 3;
    A_merge.insert(row, col + 3) = -1;
    A_merge.insert(row, col + 5) = pow(ds, 2) / 6;

    A_merge.insert(row + 1, col + 1) = 1;
    A_merge.insert(row + 1, col + 2) = ds / 2;
    A_merge.insert(row + 1, col + 4) = -1;
    A_merge.insert(row + 1, col + 5) = ds / 2;
  }

  // 3n*3n单位矩阵，起点约束，dl，ddl约束
  row_index_start = 8 * n + 2 * n - 2;
  for (int i = 0; i < 3 * n; i++) {
    A_merge.insert(row_index_start + i, i) = 1;
  }

  //% 生成 lb ub 主要是对规划起点做约束
  lb(0) = dp_path_points_dense_[0].l;
  lb(1) = dp_path_points_dense_[0].dl_ds;
  lb(2) = dp_path_points_dense_[0].ddl_ds;
  ub(0) = lb(0);
  ub(1) = lb(1);
  ub(2) = lb(2);

  for (int i = 1; i < n; i++) {
    lb(3 * i + 1) = -2;
    lb(3 * i + 2) = -0.1;
    ub(3 * i + 1) = 2;
    ub(3 * i + 2) = 0.1;
  }

  ub_merge.block(0, 0, 8 * n, 1) = b;
  ub_merge.block(8 * n, 0, 2 * n - 2, 1) = beq;
  ub_merge.block(8 * n + 2 * n - 2, 0, 3 * n, 1) = ub;

  lb_merge.block(0, 0, 8 * n, 1) = Eigen::MatrixXd::Ones(8 * n, 1) * (-DBL_MAX);
  lb_merge.block(8 * n, 0, 2 * n - 2, 1) =
      Eigen::MatrixXd::Ones(2 * n - 2, 1) * (-DBL_MAX);
  lb_merge.block(8 * n + 2 * n - 2, 0, 3 * n, 1) = lb;

  //-----------------------------------------
  // osqp的调用---------------------------------------------------------------
  OsqpEigen::Solver solver;

  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(3 * n); // A矩阵列数
  solver.data()->setNumberOfConstraints(8 * n + 2 * n - 2 + 3 * n); // A矩阵行数
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(f);
  solver.data()->setLinearConstraintsMatrix(A_merge);
  solver.data()->setLowerBound(lb_merge);
  solver.data()->setUpperBound(ub_merge);

  if (!solver.initSolver()){
    std::cout << "OSQP初始化失败" << std::endl;
    return false;
  }
  if (!solver.solve()){
    std::cout << "OSQP求解失败" << std::endl;
    return false;
  }
    
  Eigen::VectorXd qp_solution(3 * n);
  qp_solution = solver.getSolution();
  qp_path_points_ = dp_path_points_dense_;
  for (int i = 0; i < n; i++) {
    qp_path_points_[i].l = qp_solution(3 * i);
    qp_path_points_[i].dl_ds = qp_solution(3 * i + 1);
    qp_path_points_[i].ddl_ds = qp_solution(3 * i + 2);
  }
  std::cout << "二次规划优化路径点完成" << std::endl;
  std::cout << "优化后路径点数量：" << qp_path_points_.size() << std::endl;
  return true;
}

// 二次规划插值
void PathTimeGraph::QpPathInterpolation(int num, double ds) {
  std::vector<SLPoint> qp_path_points_dense;
  qp_path_points_dense.resize(num); // 初始化插值后的路径点
  qp_path_points_dense[0] = qp_path_points_[0]; 
  int n_init = qp_path_points_.size(); // 插值前路径点数量

  // 计算插值需要的间隔
  ds = (qp_path_points_.back().s - qp_path_points_.front().s) / (num - 1);
  
  int index = 0;
  double s = 0;
  for (int i = 1; i < num; i++) {
    s = qp_path_points_.front().s + i * ds;
    qp_path_points_dense[i].s = s; // 插值后的路径点s

    while (s >= qp_path_points_[index].s) {// 循环结束后，index为正在插值后一个点，
      index++;
      if (index == n_init)
        break;
    }

    //% while 循环退出的条件是x < qp_path_s(index)，所以x对应的前一个s的编号是index - 1 后一个编号是index
    int pre = index - 1;
    int cur = index;// 正在插值部分的索引
    
    double delta_s = s - qp_path_points_[pre].s;
    int jerk = (qp_path_points_[cur].ddl_ds - qp_path_points_[pre].ddl_ds) /
               (qp_path_points_[cur].s - qp_path_points_[pre].s);

    //分段加加速度优化，计算出现问题，以起点计算，与终点不重合
    qp_path_points_dense[i].l =
        qp_path_points_[pre].l +
        (qp_path_points_[cur].l - qp_path_points_[pre].l) /
            (qp_path_points_[cur].s - qp_path_points_[pre].s) * delta_s;
    qp_path_points_dense[i].dl_ds =
        qp_path_points_[pre].dl_ds +
        (qp_path_points_[cur].dl_ds - qp_path_points_[pre].dl_ds) /
            (qp_path_points_[cur].s - qp_path_points_[pre].s) * delta_s;
    qp_path_points_dense[i].ddl_ds =
        qp_path_points_[pre].ddl_ds +
        (qp_path_points_[cur].ddl_ds - qp_path_points_[pre].ddl_ds) * delta_s /
            (qp_path_points_[cur].s - qp_path_points_[pre].s);
    // % 因为此时x的后一个编号是index 必有x < qp_path_s(index),
    //在下一个循环中x = x + ds
    //也未必大于 % qp_path_s(index)，这样就进入不了while循环，所以index要回退一位
    index = index - 1;
  }

  qp_path_points_dense_ = qp_path_points_dense;
}

// 生成规划路径
void PathTimeGraph::GeneratePlaningPath() // frenet
{
  int size = qp_path_points_dense_.size();
  std::vector<ReferencePoint> planning_path_points;
  planning_path_points.resize(size);

  for (int i = 0; i < size; i++) {
    ReferencePoint project_point;
    CalcProjPoint(qp_path_points_dense_[i], sl_reference_line_,
                  reference_line_.get_reference_points(), project_point);
    Eigen::Vector2d nor(-sin(project_point.heading),
                        cos(project_point.heading));
    Eigen::Vector2d vector_proj(project_point.x, project_point.y);
    Eigen::Vector2d point = vector_proj + qp_path_points_dense_[i].l * nor;
    double heading =
        project_point.heading +
        atan(qp_path_points_dense_[i].dl_ds /
             (1 - project_point.kappa * qp_path_points_dense_[i].l));
    //% 近似认为 kappa' == 0,frenet转cartesian公式，见第一章第三节评论区的链接
    double kappa = ((qp_path_points_dense_[i].dddl_ds +
                     project_point.kappa * qp_path_points_dense_[i].dl_ds *
                         tan(heading - project_point.heading))) *
                   (pow(cos(heading - project_point.heading), 2) /
                        (1 - project_point.kappa * qp_path_points_dense_[i].l) +
                    project_point.kappa) *
                   cos(heading - project_point.heading) /
                   (1 - project_point.kappa * qp_path_points_dense_[i].l);
    planning_path_points[i].x = point(0);
    planning_path_points[i].y = point(1);
    planning_path_points[i].heading = heading;
    planning_path_points[i].kappa = kappa;
  }
  planning_path_.set_reference_points(planning_path_points);
}

void PathTimeGraph::CalcProjPoint(
    const SLPoint sl_point, std::vector<SLPoint> sl_reference_line,
    const std::vector<ReferencePoint> reference_line,
    ReferencePoint &project_point) {
  //   该函数将计算在frenet坐标系下，点(s, l)
  //   在frenet坐标轴的投影的直角坐标(proj_x, proj_y, proj_heading, proj_kappa)'
  // 1.寻找匹配点
  int match_index = 0;
  while (sl_reference_line[match_index].s < sl_point.s) {
    match_index = match_index + 1;
  }
  Eigen::Vector2d match_point(reference_line[match_index].x, reference_line[match_index].y);
  
  double match_point_heading = reference_line[match_index].heading;
  double match_point_kappa = reference_line[match_index].kappa;

  double ds = sl_point.s - sl_reference_line[match_index].s;
  Eigen::Vector2d match_tor(cos(match_point_heading), sin(match_point_heading));
  Eigen::Vector2d proj_point = match_point + ds * match_tor;
  double proj_heading = match_point_heading + ds * match_point_kappa;
  double proj_kappa = match_point_kappa;

  project_point.x = proj_point(0);
  project_point.y = proj_point(1);
  project_point.heading = proj_heading;
  project_point.kappa = proj_kappa;
}

const SLPoint PathTimeGraph::sl_plan_start() const { return sl_plan_start_; }

const ReferenceLine PathTimeGraph::planning_path() const {

  return planning_path_;
}

const std::vector<SLPoint> PathTimeGraph::dp_path_points() const {
  return dp_path_points_;
}; //动态规划路径点

const std::vector<SLPoint> PathTimeGraph::dp_path_points_dense() const {
  return dp_path_points_dense_;

}; //动态规划加密路径点

const std::vector<SLPoint> PathTimeGraph::qp_path_points() const {
  return qp_path_points_;
}; //二次规划路径点

const std::vector<SLPoint> PathTimeGraph::qp_path_points_dense() const {
  return qp_path_points_dense_;

}; //二次规划加密路径点