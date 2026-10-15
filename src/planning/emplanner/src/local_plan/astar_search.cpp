
#include "emplanner/local_plan/astar_search.hpp"
#include "emplanner/local_plan/kinematic_bicycle_model.hpp"

#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

#include <limits>
#include <memory>
#include <queue>
#include <utility>
#include <algorithm>
#include <cmath>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <algorithm>
#include <vector>

// geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
// {
//   tf2::Quaternion q;
//   q.setRPY(0, 0, yaw);
//   return tf2::toMsg(q);
// }

double calcDistance2d(geometry_msgs::msg::Pose pose1, geometry_msgs::msg::Pose pose2){
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  return std::sqrt(dx * dx + dy * dy);
}

double calcReedsSheppDistance(const Pose & p1, const Pose & p2, double radius)
{
  const auto rs_space = ReedsSheppStateSpace(radius);
  const ReedsSheppStateSpace::StateXYT pose0{
    p1.position.x, p1.position.y, tf2::getYaw(p1.orientation)};
  const ReedsSheppStateSpace::StateXYT pose1{
    p2.position.x, p2.position.y, tf2::getYaw(p2.orientation)};
  return rs_space.distance(pose0, pose1);
}

Pose calcRelativePose(const Pose & base_pose, const Pose & pose)
{
  tf2::Transform tf_transform;
  tf2::convert(base_pose, tf_transform);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::msg::PoseStamped transformed;
  geometry_msgs::msg::PoseStamped pose_orig;
  pose_orig.pose = pose;
  tf2::doTransform(pose_orig, transformed, transform);

  return transformed.pose;
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param)
: AbstractPlanningAlgorithm(
    planner_common_param, std::make_shared<rclcpp::Clock>(RCL_ROS_TIME), collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  // 表示转向分辨率，体现了每次转向操作的最小变化量
  steering_resolution_ =collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  // 航向分辨率，它反映了航向角度的最小变化单位
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  // 计算平均转向角。它是转向分辨率与最大转向角和转向分辨率差值的一半之和。
  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  
  // 平均转弯半径
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  // 是否进行反向搜索
  is_backward_search_ = astar_param_.search_method == "backward";

  // 设置最小扩展距离和最大扩展距离
  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;

  // 接近目标的距离
  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

AstarSearch::AstarSearch(
  const PlannerCommonParam & planner_common_param, const VehicleShape & collision_vehicle_shape,
  const AstarParam & astar_param, const rclcpp::Clock::SharedPtr & clock)
: AbstractPlanningAlgorithm(planner_common_param, clock, collision_vehicle_shape),
  astar_param_(astar_param),
  goal_node_(nullptr),
  use_reeds_shepp_(true)
{
  steering_resolution_ =
    collision_vehicle_shape_.max_steering / planner_common_param_.turning_steps;
  heading_resolution_ = 2.0 * M_PI / planner_common_param_.theta_size;

  const double avg_steering =
    steering_resolution_ + (collision_vehicle_shape_.max_steering - steering_resolution_) / 2.0;
  avg_turning_radius_ =
    kinematic_bicycle_model::getTurningRadius(collision_vehicle_shape_.base_length, avg_steering);

  is_backward_search_ = astar_param_.search_method == "backward";

  min_expansion_dist_ = astar_param_.expansion_distance;
  max_expansion_dist_ = collision_vehicle_shape_.base_length * base_length_max_expansion_factor_;

  near_goal_dist_ =
    std::max(astar_param.near_goal_distance, planner_common_param.longitudinal_goal_range);
}

void AstarSearch::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  AbstractPlanningAlgorithm::setMap(costmap);

  // 确保路径规划时节点扩展的最小距离和最大距离满足一定条件
  min_expansion_dist_ = std::max(astar_param_.expansion_distance, 1.5 * costmap_.info.resolution);
  max_expansion_dist_ = std::max(
    collision_vehicle_shape_.base_length * base_length_max_expansion_factor_, min_expansion_dist_);
}

void AstarSearch::resetData()
{
  // 新的空优先队列.
  openlist_ = std::priority_queue<AstarNode *, std::vector<AstarNode *>, NodeComparison>();
  // 计算栅格地图里节点的数量nb_of_grid_nodes，它等于地图宽度乘以高度
  const int nb_of_grid_nodes = costmap_.info.width * costmap_.info.height;
  // 计算 A * 算法里所有节点的总数total_astar_node_count，即栅格节点数量乘以theta_size
  const int total_astar_node_count = nb_of_grid_nodes * planner_common_param_.theta_size;
  // 把graph_数组重置为指定大小，并将每个元素初始化为AstarNode的默认构造对象。
  graph_.assign(total_astar_node_count, AstarNode{});
  // 把col_free_distance_map_数组重置为指定大小，且将每个元素初始化为double类型的最大值，用来表示初始的无碰撞距离
  col_free_distance_map_.assign(nb_of_grid_nodes, std::numeric_limits<double>::max());
  shifted_goal_pose_ = {};
}

bool AstarSearch::makePlan(const Pose & start_pose, const Pose & goal_pose)
{
  // 重置数据
  resetData();

  // 全局转换为局部坐标系
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  // 碰撞检测
  if (detectCollision(start_pose_) || detectCollision(goal_pose_)) {
    throw std::logic_error("起点或终点存在障碍物");
  }

  // 反向搜索，则交换始末位置
  if (is_backward_search_) std::swap(start_pose_, goal_pose_);

  // 设置碰撞自由距离地图
  setCollisionFreeDistanceMap();

  // 多个目标
  is_multiple_goals_ = false;

  setStartNode(); // 设置开始节点属性

  // 开始规划
  if (!search()) {
    throw std::logic_error("HA* 规划失败");
  }

  return true;
}

// 设置碰撞自由距离地图
// 该地图的每个点表示从该点到目标点的无碰撞路径的估计距离
void AstarSearch::setCollisionFreeDistanceMap()
{
  // double的含义 ： 从目标点到当前坐标点的无碰撞路径的估计距离
  using Entry = std::pair<IndexXY, double>;
  struct CompareEntry
  {
    bool operator()(const Entry & a, const Entry & b) const { return a.second > b.second; }
  };
  std::priority_queue<Entry, std::vector<Entry>, CompareEntry> heap;
  std::vector<bool> closed(col_free_distance_map_.size(), false);

  // 目标位置索引，并设置为0
  auto goal_index = pose2index(costmap_, goal_pose_, planner_common_param_.theta_size);
  col_free_distance_map_[indexToId(goal_index)] = 0.0;
  
  // 将目标位置索引和 距离0.0 压入优先队列
  heap.push({IndexXY{goal_index.x, goal_index.y}, 0.0});

  Entry current;
  std::array<int, 3> offsets = {1, 0, -1};
  
  // 非空优先队列
  while (!heap.empty()) {
    
    // 取出优先队列的最小值（距离最近）
    current = heap.top();
    heap.pop();
    const int id = indexToId(current.first);// 当前点id
    
    // 如果当前点已经访问过，则跳过
    if (closed[id]) continue;
    closed[id] = true;

    const auto & index = current.first; // 当前点索引
    /**
     * 1  2  3
     * 4  x  5
     * 6  7  8
     * 搜索顺序为6，4，1，7，2，8，5，3
     * 
     */
    for (const auto & offset_x : offsets) {
      const int x = index.x + offset_x;
      for (const auto & offset_y : offsets) {
        const int y = index.y + offset_y;
        const IndexXY n_index{x, y};


        const double offset = std::abs(offset_x) + std::abs(offset_y);// 整体偏移量

        // 检查索引是否越界，是否为障碍物，偏移量是否小于1(自身)
        if (isOutOfRange(n_index) || isObs(n_index) || offset < 1) continue;

        // 如果障碍物距离小于车辆宽度的一半，跳过(会发生碰撞)
        if (getObstacleEDT(n_index).distance < 0.5 * collision_vehicle_shape_.width) continue;

        const int n_id = indexToId(n_index);// 偏移后的点id
        
        // 更新到终点距离(当前节点距离 + 偏移距离)
        const double dist = current.second + (sqrt(offset) * costmap_.info.resolution);
        
        // 偏移后的点已经访问过， 或者已经有更小的距离，跳过
        if (closed[n_id] || col_free_distance_map_[n_id] < dist) continue;

        // 更新碰撞自由距离地图的距离
        col_free_distance_map_[n_id] = dist;
        
        // 放入优先级队列中
        heap.push({n_index, dist});
      }
    }
  }
}

// 设置开始点属性 ， 如果无cost_offset，默认为0.0
void AstarSearch::setStartNode(const double cost_offset)
{
  // 开始点索引（xyt）格式
  const auto index = pose2index(costmap_, start_pose_, planner_common_param_.theta_size);
  
  // 设置开始节点
  AstarNode * start_node = &graph_[getKey(index)];

  // 主要根据 碰撞自由距离地图 进行加权
  const double initial_cost = estimateCost(start_pose_, index) + cost_offset;

  // 设置开始节点的属性(pose,         move_cost,    total_cost,   steer_ind,        backward )
  //                 (x, y, theta,  gc,           fc,           steering_index,   is_back）
  start_node->set(start_pose_, 0.0, initial_cost, 0, false);

  start_node->dir_distance = 0.0;                                     // 
  start_node->dist_to_goal = calcDistance2d(start_pose_, goal_pose_); // 到目标点的距离
  start_node->dist_to_obs = getObstacleEDT(index).distance;           // 到障碍物的距离
  start_node->status = NodeStatus::Open;                              // 节点状态
  start_node->parent = nullptr;                                       // 父节点为空

  openlist_.push(start_node);
}

// 估计cost 通过碰撞自由距离地图作为总体代价的启发式估计
double AstarSearch::estimateCost(const Pose & pose, const IndexXYT & index) const
{
  // 碰撞自由距离地图的距离
  double total_cost = col_free_distance_map_[indexToId(index)];
  // 加权
  return astar_param_.distance_heuristic_weight * total_cost;
}

// astar搜索算法
bool AstarSearch::search()
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  // 开始 A* search，直到openlist_空（初始只有开始点）
  while (!openlist_.empty()) {
    
    // 规划时间限制检查
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;
    if (msec > planner_common_param_.time_limit) {
      return false;
    }

    AstarNode * current_node = openlist_.top();// 取出优先队列的最小值（cost最小）
    openlist_.pop();

    // 如果当前节点已经访问过，则跳过
    if (current_node->status == NodeStatus::Closed) continue;
    current_node->status = NodeStatus::Closed;

    // 是否到达（接近）
    if (isGoal(*current_node)) {
      goal_node_ = current_node;
      setPath(*current_node);
      return true;
    }

    // 扩展节点
    expandNodes(*current_node);
  }

  // Failed to find path
  return false;
}

// 扩展节点
void AstarSearch::expandNodes(AstarNode & current_node, const bool is_back)
{
  // 获取当前节点的位姿，并根据搜索方向确定前进或后退
  const auto current_pose = node2pose(current_node);
  // 确定移动方向: 
  // 1.0表示向前/向目标方向
  // -1.0表示向后/远离目标方向
  const double direction = (is_back == is_backward_search_) ? 1.0 : -1.0;
  
  // 计算自适应的扩展距离，并应用方向系数
  const double distance = getExpansionDistance(current_node) * direction;

  // 遍历所有可能的转向角度
  // 从最大左转到最大右转(-turning_steps到+turning_steps)
  int steering_index = -1 * planner_common_param_.turning_steps;
  for (; steering_index <= planner_common_param_.turning_steps; ++steering_index) {
    
    // 防止无效路径扩展：
    // 如果当前节点有父节点，且搜索方向发生了变化（前进→倒车 或 倒车→前进），
    // 并且转向角度与父节点相同，则跳过此扩展（防止循环或无效路径）
    if (current_node.parent != nullptr && is_back != current_node.is_back &&
      steering_index == current_node.steering_index) {
      continue;
    }

    // 将转向索引转换为实际的转向角度（弧度）
    const double steering = static_cast<double>(steering_index) * steering_resolution_;

    // 使用自行车运动学模型计算下一个节点的位姿：
    // 根据当前位姿、车辆轴距、转向角度和移动距离，计算可能的下一位置
    const auto next_pose = kinematic_bicycle_model::getPose(
      current_pose, collision_vehicle_shape_.base_length, steering, distance);
    
    // 将连续位姿坐标转换为离散网格索引
    const auto next_index = pose2index(costmap_, next_pose, planner_common_param_.theta_size);

    // 有效性检查：
    // 如果下一节点位于地图边界外或位于障碍物上，跳过此扩展
    if (isOutOfRange(next_index) || isObs(next_index)) continue;

    // 根据索引在预分配的图结构中获取对应的节点
    AstarNode * next_node = &graph_[getKey(next_index)];

    // 如果节点已经被探索过(关闭状态)或检测到碰撞，跳过此扩展
    if (next_node->status == NodeStatus::Closed || detectCollision(next_index)) continue;

    // 计算下一节点处的障碍物距离数据
    const auto obs_edt = getObstacleEDT(next_index);
    
    // 判断是否发生了行驶方向的切换（前进→倒车 或 倒车→前进）
    const bool is_direction_switch =
      (current_node.parent != nullptr) && (is_back != current_node.is_back);

    // ===== 代价计算部分 =====
    // 初始化基础移动权重
    double total_weight = 1.0;
    // 添加转向惩罚（大角度转向权重更高）
    total_weight += getSteeringCost(steering_index);
    

    // 实际移动代价计算:
    // 当前节点的实际代价(g_cost) + 加权后的移动距离
    double move_cost = current_node.gc + (total_weight * std::abs(distance));
    
    // 添加转向变化惩罚（转向角变化越大，代价越高，鼓励平滑转向）
    move_cost += getSteeringChangeCost(steering_index, current_node.steering_index);
    
    // 添加障碍物距离惩罚（离障碍物越近，代价越高，鼓励远离障碍物）
    move_cost += getObsDistanceCost(next_index, obs_edt);
    
    // 添加横向距离惩罚（惩罚偏离参考路径的节点，鼓励保持在理想路径附近）
    move_cost += getLatDistanceCost(next_pose);
    
    // 如果发生了方向切换，添加方向切换惩罚（避免频繁改变行驶方向）
    if (is_direction_switch) move_cost += getDirectionChangeCost(current_node.dir_distance);

    // 计算总代价：实际移动代价(g) + 启发式估计代价(h)
    double total_cost = move_cost + estimateCost(next_pose, next_index);
    
    // 节点更新条件：
    // 1.节点是首次访问(状态为None) 
    // 2.找到了更优的路径(总代价更低)
    if (next_node->status == NodeStatus::None || next_node->fc > total_cost) {
      
      // 更新节点状态和属性
      next_node->status = NodeStatus::Open;  // 将节点标记为开放状态
      next_node->set(next_pose, move_cost, total_cost, steering_index, is_back);  // 设置位姿和代价
      // 计算连续同方向行驶的距离(用于计算方向切换代价)
      next_node->dir_distance =
        std::abs(distance) + (is_direction_switch ? 0.0 : current_node.dir_distance);
      next_node->dist_to_goal = calcDistance2d(next_pose, goal_pose_);  // 更新到目标的距离
      next_node->dist_to_obs = obs_edt.distance;  // 更新到障碍物的距离
      next_node->parent = &current_node;  // 设置父节点，用于回溯路径
      openlist_.push(next_node);  // 将节点加入优先队列(开放列表)
      continue;
    }
  }
}

// 自适应扩展距离计算
// 它根据当前节点的状态动态调整搜索步长，从而在搜索效率和路径质量之间取得平衡
double AstarSearch::getExpansionDistance(const AstarNode & current_node) const
{
  // 如果未启用自适应扩展
  // 或最大扩展距离不大于最小扩展距离，则直接返回最小扩展距离
  if (!astar_param_.adapt_expansion_distance || max_expansion_dist_ <= min_expansion_dist_) {
    return min_expansion_dist_;
  }

  // 计算基础扩展距离 为 当前节点到目标点和障碍物的距离*权重  最小值
  double exp_dist = std::min(
    current_node.dist_to_goal * dist_to_goal_expansion_factor_,
    current_node.dist_to_obs * dist_to_obs_expansion_factor_);

  // 保扩展距离在预设的最小值和最大值之间
  if(exp_dist < min_expansion_dist_) {
    exp_dist = min_expansion_dist_;
  } else if (exp_dist > max_expansion_dist_) {
    exp_dist = max_expansion_dist_;
  }

  // 返回经过限制的扩展距离
  return exp_dist;
}


// 计算转向代价
double AstarSearch::getSteeringCost(const int steering_index) const
{
  return planner_common_param_.curve_weight *
         (static_cast<double>(abs(steering_index)) / planner_common_param_.turning_steps);
}

double AstarSearch::getSteeringChangeCost(
  const int steering_index, const int prev_steering_index) const
{
  double steering_index_diff = abs(steering_index - prev_steering_index);
  return astar_param_.smoothness_weight * steering_index_diff /
         (2.0 * planner_common_param_.turning_steps);
}

double AstarSearch::getDirectionChangeCost(const double dir_distance) const
{
  return planner_common_param_.direction_change_weight * (1.0 + (1.0 / (1.0 + dir_distance)));
}

double AstarSearch::getObsDistanceCost(const IndexXYT & index, const EDTData & obs_edt) const
{
  if (obs_edt.distance > collision_vehicle_shape_.max_dimension + cost_free_obs_dist) {
    return 0.0;
  }
  const double yaw = index.theta * (2.0 * M_PI / planner_common_param_.theta_size);
  const double base_to_frame_dist = getVehicleBaseToFrameDistance(yaw - obs_edt.angle);
  const double vehicle_to_obs_dist = std::max(obs_edt.distance - base_to_frame_dist, 0.0);
  return astar_param_.obstacle_distance_weight *
         std::max(1.0 - (vehicle_to_obs_dist / cost_free_obs_dist), 0.0);
}

// 计算横向距离代价
double AstarSearch::getLatDistanceCost(const Pose & pose) const
{
  // 如果是多目标搜索，返回0.0
  if (is_multiple_goals_) return 0.0;
  // 设置参考点（起点或终点）
  const auto ref_pose = is_backward_search_ ? start_pose_ : goal_pose_;
  // 计算当前节点到参考点的距离
  const double distance_to_goal = calcDistance2d(pose, ref_pose);
  // 如果距离大于接近目标的距离，则返回0.0
  if (distance_to_goal > near_goal_dist_) return 0.0;
  // 横向距离
  const double lat_distance = std::abs(calcRelativePose(ref_pose, pose).position.y);
  return astar_param_.goal_lat_distance_weight * lat_distance;
}

// 设置路径
void AstarSearch::setPath(const AstarNode & goal_node){
  // 消息头
  std_msgs::msg::Header header;
  header.stamp = clock_->now();
  header.frame_id = costmap_.header.frame_id;

  // 目标节点
  const AstarNode * node = &goal_node;

  std::vector<PlannerWaypoint> waypoints;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;

  // 处理当前节点 与 其父节点 之间是否需要插值
  const auto interpolate = [this, &waypoints, &pose](const AstarNode & node) {

    // 父节点空 或 不需要插值时，直接返回
    if (node.parent == nullptr || !astar_param_.adapt_expansion_distance) return;

    // 计算父节点到当前节点的距离
    const auto parent_pose = node2pose(*node.parent);
    const double distance_2d = calcDistance2d(node2pose(node), parent_pose);
    
    // 当两点间距离超过阈值（min_expansion_dist_）时，在它们之间插入额外的路径点
    // n为需要插值数量
    const int n = static_cast<int>(distance_2d / min_expansion_dist_);
    
    // 插值
    for (int i = 1; i < n; ++i) {
      const double dist =((distance_2d * i) / n) * (node.is_back == is_backward_search_ ? 1.0 : -1.0);
      const double steering = node.steering_index * steering_resolution_;
      const auto local_pose = kinematic_bicycle_model::getPose(
        parent_pose, collision_vehicle_shape_.base_length, steering, dist);
      pose.pose = local2global(costmap_, local_pose);
      waypoints.push_back({pose, node.is_back});
    }
  };

  // 从目标节点开始，沿着父节点的路径回溯到起始节点
  while (node != nullptr) {
    // 局部位置转换为全局位置
    pose.pose = local2global(costmap_, node2pose(*node));
    // 插入当前节点到路径点列表中
    waypoints.push_back({pose, node->is_back});
    // 插值当前点与父节点之间的路径点
    interpolate(*node);
    node = node->parent;
  }

  if (waypoints.empty()) return;

  // 对路径点列表 waypoints 中的最后一个路径点的行驶方向（is_back）进行调整，使其与倒数第二个路径点的行驶方向保持一致
  if (waypoints.size() > 1) waypoints.back().is_back = waypoints.rbegin()[1].is_back;

  // 如果是正向搜索，则将路径点列表反转，使其从起点到终点的顺序
  if (!is_backward_search_) {
    std::reverse(waypoints.begin(), waypoints.end());
  }

  // 赋值
  waypoints_.header = header;
  waypoints_.waypoints = waypoints;

  // 如果是正向搜索，返回
  if (!is_backward_search_) return;

  // 针对反向搜索
  // 合并短距离内的连续倒车 / 前进切换，减少不必要的方向变化
  for (size_t i = 0; i < waypoints_.waypoints.size() - 1; ++i) {
    const auto & current = waypoints_.waypoints[i];
    auto & next = waypoints_.waypoints[i + 1];

    if (current.is_back != next.is_back) {
      next.is_back = current.is_back;
      ++i;  // 跳过下一个节点
    }
  }
}

// 到达目标点检查
bool AstarSearch::isGoal(const AstarNode & node) const
{
  // 定义目标判定的容差范围
  const double lateral_goal_range = planner_common_param_.lateral_goal_range / 2.0;      // 横向容差范围
  const double longitudinal_goal_range = planner_common_param_.longitudinal_goal_range / 2.0;  // 纵向容差范围

  // 将节点转换为姿态
  const auto node_pose = node2pose(node);

  // 定义一个lambda函数，用于检查节点是否满足目标条件
  auto checkGoal = [this, &node_pose, &lateral_goal_range, &longitudinal_goal_range,
                    &is_back = node.is_back](const Pose & pose) {
    // 获取节点和目标的网格索引
    const auto node_index = pose2index(costmap_, node_pose, planner_common_param_.theta_size);
    const auto goal_index = pose2index(costmap_, pose, planner_common_param_.theta_size);

    // 如果节点和目标在同一网格位置，则判定为达到目标
    if (node_index == goal_index) return true;// 同一位置

    // 计算目标姿态相对于当前节点的相对位置
    const auto relative_pose = calcRelativePose(pose, node_pose);

    
    // 检查横向和纵向距离是否在容差范围内
    if (
      std::fabs(relative_pose.position.x) > longitudinal_goal_range ||
      std::fabs(relative_pose.position.y) > lateral_goal_range) {
      return false;
    }

    // 所有条件满足，返回true
    return true;
  };

  // 首先检查是否满足主目标条件
  if (checkGoal(goal_pose_)) return true;

  // 如果不满足主目标，检查是否满足任一备选目标
  return std::any_of(
    alternate_goals_.begin(), alternate_goals_.end(),
    [&checkGoal](const Pose & pose) { return checkGoal(pose); });
}

void AstarSearch::setShiftedGoalPose(const Pose & goal_pose, const double lat_offset) const
{
  tf2::Transform tf;
  tf2::convert(goal_pose, tf);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf);

  Pose lat_pose;
  lat_pose.position = geometry_msgs::build<geometry_msgs::msg::Point>().x(0.0).y(lat_offset).z(0.0);
  lat_pose.orientation =
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0);

  shifted_goal_pose_ = transformPose(lat_pose, transform);
}

// 生成nav_msgs::msg::Path格式的路径消息
nav_msgs::msg::Path AstarSearch::getPath() const
{
  // 创建Path消息
  nav_msgs::msg::Path path;
  path.header = waypoints_.header;
  
  // 如果路径点为空，直接返回空Path
  if (waypoints_.waypoints.empty()) {
    return path;
  }
  
  // 将PlannerWaypoint格式转换为Path格式
  // Path只包含位姿信息，不包含方向信息(is_back)
  for (const auto & waypoint : waypoints_.waypoints) {
    path.poses.push_back(waypoint.pose);
  }
  
  return path;
}



Pose AstarSearch::node2pose(const AstarNode & node) const
{
  Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = goal_pose_.position.z;
  tf2::Quaternion q;
  q.setRPY(0, 0, node.theta);
  pose_local.orientation =  tf2::toMsg(q);

  return pose_local;
}