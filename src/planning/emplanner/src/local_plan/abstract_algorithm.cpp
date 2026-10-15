#include "emplanner/local_plan/abstract_algorithm.hpp"
#include "emplanner/local_plan/kinematic_bicycle_model.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

/**
 * 从偏航角创建四元数 - 将偏航角转换为四元数表示
 * @param yaw 偏航角(弧度)
 * @return 对应的四元数
 */
geometry_msgs::msg::Quaternion createQuaternionFromYaw(const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);  // 设置Roll=0, Pitch=0, Yaw=指定值
  
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = q.x();
  quaternion.y = q.y();
  quaternion.z = q.z();
  quaternion.w = q.w();
  
  return quaternion;
}

/**
 * 计算两点间的2D距离
 * @param pose1 第一个位姿
 * @param pose2 第二个位姿
 * @return 两点间的欧氏距离
 */
double calcDistance2d(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2)
{
  const double dx = pose2.pose.position.x - pose1.pose.position.x;
  const double dy = pose2.pose.position.y - pose1.pose.position.y;
  return std::hypot(dx, dy);
}

/**
 * 位姿坐标变换实现 - 将给定位姿通过变换矩阵进行转换
 */
geometry_msgs::msg::Pose transformPose(
  const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform)
{
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

/**
 * 角度离散化实现 - 将连续角度值离散化为指定大小的索引
 */
int discretizeAngle(const double theta, const int theta_size)
{
  const double angle_resolution = 2.0 * M_PI / theta_size;  // 角度分辨率
  // TODO
  return static_cast<int>(std::round(normalizeRadian(theta, 0.0) / angle_resolution)) % theta_size;
}

/**
 * 位姿转索引实现 - 将位姿转换为栅格地图上的三维索引
 */
IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size)
{
  const int index_x = std::round(pose_local.position.x / costmap.info.resolution);
  const int index_y = std::round(pose_local.position.y / costmap.info.resolution);
  const int index_theta = discretizeAngle(tf2::getYaw(pose_local.orientation), theta_size);
  return {index_x, index_y, index_theta};
}

/**
 * 索引转位姿实现 - 将三维索引转换为位姿
 */
geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const int theta_size)
{
  geometry_msgs::msg::Pose pose_local;

  pose_local.position.x = index.x * costmap.info.resolution;
  pose_local.position.y = index.y * costmap.info.resolution;

  const double angle_resolution = 2.0 * M_PI / theta_size;
  const double yaw = index.theta * angle_resolution;
  // TODO
  pose_local.orientation = createQuaternionFromYaw(yaw);

  return pose_local;
}

/**
 * 全局坐标系转局部坐标系实现 - 将全局位姿转换为代价地图局部坐标系下的位姿
 */
geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global)
{
  // 获取代价地图原点变换
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  // 创建逆变换
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  // 应用变换
  return transformPose(pose_global, transform);
}

/**
 * 局部坐标系转全局坐标系实现 - 将局部位姿转换为全局坐标系下的位姿
 */
geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local)
{
  // 获取代价地图原点变换
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  // 创建变换
  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin);

  // 应用变换
  return transformPose(pose_local, transform);
}

/**
 * 计算路径总长度实现
 * @return 路径长度 [m]
 */
double PlannerWaypoints::compute_length() const
{
  if (waypoints.empty()) {
    throw std::runtime_error("cannot compute cost because waypoint has size 0");
  }
  
  double total_cost = 0.0;
  // 累加相邻路径点之间的距离
  for (size_t i = 0; i < waypoints.size() - 1; ++i) {
    const auto pose_a = waypoints.at(i);
    const auto pose_b = waypoints.at(i + 1);
    // TODO
    total_cost += calcDistance2d(pose_a.pose, pose_b.pose);
  }
  return total_cost;
}

/**
 * 设置规划使用的代价地图实现
 * 初始化障碍物表格、欧氏距离变换地图和碰撞检测索引表
 */
void AbstractPlanningAlgorithm::setMap(const nav_msgs::msg::OccupancyGrid & costmap)
{
  costmap_ = costmap;
  const uint32_t nb_of_cells = costmap_.data.size();  // 栅格总数量
  
  // 初始化障碍物表格
  std::vector<bool> is_obstacle_table;
  is_obstacle_table.resize(nb_of_cells);
  
  // 遍历代价地图每个栅格，判断是否为障碍物
  // 若代价值小于0（未知区域）或者大于等于obstacle_threshold，则认为是障碍物
  for (uint32_t i = 0; i < nb_of_cells; ++i) {
    const int cost = costmap_.data[i];
    if (cost < 0 || planner_common_param_.obstacle_threshold <= cost) {
      is_obstacle_table[i] = true;
    }
  }
  is_obstacle_table_ = is_obstacle_table;

  // 计算欧氏距离变换地图(EDT)
  computeEDTMap();

  // 如果碰撞表尚未初始化，则为每个角度计算碰撞检测索引
  if (is_collision_table_initialized == false) {
    for (int i = 0; i < planner_common_param_.theta_size; i++) {
      std::vector<IndexXY> indexes_2d, vertex_indexes_2d;
      computeCollisionIndexes(i, indexes_2d, vertex_indexes_2d);
      coll_indexes_table_.push_back(indexes_2d);
      vertex_indexes_table_.push_back(vertex_indexes_2d);
    }
    is_collision_table_initialized = true;
  }

  // 计算安全边距单元格数量
  // 首先计算车辆从基点到前端的距离 base2front
  // 然后根据车辆的宽度和 base2front 计算车辆的最大斜边长度
  // 最后将这个长度除以地图分辨率并向上取整，得到需要考虑的边缘单元格数量
  const double base2front = collision_vehicle_shape_.length - collision_vehicle_shape_.base2back;
  nb_of_margin_cells_ = std::ceil(std::hypot(0.5 * collision_vehicle_shape_.width, base2front) / costmap_.info.resolution);
}

/**
 * 获取指定位姿到最近障碍物的距离实现
 * @return 到最近障碍物的距离 [m]
 */
double AbstractPlanningAlgorithm::getDistanceToObstacle(const geometry_msgs::msg::Pose & pose) const
{
  // 将全局位姿转换为局部位姿
  const auto local_pose = global2local(costmap_, pose);
  // 将局部位姿转换为索引
  const auto index = pose2index(costmap_, local_pose, planner_common_param_.theta_size);
  
  // 检查索引是否有效
  if (indexToId(index) >= static_cast<int>(edt_map_.size())) {
    return std::numeric_limits<double>::max();
  }
  
  // 返回该索引处的障碍物距离
  return getObstacleEDT(index).distance;
}

/**
 * 计算欧氏距离变换地图(EDT)实现
 * 为每个栅格计算到最近障碍物的距离和方向
 */
void AbstractPlanningAlgorithm::computeEDTMap()
{
  edt_map_.clear();
  
  // 获取地图参数
  const int height = costmap_.info.height;
  const int width = costmap_.info.width;
  const double resolution_m = costmap_.info.resolution;
  
  // 创建临时EDT地图，存储每个点到最近障碍物的距离和相对位置
  std::vector<std::pair<double, geometry_msgs::msg::Point>> edt_map;
  edt_map.reserve(costmap_.data.size());

  // 临时存储，大小为宽度（用于行扫描）
  std::vector<std::pair<double, geometry_msgs::msg::Point>> temporary_storage(width);

  // 第一阶段：行扫描 - 计算每个点到水平方向最近障碍物的距离
  for (int i = 0; i < height; ++i) {
    double distance = resolution_m;  // 初始化距离为一个栅格的分辨率
    bool found_obstacle = false;
    
    // 从左到右扫描
    for (int j = 0; j < width; ++j) {
      // 检查当前位置是否是障碍物
      if (isObs(IndexXY{j, i})) {
        // 如果是障碍物，则设置距离为0，相对位置x为0
        temporary_storage[j].first = 0.0;
        temporary_storage[j].second.x = 0.0;
        distance = resolution_m;  // 重置距离
        found_obstacle = true;
      } else if (found_obstacle) {
        // 如果不是障碍物，但前面有障碍物，则记录当前距离和相对位置（负X表示左侧）
        temporary_storage[j].first = distance;
        temporary_storage[j].second.x = -distance;  // 障碍物在左侧，所以是负值
        distance += resolution_m;  // 增加距离
      } else {
        // 如果不是障碍物，且前面没有障碍物，则设置为无穷大
        temporary_storage[j].first = std::numeric_limits<double>::infinity();
      }
    }

    // 从右到左扫描
    distance = resolution_m;  // 重置距离
    found_obstacle = false;
    for (int j = width - 1; j >= 0; --j) {
      // 检查当前位置是否是障碍物
      if (isObs(IndexXY{j, i})) {
        // 如果是障碍物，重置距离
        distance = resolution_m;
        found_obstacle = true;
      } else if (found_obstacle && temporary_storage[j].first > distance) {
        // 如果不是障碍物，且前面有障碍物，并且右侧障碍物距离更近
        // 则更新距离和相对位置（正X表示右侧）
        temporary_storage[j].first = distance;
        temporary_storage[j].second.x = distance;
        distance += resolution_m;
      }
    }

    // 将当前行的结果添加到EDT地图中
    edt_map.insert(edt_map.end(), temporary_storage.begin(), temporary_storage.end());
  }

  // 第二阶段：列扫描 - 结合水平和垂直方向的距离
  // 清空临时存储，重新设置大小为高度
  temporary_storage.clear();
  temporary_storage.resize(height);
  
  // 对每一列进行处理
  for (int j = 0; j < width; ++j) {
    // 对当前列的每个点进行处理
    for (int i = 0; i < height; ++i) {
      // 当前点索引
      int id = indexToId(IndexXY{j, i});
      
      // 初始化最小距离为水平方向的距离平方
      double min_value = edt_map[id].first * edt_map[id].first;
      // 初始化相对位置为水平方向的位置
      geometry_msgs::msg::Point rel_pos = edt_map[id].second;

      // 遍历当前列的所有点，寻找最近的障碍物
      for (int k = 0; k < height; ++k) {
        // 计算循环点的索引
        id = indexToId(IndexXY{j, k});
        // 当前点与循环点的垂直距离
        double dist = resolution_m * std::abs(static_cast<double>(i - k));
        
        // 计算总距离的平方 = 循环点水平距离平方 + 当前点与循环点的垂直距离平方
        double value = edt_map[id].first * edt_map[id].first + dist * dist;
        
        // 更新最近距离
        if (value < min_value) {
          min_value = value;
          // 保存水平方向的相对位置
          rel_pos.x = edt_map[id].second.x;
          // 保存垂直方向的相对位置（正负表示上下方向）
          rel_pos.y = (i > k) ? dist : -dist;
        }
      }
      
      // 将结果保存到临时存储中
      temporary_storage[i].first = sqrt(min_value);  // 距离
      temporary_storage[i].second = rel_pos;  // 相对位置
    }
    
    // 将临时存储的结果更新到EDT地图中
    for (int i = 0; i < height; ++i) {
      edt_map[indexToId(IndexXY{j, i})] = temporary_storage[i];
    }
  }
  
  // 第三阶段：转换为极坐标形式（距离和角度）
  for (const auto & edt : edt_map) {
    // 计算相对位置的角度
    const double angle = std::atan2(edt.second.y, edt.second.x);
    // 添加到最终的EDT地图中
    edt_map_.push_back({edt.first, angle});
  }
}

/**
 * 计算碰撞检测索引实现
 * 根据给定的角度索引计算车辆占用的栅格和顶点栅格
 */
void AbstractPlanningAlgorithm::computeCollisionIndexes(
  int theta_index, std::vector<IndexXY> & indexes_2d,
  std::vector<IndexXY> & vertex_indexes_2d) const
{
  // 设置基准索引
  IndexXYT base_index{0, 0, theta_index};
  const VehicleShape & vehicle_shape = collision_vehicle_shape_;

  /**
   * 车辆模型示意图
   * ------------
   * |          |
   * |          |
   * |          |
   * |          |
   * |   .      | 点表示车辆基准点(base)，车辆的前后左右距离都是相对于这个点
   * |          |
   * ------------
   */
  // 计算车辆边界相对于基准点的距离
  const double back = -1.0 * vehicle_shape.base2back;  // 后方距离为负值
  const double front = vehicle_shape.length - vehicle_shape.base2back;  // 前方距离
  const double right = -1.0 * vehicle_shape.width / 2.0;  // 右侧距离为负值
  const double left = vehicle_shape.width / 2.0;  // 左侧距离

  // 获取基准位姿和角度
  const auto base_pose = index2pose(costmap_, base_index, planner_common_param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // 定义添加索引的辅助函数
  const auto addIndex2d = [&](const double x, const double y, std::vector<IndexXY> & indexes_cache) {
    // 旋转坐标系下的偏移计算
    const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
    const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

    // 计算偏移后的位姿
    geometry_msgs::msg::Pose pose_local;
    pose_local.position.x = base_pose.position.x + offset_x;
    pose_local.position.y = base_pose.position.y + offset_y;

    // 将位姿转换为索引，并添加到索引缓存中
    const auto index = pose2index(costmap_, pose_local, planner_common_param_.theta_size);
    const auto index_2d = IndexXY{index.x, index.y};
    indexes_cache.push_back(index_2d);
  };

  // 生成车辆占用的所有栅格索引
  // 以一定的分辨率采样车辆几何区域
  for (double x = back; x <= front; x += costmap_.info.resolution / 2) {
    for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
      addIndex2d(x, y, indexes_2d);
    }
    addIndex2d(x, left, indexes_2d);  // 确保左侧边界被采样
  }
  // 确保前部边界被采样
  for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
    addIndex2d(front, y, indexes_2d);
  }
  // 确保前左角被采样
  addIndex2d(front, left, indexes_2d);

  // 定义索引比较函数和相等函数
  const auto compareIndex2d = [](const IndexXY & left, const IndexXY & right) {
    if (left.x != right.x) {
      return (left.x < right.x);
    } else {
      return (left.y < right.y);
    }
  };

  const auto equalIndex2d = [](const IndexXY & left, const IndexXY & right) {
    return ((left.x == right.x) && (left.y == right.y));
  };

  // 移除重复的索引
  std::sort(indexes_2d.begin(), indexes_2d.end(), compareIndex2d);
  indexes_2d.erase(
    std::unique(indexes_2d.begin(), indexes_2d.end(), equalIndex2d), indexes_2d.end());

  // 生成车辆顶点索引（四个角点）
  addIndex2d(front, left, vertex_indexes_2d);   // 左前角
  addIndex2d(front, right, vertex_indexes_2d);  // 右前角
  addIndex2d(back, right, vertex_indexes_2d);   // 右后角
  addIndex2d(back, left, vertex_indexes_2d);    // 左后角
}

/**
 * 检测是否超出地图边界实现
 * @return 是否超出边界
 */
bool AbstractPlanningAlgorithm::detectBoundaryExit(const IndexXYT & base_index) const
{
  // 如果基准索引在安全边界内，则不会超出边界
  if (isWithinMargin(base_index)) return false;

  // 获取当前角度对应的车辆顶点索引
  const auto & vertex_indexes_2d = vertex_indexes_table_[base_index.theta];
  
  // 检查每个顶点是否超出边界
  for (const auto & vertex_index_2d : vertex_indexes_2d) {
    // 将顶点索引平移到当前基准位置
    IndexXY vertex_index{vertex_index_2d.x, vertex_index_2d.y};
    vertex_index.x += base_index.x;
    vertex_index.y += base_index.y;
    
    // 检查是否超出地图范围
    if (isOutOfRange(vertex_index)) {
      return true;
    }
  }
  return false;
}

/**
 * 检测给定位姿是否发生碰撞实现
 * @return 是否碰撞
 */
bool AbstractPlanningAlgorithm::detectCollision(const geometry_msgs::msg::Pose & base_pose) const
{
  // 将位姿转换为索引
  const auto base_index = pose2index(costmap_, base_pose, planner_common_param_.theta_size);
  // 调用索引版本的碰撞检测函数
  return detectCollision(base_index);
}

/**
 * 检测给定索引位置是否发生碰撞实现
 * @return 是否碰撞
 */
bool AbstractPlanningAlgorithm::detectCollision(const IndexXYT & base_index) const
{
  // 检查碰撞表是否已初始化
  if (coll_indexes_table_.empty()) {
    std::cerr << "[abstract_algorithm] setMap has not yet been done." << std::endl;
    return false;
  }

  // 检查是否超出边界
  if (detectBoundaryExit(base_index)) return true;

  // 获取当前位置到最近障碍物的距离
  double obstacle_edt = getObstacleEDT(base_index).distance;

  // 快速碰撞判断：
  // 1. 如果距离大于车辆的最大尺寸（如对角线长度），则肯定不会碰撞
  if (obstacle_edt > collision_vehicle_shape_.max_dimension) return false;
  // 2. 如果距离小于车辆的最小尺寸（如宽度一半），则肯定会碰撞
  if (obstacle_edt < collision_vehicle_shape_.min_dimension) return true;

  // 详细碰撞检查 - 遍历车辆所有占用栅格
  const auto & coll_indexes_2d = coll_indexes_table_[base_index.theta];
  for (const auto & coll_index_2d : coll_indexes_2d) {
    // 将碰撞索引平移到当前基准位置
    IndexXY coll_index{coll_index_2d.x, coll_index_2d.y};
    coll_index.x += base_index.x;
    coll_index.y += base_index.y;

    // 检查该位置是否有障碍物
    if (isObs(coll_index)) {
      return true;
    }
  }

  // 没有碰撞
  return false;
}

/**
 * 检查轨迹上是否存在障碍物实现
 * @return 是否存在障碍物
 */
bool AbstractPlanningAlgorithm::hasObstacleOnTrajectory(
  const geometry_msgs::msg::PoseArray & trajectory) const
{
  // 遍历轨迹上的每个位姿
  for (const auto & pose : trajectory.poses) {
    // 将全局位姿转换为局部位姿
    const auto pose_local = global2local(costmap_, pose);
    // 检查该位姿是否发生碰撞
    if (detectCollision(pose_local)) {
      return true;
    }
  }

  return false;
}