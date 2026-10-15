#include "costmap_generator/utils/points_to_costmap.hpp"

#include <string>
#include <vector>


// 获取map的参数
void PointsToCostmap::initGridmapParam(const grid_map::GridMap & gridmap)
{
  grid_length_x_ = gridmap.getLength().x();
  grid_length_y_ = gridmap.getLength().y();
  grid_resolution_ = gridmap.getResolution();
  grid_position_x_ = gridmap.getPosition().x();
  grid_position_y_ = gridmap.getPosition().y();
}

// 索引 有效性检查
bool PointsToCostmap::isValidInd(const grid_map::Index & grid_ind)
{
  bool is_valid = false;
  int x_grid_ind = grid_ind.x();
  int y_grid_ind = grid_ind.y();
  if (x_grid_ind >= 0 && x_grid_ind < std::ceil(grid_length_x_ * (1 / grid_resolution_)) &&
    y_grid_ind >= 0 && y_grid_ind < std::ceil(grid_length_y_ * (1 / grid_resolution_))) {
    is_valid = true;
  }
  return is_valid;
}

// 点云点 到 栅格map 索引
grid_map::Index PointsToCostmap::fetchGridIndexFromPoint(const pcl::PointXYZ & point)
{
  // 地图中心点 到 原点的偏移量
  const double origin_x_offset = grid_length_x_ / 2.0 - grid_position_x_;
  const double origin_y_offset = grid_length_y_ / 2.0 - grid_position_y_;
  

  double mapped_x = (grid_length_x_ - origin_x_offset - point.x) / grid_resolution_;
  double mapped_y = (grid_length_y_ - origin_y_offset - point.y) / grid_resolution_;

  int mapped_x_ind = std::ceil(mapped_x);
  int mapped_y_ind = std::ceil(mapped_y);
  grid_map::Index index(mapped_x_ind, mapped_y_ind);
  return index;
}

// 创建三维栅格,vec_x_y_z[x][y] 表示栅格（x，y）中 所有点的高度z（向量）
std::vector<std::vector<std::vector<double>>> PointsToCostmap::assignPoints2GridCell(
  const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points)
{
  // 地图在 x 和 y 方向大小，grid_resolution_ 表示栅格分辨率（米 / 格）
  double y_cell_size = std::ceil(grid_length_y_ * (1 / grid_resolution_));
  double x_cell_size = std::ceil(grid_length_x_ * (1 / grid_resolution_));

  // 创建三维数组结构vec_x_y_z[x_cell_size][y_cell_size]
  // vec_x_y_z[x][y] 表示栅格（x，y）中 所有点的高度z（向量）
  std::vector<double> z_vec;
  std::vector<std::vector<double>> vec_y_z(y_cell_size, z_vec);
  std::vector<std::vector<std::vector<double>>> vec_x_y_z(x_cell_size, vec_y_z);

  // 遍历点云并分配到栅格
  for (const auto & point : in_sensor_points) {
    // 点云点 对应 索引
    grid_map::Index grid_ind = fetchGridIndexFromPoint(point);
    if (isValidInd(grid_ind)) {// 有效性检查
      // 将该点的z 放入到对应的栅格中
      vec_x_y_z[grid_ind.x()][grid_ind.y()].push_back(point.z);
    }
  }
  return vec_x_y_z;
}

// 创建栅格图grid_map::Matrix
grid_map::Matrix PointsToCostmap::calculateCostmap(
  const double maximum_height_thres, const double minimum_lidar_height_thres,
  const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name,
  const std::vector<std::vector<std::vector<double>>> grid_vec)
{
  // 创建地图
  grid_map::Matrix gridmap_data = gridmap[gridmap_layer_name];
  // 遍历三维结构数组
  for (size_t x_ind = 0; x_ind < grid_vec.size(); x_ind++) {// x
    for (size_t y_ind = 0; y_ind < grid_vec[0].size(); y_ind++) {//y
      
      // 点云中的点，并没有落在该栅格中，则设置为最小值
      if (grid_vec[x_ind][y_ind].size() == 0) {
        gridmap_data(x_ind, y_ind) = grid_min_value;
        continue;
      }
      // 遍历该栅格中的所有z，
      for (const auto & z : grid_vec[x_ind][y_ind]) {
        // z超出考虑范围
        if (z > maximum_height_thres || z < minimum_lidar_height_thres) {
          continue;
        }
        // 如果有z值，则设置为最大值，退出，遍历下一个栅格
        gridmap_data(x_ind, y_ind) = grid_max_value;
        break;
      }
    }
  }
  return gridmap_data;
}

// pointTOcostmap
grid_map::Matrix PointsToCostmap::makeCostmapFromPoints(
  const double maximum_height_thres, const double minimum_lidar_height_thres,
  const double grid_min_value, const double grid_max_value, const grid_map::GridMap & gridmap,
  const std::string & gridmap_layer_name, const pcl::PointCloud<pcl::PointXYZ> & in_sensor_points)
{
  // 初始化参数
  initGridmapParam(gridmap);
  // 三维的栅格创建
  std::vector<std::vector<std::vector<double>>> grid_vec = assignPoints2GridCell(in_sensor_points);
  
  grid_map::Matrix costmap = calculateCostmap(
    maximum_height_thres, minimum_lidar_height_thres, grid_min_value, grid_max_value, gridmap,
    gridmap_layer_name, grid_vec);
  return costmap;
}
