#include "pcd2pgm/pcd2pgm.hpp"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <rclcpp/qos.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

namespace pcd2pgm
{

PCLFiltersNode::PCLFiltersNode(const rclcpp::NodeOptions & options)
: Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  // 发布 map 时使用 transient_local，保证后启动的 RViz / nav2 也能收到地图
  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    map_topic_name_, map_qos);

  // 读取 PCD 文件
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Initial point cloud size: %lu",
    pcd_cloud->points.size());

  /*
   * 重要修改：
   *
   * 原代码这里会强制执行：
   * 1. segmentGroundPlane()
   * 2. radiusOutlierFilter()
   *
   * 这样会导致很多有效墙体点被误删。
   *
   * 现在改为：
   * 直接使用原始 PCD 点云生成地图。
   *
   * flag_pass_through_ == false:
   *   不做 Z 轴过滤，直接 XY 投影。
   *
   * flag_pass_through_ == true:
   *   只保留 thre_z_min_ 到 thre_z_max_ 范围内的点。
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_map =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::size_t invalid_count = 0;
  std::size_t z_filtered_count = 0;

  for (const auto & point : pcd_cloud->points) {
    // 过滤 NaN / Inf 点
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z))
    {
      invalid_count++;
      continue;
    }

    // 只有 flag_pass_through_ 为 true 时，才启用 Z 轴高度过滤
    if (flag_pass_through_) {
      if (point.z < thre_z_min_ || point.z > thre_z_max_) {
        z_filtered_count++;
        continue;
      }
    }

    cloud_for_map->points.push_back(point);
  }

  cloud_for_map->width = static_cast<uint32_t>(cloud_for_map->points.size());
  cloud_for_map->height = 1;
  cloud_for_map->is_dense = false;

  RCLCPP_INFO(
    get_logger(),
    "Cloud for map: raw=%lu | valid=%lu | invalid=%lu | z_filtered=%lu | z_filter=%s",
    pcd_cloud->points.size(),
    cloud_for_map->points.size(),
    invalid_count,
    z_filtered_count,
    flag_pass_through_ ? "ON" : "OFF");

  if (cloud_for_map->empty()) {
    RCLCPP_WARN(get_logger(), "No valid points left for map generation!");
    return;
  }

  // 直接根据点云生成 OccupancyGrid
  setMapTopicMsg(cloud_for_map, map_topic_msg_);

  // 周期性发布地图
  time_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PCLFiltersNode::publishMap, this));

  RCLCPP_INFO(get_logger(), "Node initialized successfully.");
}

PCLFiltersNode::~PCLFiltersNode() {}

void PCLFiltersNode::publishMap()
{
  if (rclcpp::ok()) {
    map_publisher_->publish(map_topic_msg_);
  }
}

void PCLFiltersNode::declareParameters()
{
  declare_parameter("file_directory", "/home/nvidia/auto_driving/");
  declare_parameter("file_name", "map");

  /*
   * Z 轴过滤参数：
   * 只有 flag_pass_through = true 时才生效。
   * 如果 flag_pass_through = false，这两个参数不会参与过滤。
   */
  declare_parameter("thre_z_min", -1.0);
  declare_parameter("thre_z_max", 5.0);
  declare_parameter("flag_pass_through", false);

  /*
   * 重要：
   * 现在 thre_radius 不再用来做离群点滤波，
   * 而是用作 PGM 里障碍物膨胀半径。
   *
   * 例如：
   * map_resolution = 0.05
   * thre_radius = 0.15
   * 代表障碍物会膨胀大约 3 个栅格。
   */
  declare_parameter("thre_radius", 0.15);

  // 地图分辨率，单位：m/pixel
  declare_parameter("map_resolution", 0.05);

  /*
   * 保留参数，但默认主流程不使用它。
   * 原来用于半径滤波，现在默认不启用滤波。
   */
  declare_parameter("thres_point_count", 1);

  declare_parameter("map_topic_name", "map");

  /*
   * crop_range:
   * 0.0 表示不裁剪。
   *
   * 原代码默认 ±50m 裁剪，容易把地图裁掉。
   * 所以这里默认关闭裁剪。
   *
   * 如果确实需要裁剪，可以在 yaml 中设置：
   * crop_range: 100.0
   */
  declare_parameter("crop_range", 0.0);
}

void PCLFiltersNode::getParameters()
{
  get_parameter("file_directory", file_directory_);
  get_parameter("file_name", file_name_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("crop_range", crop_range_);

  pcd_file_ = file_directory_ + file_name_ + ".pcd";

  RCLCPP_INFO(get_logger(), "PCD file: %s", pcd_file_.c_str());
  RCLCPP_INFO(get_logger(), "map_resolution: %.3f", map_resolution_);
  RCLCPP_INFO(get_logger(), "inflate radius thre_radius: %.3f", thre_radius_);
  RCLCPP_INFO(get_logger(), "flag_pass_through: %s", flag_pass_through_ ? "true" : "false");
  RCLCPP_INFO(get_logger(), "z range: [%.3f, %.3f]", thre_z_min_, thre_z_max_);
  RCLCPP_INFO(get_logger(), "crop_range: %.3f", crop_range_);
}

/*
 * 这个函数保留备用。
 * 当前主流程不再调用它。
 *
 * 原因：
 * 你的 PCD 本身已经是地图点云，强行做 RANSAC 地面分割，
 * 很容易把有效墙体点、边缘点、地面投影点误删。
 */
void PCLFiltersNode::segmentGroundPlane(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr & output_cloud)
{
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr ground_inliers = std::make_shared<pcl::PointIndices>();
  pcl::ModelCoefficients::Ptr plane_coeffs = std::make_shared<pcl::ModelCoefficients>();

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0));
  seg.setEpsAngle(0.15);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.15);

  seg.setInputCloud(input_cloud);
  seg.segment(*ground_inliers, *plane_coeffs);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(input_cloud);
  extract.setIndices(ground_inliers);
  extract.setNegative(true);
  extract.filter(*output_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_height_filtered =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  for (const auto & point : output_cloud->points) {
    if (std::isfinite(point.x) &&
        std::isfinite(point.y) &&
        std::isfinite(point.z) &&
        point.z >= thre_z_min_ &&
        point.z <= thre_z_max_)
    {
      cloud_height_filtered->points.push_back(point);
    }
  }

  cloud_height_filtered->width =
    static_cast<uint32_t>(cloud_height_filtered->points.size());
  cloud_height_filtered->height = 1;
  cloud_height_filtered->is_dense = false;

  output_cloud = cloud_height_filtered;

  RCLCPP_INFO(
    get_logger(),
    "Ground segmentation backup function: input=%lu | removed_ground=%lu | remaining=%lu",
    input_cloud->points.size(),
    ground_inliers->indices.size(),
    output_cloud->points.size());
}

/*
 * 这个函数保留备用。
 * 当前主流程没有调用。
 */
void PCLFiltersNode::passThroughFilter(
  const double & thre_low,
  const double & thre_high,
  const bool & flag_in)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr capt =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  cloud_after_PassThrough_ = capt;

  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*cloud_after_PassThrough_);

  RCLCPP_INFO(
    get_logger(),
    "After PassThrough filtering, point cloud size: %lu",
    cloud_after_PassThrough_->points.size());
}

/*
 * 这个函数保留备用。
 * 当前主流程没有调用。
 *
 * 注意：
 * 统计滤波可能会把稀疏墙体、细线、边缘点删掉。
 * 所以在 PCD 转 PGM 时，不建议默认启用。
 */
void PCLFiltersNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & pcd_cloud0,
  const double & radius,
  const int & thre_count)
{
  cloud_after_Radius_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(pcd_cloud0);
  sor.setMeanK(30);
  sor.setStddevMulThresh(1.5);
  sor.filter(*cloud_after_Radius_);

  if (cloud_after_Radius_->size() < pcd_cloud0->size() * 0.3) {
    RCLCPP_WARN(
      get_logger(),
      "Statistical filter too strict, fallback to radius filter");

    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(pcd_cloud0);
    radius_filter.setRadiusSearch(radius);
    radius_filter.setMinNeighborsInRadius(std::max(1, thre_count - 1));
    radius_filter.filter(*cloud_after_Radius_);
  }

  RCLCPP_INFO(
    get_logger(),
    "After outlier filtering: input=%lu | output=%lu",
    pcd_cloud0->points.size(),
    cloud_after_Radius_->points.size());
}

/*
 * 核心函数：
 * 点云 XY 投影生成 OccupancyGrid。
 *
 * 修改重点：
 * 1. 默认不裁剪点云。
 * 2. 正确计算 x_min / y_min / x_max / y_max。
 * 3. 地图尺寸 width / height 加 +1，避免边界点丢失。
 * 4. 加 padding，避免膨胀后边缘被切掉。
 * 5. 使用 thre_radius_ 对障碍物进行膨胀，让墙体更连续。
 */
void PCLFiltersNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
  nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.info.map_load_time = this->now();

  msg.info.resolution = map_resolution_;

  if (msg.info.resolution <= 0.0) {
    RCLCPP_ERROR(get_logger(), "Invalid map_resolution: %.3f", msg.info.resolution);
    return;
  }

  /*
   * 第一步：可选裁剪点云
   *
   * crop_range_ <= 0.0:
   *   不裁剪，保留全部点。
   *
   * crop_range_ > 0.0:
   *   只保留 x 和 y 在 ±crop_range_ 范围内的点。
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped =
    std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::size_t crop_removed_count = 0;

  for (const auto & point : cloud->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
      continue;
    }

    if (crop_range_ > 0.0) {
      if (std::fabs(point.x) > crop_range_ ||
          std::fabs(point.y) > crop_range_)
      {
        crop_removed_count++;
        continue;
      }
    }

    cloud_cropped->points.push_back(point);
  }

  if (cloud_cropped->empty()) {
    RCLCPP_WARN(get_logger(), "No valid points left after cropping!");
    return;
  }

  /*
   * 第二步：计算点云 XY 边界
   */
  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : cloud_cropped->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  /*
   * 第三步：给地图边界增加 padding
   *
   * 如果没有 padding，地图最边缘的墙体膨胀后会被裁掉。
   */
  const double padding = std::max(
  static_cast<double>(thre_radius_),
  static_cast<double>(msg.info.resolution) * 2.0);


  const double origin_x = x_min - padding;
  const double origin_y = y_min - padding;

  const double x_range = (x_max - x_min) + 2.0 * padding;
  const double y_range = (y_max - y_min) + 2.0 * padding;

  msg.info.width =
    static_cast<unsigned int>(std::ceil(x_range / msg.info.resolution)) + 1;
  msg.info.height =
    static_cast<unsigned int>(std::ceil(y_range / msg.info.resolution)) + 1;

  msg.info.origin.position.x = origin_x;
  msg.info.origin.position.y = origin_y;
  msg.info.origin.position.z = 0.0;

  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  /*
   * 第四步：初始化地图
   *
   * 这里使用 0 表示空闲区域，100 表示障碍物。
   * 这样保存成 PGM 时，通常白色为空闲，黑色为障碍。
   *
   * 如果你希望未知区域显示为灰色，可以改成：
   * msg.data.assign(msg.info.width * msg.info.height, -1);
   */
  msg.data.assign(msg.info.width * msg.info.height, 0);

  /*
   * 第五步：计算膨胀栅格数量
   *
   * thre_radius_ = 0.15
   * resolution = 0.05
   * inflate_cells = 3
   */
  int inflate_cells =
    static_cast<int>(std::ceil(thre_radius_ / msg.info.resolution));

  if (inflate_cells < 0) {
    inflate_cells = 0;
  }

  std::size_t occupied_raw_count = 0;
  std::size_t occupied_inflated_count = 0;

  /*
   * 第六步：把点云投影到 2D 栅格地图
   *
   * 原始点坐标：
   *   point.x, point.y
   *
   * 地图原点：
   *   origin_x, origin_y
   *
   * 栅格坐标：
   *   i = (point.x - origin_x) / resolution
   *   j = (point.y - origin_y) / resolution
   */
  for (const auto & point : cloud_cropped->points) {
    int i = static_cast<int>(
      std::floor((static_cast<double>(point.x) - origin_x) / msg.info.resolution));

    int j = static_cast<int>(
      std::floor((static_cast<double>(point.y) - origin_y) / msg.info.resolution));

    if (i < 0 || i >= static_cast<int>(msg.info.width) ||
        j < 0 || j >= static_cast<int>(msg.info.height))
    {
      continue;
    }

    occupied_raw_count++;

    /*
     * 障碍物膨胀：
     * 以当前点所在栅格为圆心，在 inflate_cells 半径内都标记为障碍物。
     *
     * 这样可以解决：
     * 1. 点云稀疏导致墙体断裂
     * 2. PGM 墙线太细
     * 3. 导航地图墙体不连续
     */
    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx) {
      for (int dy = -inflate_cells; dy <= inflate_cells; ++dy) {
        if (dx * dx + dy * dy > inflate_cells * inflate_cells) {
          continue;
        }

        int ni = i + dx;
        int nj = j + dy;

        if (ni >= 0 && ni < static_cast<int>(msg.info.width) &&
            nj >= 0 && nj < static_cast<int>(msg.info.height))
        {
          const std::size_t index =
            static_cast<std::size_t>(ni) +
            static_cast<std::size_t>(nj) * static_cast<std::size_t>(msg.info.width);

          if (msg.data[index] != 100) {
            occupied_inflated_count++;
          }

          msg.data[index] = 100;
        }
      }
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Map generated: width=%u height=%u res=%.3f origin=(%.3f, %.3f)",
    msg.info.width,
    msg.info.height,
    msg.info.resolution,
    msg.info.origin.position.x,
    msg.info.origin.position.y);

  RCLCPP_INFO(
    get_logger(),
    "Point range: x=[%.3f, %.3f] y=[%.3f, %.3f] range=(%.3f x %.3f)",
    x_min,
    x_max,
    y_min,
    y_max,
    x_max - x_min,
    y_max - y_min);

  RCLCPP_INFO(
    get_logger(),
    "Map points: input=%lu cropped=%lu crop_removed=%lu raw_occupied=%lu inflated_cells=%lu inflate_cells=%d",
    cloud->points.size(),
    cloud_cropped->points.size(),
    crop_removed_count,
    occupied_raw_count,
    occupied_inflated_count,
    inflate_cells);
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::PCLFiltersNode)

