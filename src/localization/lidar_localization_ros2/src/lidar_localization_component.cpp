#include <lidar_localization/lidar_localization_component.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <tf2/LinearMath/Transform.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");

  declare_parameter("cloud_topic", "/rslidar_points");
  declare_parameter("odom_topic", "/odom");
  declare_parameter("imu_topic", "/imu");
  declare_parameter("map_topic", "/map");
  declare_parameter("initialpose_topic", "/initialpose");

  declare_parameter("registration_method", "NDT_OMP");
  declare_parameter("score_threshold", 1.5);
  declare_parameter("init_score_threshold", 3.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.1);

  declare_parameter("scan_max_range", 60.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);

  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");

  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);

  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);

  declare_parameter("reject_translation_threshold", 0.5);
  declare_parameter("reject_yaw_threshold_deg", 5.0);
  declare_parameter("reject_z_threshold", 0.3);
  declare_parameter("reject_roll_pitch_threshold_deg", 10.0);

  declare_parameter("localization_success_frame_num", 3);
  declare_parameter("tracking_lost_frame_num", 5);
  declare_parameter("allow_tracking_lost_reinit", false);

  declare_parameter("map_odom_publish_rate", 20.0);
  declare_parameter("transform_tolerance", 0.1);

  declare_parameter("enable_odom_consistency_gate", true);
  declare_parameter("odom_consistency_trans_margin", 0.05);
  declare_parameter("odom_consistency_yaw_margin_deg", 1.0);

  declare_parameter("enable_static_ndt_reject", true);
  declare_parameter("static_odom_trans_threshold", 0.01);
  declare_parameter("static_odom_yaw_threshold_deg", 0.3);
  declare_parameter("static_ndt_trans_threshold", 0.025);
  declare_parameter("static_ndt_yaw_threshold_deg", 0.8);

  declare_parameter("enable_backward_reject", true);
  declare_parameter("backward_correction_threshold", 0.03);
}

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  localized_ = false;
  consecutive_success_count_ = 0;
  consecutive_reject_count_ = 0;
  odom_initialized_ = false;
  map_recieved_ = false;
  initialpose_recieved_ = false;

  map_odom_initialized_ = false;
  last_map_odom_.setIdentity();

  odom_pose_at_last_accept_initialized_ = false;
  odom_pose_at_last_accept_.setIdentity();

  if (path_ptr_) {
    path_ptr_->poses.clear();
    path_ptr_->header.frame_id = global_frame_id_;
  }

  pose_pub_->on_activate();
  orin_pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();

  if (map_odom_publish_rate_ < 1.0) {
    map_odom_publish_rate_ = 1.0;
  }

  map_odom_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / map_odom_publish_rate_)),
    [this]() {
      this->publishLastMapToOdom(this->now());
    });

  RCLCPP_INFO(
    get_logger(),
    "map->odom republish timer started. rate=%.2f Hz, transform_tolerance=%.3f s",
    map_odom_publish_rate_,
    transform_tolerance_);

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile(map_path_, *map_cloud_ptr) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", map_path_.c_str());
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2 map_msg;
    pcl::toROSMsg(*map_cloud_ptr, map_msg);
    map_msg.header.frame_id = global_frame_id_;
    initial_map_pub_->publish(map_msg);

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZI>);
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
    RCLCPP_INFO(get_logger(), "Initial Map Published");
  }

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    initialPoseReceived(msg);
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (map_odom_timer_) {
    map_odom_timer_->cancel();
    map_odom_timer_.reset();
  }

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  orin_pose_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");

  initial_pose_sub_.reset();
  map_sub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  initial_map_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  orin_pose_pub_.reset();

  if (map_odom_timer_) {
    map_odom_timer_->cancel();
    map_odom_timer_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());
  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");

  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);

  get_parameter("cloud_topic", cloud_topic_);
  get_parameter("odom_topic", odom_topic_);
  get_parameter("imu_topic", imu_topic_);
  get_parameter("map_topic", map_topic_);
  get_parameter("initialpose_topic", initialpose_topic_);

  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("init_score_threshold", init_score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);

  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);

  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);

  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);

  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);

  get_parameter("reject_translation_threshold", reject_translation_threshold_);
  get_parameter("reject_yaw_threshold_deg", reject_yaw_threshold_deg_);
  get_parameter("reject_z_threshold", reject_z_threshold_);
  get_parameter("reject_roll_pitch_threshold_deg", reject_roll_pitch_threshold_deg_);

  get_parameter("localization_success_frame_num", localization_success_frame_num_);
  get_parameter("tracking_lost_frame_num", tracking_lost_frame_num_);
  get_parameter("allow_tracking_lost_reinit", allow_tracking_lost_reinit_);

  get_parameter("map_odom_publish_rate", map_odom_publish_rate_);
  get_parameter("transform_tolerance", transform_tolerance_);

  get_parameter("enable_odom_consistency_gate", enable_odom_consistency_gate_);
  get_parameter("odom_consistency_trans_margin", odom_consistency_trans_margin_);
  get_parameter("odom_consistency_yaw_margin_deg", odom_consistency_yaw_margin_deg_);

  get_parameter("enable_static_ndt_reject", enable_static_ndt_reject_);
  get_parameter("static_odom_trans_threshold", static_odom_trans_threshold_);
  get_parameter("static_odom_yaw_threshold_deg", static_odom_yaw_threshold_deg_);
  get_parameter("static_ndt_trans_threshold", static_ndt_trans_threshold_);
  get_parameter("static_ndt_yaw_threshold_deg", static_ndt_yaw_threshold_deg_);

  get_parameter("enable_backward_reject", enable_backward_reject_);
  get_parameter("backward_correction_threshold", backward_correction_threshold_);

  lidar_undistortion_.setScanPeriod(scan_period_);

  RCLCPP_INFO(
    get_logger(),
    "Parameters loaded. map=%s odom=%s base=%s cloud=%s use_pcd_map=%s set_initial_pose=%s use_odom=%s",
    global_frame_id_.c_str(),
    odom_frame_id_.c_str(),
    base_frame_id_.c_str(),
    cloud_topic_.c_str(),
    use_pcd_map_ ? "true" : "false",
    set_initial_pose_ ? "true" : "false",
    use_odom_ ? "true" : "false");
}

void PCLLocalization::initializePubSub()
{
  pose_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  orin_pose_pub_ =
    create_publisher<car_msg::msg::OrinPose>(
    "orin_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ =
    create_publisher<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ =
    create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ =
    create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initialpose_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ =
    create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ =
    create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ =
    create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));
}

void PCLLocalization::initializeRegistration()
{
  if (registration_method_ == "GICP") {
    auto gicp =
      boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>>(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  } else if (registration_method_ == "NDT") {
    auto ndt =
      boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  } else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    ndt_omp->setNumThreads(ndt_num_threads_ > 0 ? ndt_num_threads_ : omp_get_max_threads());
    registration_ = ndt_omp;
  } else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method: %s", registration_method_.c_str());
    throw std::runtime_error("Invalid registration method");
  }

  registration_->setMaximumIterations(ndt_max_iterations_);
  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

bool PCLLocalization::computeAndPublishMapToOdom(
  const tf2::Transform & T_map_base,
  const rclcpp::Time & stamp)
{
  (void)stamp;

  tf2::Transform T_odom_base;
  bool got_odom_from_msg = false;

  if (use_odom_ && odom_initialized_) {
    T_odom_base = current_odom_pose_;
    got_odom_from_msg = true;
  }

  if (!got_odom_from_msg) {
    try {
      auto tf_odom_base_msg =
        tfbuffer_.lookupTransform(odom_frame_id_, base_frame_id_, tf2::TimePointZero);
      tf2::fromMsg(tf_odom_base_msg.transform, T_odom_base);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to compute map->odom because %s->%s is unavailable: %s",
        odom_frame_id_.c_str(),
        base_frame_id_.c_str(),
        ex.what());
      return false;
    }
  }

  last_map_odom_ = T_map_base * T_odom_base.inverse();
  map_odom_initialized_ = true;

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = this->now() + rclcpp::Duration::from_seconds(transform_tolerance_);
  map_to_odom.header.frame_id = global_frame_id_;
  map_to_odom.child_frame_id = odom_frame_id_;
  map_to_odom.transform = tf2::toMsg(last_map_odom_);
  broadcaster_.sendTransform(map_to_odom);

  if (enable_debug_) {
    double roll, pitch, yaw;
    tf2::Matrix3x3(last_map_odom_.getRotation()).getRPY(roll, pitch, yaw);
    RCLCPP_INFO(
      get_logger(),
      "[map->odom] published. xyz=(%.3f, %.3f, %.3f), yaw=%.3f deg",
      last_map_odom_.getOrigin().x(),
      last_map_odom_.getOrigin().y(),
      last_map_odom_.getOrigin().z(),
      yaw * 180.0 / M_PI);
  }

  return true;
}

void PCLLocalization::publishLastMapToOdom(const rclcpp::Time & stamp)
{
  (void)stamp;

  if (!map_odom_initialized_) {
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_odom;
  map_to_odom.header.stamp = this->now() + rclcpp::Duration::from_seconds(transform_tolerance_);
  map_to_odom.header.frame_id = global_frame_id_;
  map_to_odom.child_frame_id = odom_frame_id_;
  map_to_odom.transform = tf2::toMsg(last_map_odom_);
  broadcaster_.sendTransform(map_to_odom);
}

void PCLLocalization::initialPoseReceived(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(),
      "initialpose frame_id [%s] does not match global_frame_id [%s]",
      msg->header.frame_id.c_str(),
      global_frame_id_.c_str());
    return;
  }

  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;

  localized_ = false;
  consecutive_success_count_ = 0;
  consecutive_reject_count_ = 0;

  if (use_odom_ && odom_initialized_) {
    last_odom_pose_ = current_odom_pose_;
    odom_pose_at_last_accept_ = current_odom_pose_;
    odom_pose_at_last_accept_initialized_ = true;
  } else {
    odom_pose_at_last_accept_initialized_ = false;
  }

  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  tf2::Transform T_map_base_init;
  tf2::fromMsg(msg->pose.pose, T_map_base_init);

  if (!computeAndPublishMapToOdom(T_map_base_init, this->now())) {
    RCLCPP_WARN(
      get_logger(),
      "[initialpose] pose received, but failed to publish map->odom now. "
      "It will be retried when odom is initialized.");
  }

  if (enable_debug_) {
    double roll, pitch, yaw;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_INFO(
      get_logger(),
      "[initialpose] set to (%.3f, %.3f, %.3f), yaw=%.3f deg. Reset to INITIALIZING.",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z,
      yaw * 180.0 / M_PI);
  }
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(
      get_logger(),
      "map frame_id [%s] does not match global_frame_id [%s]",
      msg->header.frame_id.c_str(),
      global_frame_id_.c_str());
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (map_cloud_ptr->empty()) {
    RCLCPP_WARN(get_logger(), "Received empty map cloud.");
    return;
  }

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;

  RCLCPP_INFO(get_logger(), "Map received. size=%ld", map_cloud_ptr->size());
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_ || !msg) {
    return;
  }

  tf2::Transform T_odom_base;
  tf2::fromMsg(msg->pose.pose, T_odom_base);

  if (!odom_initialized_) {
    last_odom_pose_ = T_odom_base;
    current_odom_pose_ = T_odom_base;
    odom_initialized_ = true;

    if (!odom_pose_at_last_accept_initialized_) {
      odom_pose_at_last_accept_ = T_odom_base;
      odom_pose_at_last_accept_initialized_ = true;
    }

    RCLCPP_INFO(get_logger(), "[odom] initialized.");

    if (initialpose_recieved_ && !map_odom_initialized_ && corrent_pose_with_cov_stamped_ptr_) {
      tf2::Transform T_map_base_init;
      tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_map_base_init);

      if (computeAndPublishMapToOdom(T_map_base_init, this->now())) {
        RCLCPP_INFO(
          get_logger(),
          "[odom] initialized and published initial map->odom.");
      } else {
        RCLCPP_WARN(
          get_logger(),
          "[odom] initialized but failed to publish initial map->odom.");
      }
    }

    return;
  }

  last_odom_pose_ = current_odom_pose_;
  current_odom_pose_ = T_odom_base;

  if (initialpose_recieved_ && !map_odom_initialized_ && corrent_pose_with_cov_stamped_ptr_) {
    tf2::Transform T_map_base_init;
    tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_map_base_init);

    if (computeAndPublishMapToOdom(T_map_base_init, this->now())) {
      RCLCPP_INFO(
        get_logger(),
        "[odom] republished missing initial map->odom.");
    }
  }
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_ || !msg) {
    return;
  }

  Eigen::Vector3f angular_velo(
    static_cast<float>(msg->angular_velocity.x),
    static_cast<float>(msg->angular_velocity.y),
    static_cast<float>(msg->angular_velocity.z));

  Eigen::Vector3f acc(
    static_cast<float>(msg->linear_acceleration.x),
    static_cast<float>(msg->linear_acceleration.y),
    static_cast<float>(msg->linear_acceleration.z));

  Eigen::Quaternionf quat(
    static_cast<float>(msg->orientation.w),
    static_cast<float>(msg->orientation.x),
    static_cast<float>(msg->orientation.y),
    static_cast<float>(msg->orientation.z));

  if (quat.norm() < 1e-6f) {
    quat = Eigen::Quaternionf::Identity();
  } else {
    quat.normalize();
  }

  const double imu_time =
    static_cast<double>(msg->header.stamp.sec) +
    static_cast<double>(msg->header.stamp.nanosec) * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);
}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!map_recieved_ || !initialpose_recieved_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "[cloud] waiting. map_recieved=%s initialpose_recieved=%s. "
      "No localization will run before both are true.",
      map_recieved_ ? "true" : "false",
      initialpose_recieved_ ? "true" : "false");

    publishLastMapToOdom(this->now());
    return;
  }

  if (use_odom_ && !odom_initialized_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "use_odom is true, but odom has not been initialized yet.");
  }

  const std::string cloud_frame = msg->header.frame_id;
  const rclcpp::Time stamp(msg->header.stamp);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (cloud_ptr->empty()) {
    RCLCPP_WARN(get_logger(), "Received empty cloud.");
    publishLastMapToOdom(this->now());
    return;
  }

  if (use_imu_) {
    const double received_time =
      static_cast<double>(msg->header.stamp.sec) +
      static_cast<double>(msg->header.stamp.nanosec) * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
    new pcl::PointCloud<pcl::PointXYZI>);
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI> tmp;
  tmp.reserve(filtered_cloud_ptr->size());

  for (const auto & p : filtered_cloud_ptr->points) {
    const double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }

  if (tmp.empty()) {
    handleReject("filtered cloud is empty");
    publishLastMapToOdom(this->now());
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(
    new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  tf2::Transform T_map_base_last;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, T_map_base_last);

  tf2::Transform T_pred = T_map_base_last;

  if (
    localized_ &&
    use_odom_ &&
    odom_initialized_ &&
    odom_pose_at_last_accept_initialized_)
  {
    const tf2::Transform delta_odom =
      odom_pose_at_last_accept_.inverse() * current_odom_pose_;

    T_pred = T_map_base_last * delta_odom;

    if (enable_debug_) {
      double roll, pitch, yaw;
      tf2::Matrix3x3(T_pred.getRotation()).getRPY(roll, pitch, yaw);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[predict] use odom since last accepted scan. pred=(%.3f, %.3f, %.3f), yaw=%.3f deg",
        T_pred.getOrigin().x(),
        T_pred.getOrigin().y(),
        T_pred.getOrigin().z(),
        yaw * 180.0 / M_PI);
    }
  } else if (enable_debug_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "[predict] use last pose as init guess.");
  }

  // ==============================
  // 关键修改：
  // PCL source cloud 在 cloud_frame，比如 rslidar。
  // 所以 NDT 初值必须是 map->cloud_frame，而不是 map->base_link。
  // ==============================
  tf2::Transform T_base_cloud;
  tf2::Transform T_cloud_base;
  T_base_cloud.setIdentity();
  T_cloud_base.setIdentity();

  if (cloud_frame != base_frame_id_) {
    try {
      auto tf_base_cloud_msg =
        tfbuffer_.lookupTransform(base_frame_id_, cloud_frame, tf2::TimePointZero);

      tf2::fromMsg(tf_base_cloud_msg.transform, T_base_cloud);
      T_cloud_base = T_base_cloud.inverse();
    } catch (const tf2::TransformException & ex) {
      handleReject(
        std::string("No TF ") + base_frame_id_ + " -> " + cloud_frame + ", " + ex.what());
      publishLastMapToOdom(this->now());
      return;
    }
  }

  const tf2::Transform T_map_cloud_pred = T_pred * T_base_cloud;
  const Eigen::Matrix4f init_guess = tf2TransformToEigenMatrix(T_map_cloud_pred);

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
    new pcl::PointCloud<pcl::PointXYZI>);
  registration_->align(*output_cloud, init_guess);

  if (!registration_->hasConverged()) {
    handleReject(localized_ ? "[TRACKING] registration not converged"
                            : "[INITIALIZING] registration not converged");
    publishLastMapToOdom(this->now());
    return;
  }

  const double fitness_score = registration_->getFitnessScore();
  const double active_score_threshold = localized_ ? score_threshold_ : init_score_threshold_;

  if (fitness_score > active_score_threshold) {
    handleReject(
      (localized_ ? "[TRACKING] fitness too large: " : "[INITIALIZING] fitness too large: ") +
      std::to_string(fitness_score));
    publishLastMapToOdom(this->now());
    return;
  }

  const Eigen::Matrix4f T_map_cloud_e = registration_->getFinalTransformation();
  tf2::Transform T_map_cloud = eigenMatrixToTf2Transform(T_map_cloud_e);

  const tf2::Transform T_map_base = T_map_cloud * T_cloud_base;

  if (
    localized_ &&
    use_odom_ &&
    odom_initialized_ &&
    odom_pose_at_last_accept_initialized_)
  {
    const tf2::Transform T_odom_delta =
      odom_pose_at_last_accept_.inverse() * current_odom_pose_;

    const tf2::Transform T_ndt_delta =
      T_map_base_last.inverse() * T_map_base;

    const tf2::Transform T_corr =
      T_pred.inverse() * T_map_base;

    const double odom_dx = T_odom_delta.getOrigin().x();
    const double odom_dy = T_odom_delta.getOrigin().y();
    const double odom_dtrans = std::sqrt(odom_dx * odom_dx + odom_dy * odom_dy);

    const double ndt_dx = T_ndt_delta.getOrigin().x();
    const double ndt_dy = T_ndt_delta.getOrigin().y();
    const double ndt_dtrans = std::sqrt(ndt_dx * ndt_dx + ndt_dy * ndt_dy);

    const double corr_dx = T_corr.getOrigin().x();
    const double corr_dy = T_corr.getOrigin().y();
    const double corr_dtrans = std::sqrt(corr_dx * corr_dx + corr_dy * corr_dy);

    double odom_roll, odom_pitch, odom_yaw;
    double ndt_roll, ndt_pitch, ndt_yaw;
    double corr_roll, corr_pitch, corr_yaw;

    tf2::Matrix3x3(T_odom_delta.getRotation()).getRPY(
      odom_roll, odom_pitch, odom_yaw);
    tf2::Matrix3x3(T_ndt_delta.getRotation()).getRPY(
      ndt_roll, ndt_pitch, ndt_yaw);
    tf2::Matrix3x3(T_corr.getRotation()).getRPY(
      corr_roll, corr_pitch, corr_yaw);

    const double odom_yaw_deg = std::fabs(odom_yaw) * 180.0 / M_PI;
    const double ndt_yaw_deg = std::fabs(ndt_yaw) * 180.0 / M_PI;
    const double corr_yaw_deg = std::fabs(corr_yaw) * 180.0 / M_PI;

    if (enable_static_ndt_reject_) {
      const bool odom_static =
        odom_dtrans < static_odom_trans_threshold_ &&
        odom_yaw_deg < static_odom_yaw_threshold_deg_;

      if (
        odom_static &&
        (corr_dtrans > static_ndt_trans_threshold_ ||
         corr_yaw_deg > static_ndt_yaw_threshold_deg_))
      {
        handleSoftReject(
          "[TRACKING] static reject. odom_d=" + std::to_string(odom_dtrans) +
          " corr_d=" + std::to_string(corr_dtrans) +
          " corr_yaw_deg=" + std::to_string(corr_yaw_deg));
        publishLastMapToOdom(this->now());
        return;
      }
    }

    if (enable_backward_reject_) {
      if (
        odom_dx > -0.005 &&
        ndt_dx < -backward_correction_threshold_)
      {
        handleSoftReject(
          "[TRACKING] backward reject. odom_dx=" + std::to_string(odom_dx) +
          " ndt_dx=" + std::to_string(ndt_dx) +
          " corr_dx=" + std::to_string(corr_dx));
        publishLastMapToOdom(this->now());
        return;
      }
    }

    if (enable_odom_consistency_gate_) {
      const double allowed_trans =
        odom_dtrans + odom_consistency_trans_margin_;

      const double allowed_yaw_deg =
        odom_yaw_deg + odom_consistency_yaw_margin_deg_;

      if (
        ndt_dtrans > allowed_trans ||
        ndt_yaw_deg > allowed_yaw_deg)
      {
        handleSoftReject(
          "[TRACKING] odom consistency reject. odom_d=" + std::to_string(odom_dtrans) +
          " ndt_d=" + std::to_string(ndt_dtrans) +
          " allowed_d=" + std::to_string(allowed_trans) +
          " odom_yaw_deg=" + std::to_string(odom_yaw_deg) +
          " ndt_yaw_deg=" + std::to_string(ndt_yaw_deg));
        publishLastMapToOdom(this->now());
        return;
      }
    }
  }

  if (localized_) {
    const tf2::Transform T_diff = T_pred.inverse() * T_map_base;

    const double dx = T_diff.getOrigin().x();
    const double dy = T_diff.getOrigin().y();
    const double dz = T_diff.getOrigin().z();
    const double dtrans = std::sqrt(dx * dx + dy * dy);

    double droll, dpitch, dyaw;
    tf2::Matrix3x3(T_diff.getRotation()).getRPY(droll, dpitch, dyaw);

    const double dyaw_deg = std::fabs(dyaw) * 180.0 / M_PI;
    const double droll_deg = std::fabs(droll) * 180.0 / M_PI;
    const double dpitch_deg = std::fabs(dpitch) * 180.0 / M_PI;

    if (
      dtrans > reject_translation_threshold_ ||
      std::fabs(dz) > reject_z_threshold_ ||
      dyaw_deg > reject_yaw_threshold_deg_ ||
      droll_deg > reject_roll_pitch_threshold_deg_ ||
      dpitch_deg > reject_roll_pitch_threshold_deg_)
    {
      handleReject(
        "[TRACKING] jump too large. dtrans=" + std::to_string(dtrans) +
        " dz=" + std::to_string(dz) +
        " dyaw_deg=" + std::to_string(dyaw_deg));
      publishLastMapToOdom(this->now());
      return;
    }
  }

  tf2::Transform T_odom_base;
  bool got_odom_from_msg = false;

  if (use_odom_ && odom_initialized_) {
    T_odom_base = current_odom_pose_;
    got_odom_from_msg = true;
  }

  if (!got_odom_from_msg) {
    try {
      auto tf_odom_base_msg =
        tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::fromMsg(tf_odom_base_msg.transform, T_odom_base);
    } catch (const tf2::TransformException & ex1) {
      try {
        auto tf_odom_base_msg =
          tfbuffer_.lookupTransform(odom_frame_id_, base_frame_id_, tf2::TimePointZero);
        tf2::fromMsg(tf_odom_base_msg.transform, T_odom_base);
        RCLCPP_WARN(
          get_logger(),
          "lookupTransform(odom->%s) fallback TimePointZero: %s",
          base_frame_id_.c_str(),
          ex1.what());
      } catch (const tf2::TransformException & ex2) {
        handleReject(
          std::string("TF lookup failed (") + odom_frame_id_ + "->" + base_frame_id_ + "): " +
          ex2.what());
        publishLastMapToOdom(this->now());
        return;
      }
    }
  }

  last_map_odom_ = T_map_base * T_odom_base.inverse();
  map_odom_initialized_ = true;

  if (use_odom_ && odom_initialized_) {
    odom_pose_at_last_accept_ = current_odom_pose_;
    odom_pose_at_last_accept_initialized_ = true;
  }

  consecutive_reject_count_ = 0;

  if (!localized_) {
    consecutive_success_count_++;

    RCLCPP_INFO(
      get_logger(),
      "[INITIALIZING] success %d / %d, fitness=%.4f",
      consecutive_success_count_,
      localization_success_frame_num_,
      fitness_score);

    if (consecutive_success_count_ >= localization_success_frame_num_) {
      localized_ = true;
      RCLCPP_INFO(get_logger(), "Localization locked. Switch to TRACKING mode.");
    }
  }

  publishLastMapToOdom(this->now());

  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
    *corrent_pose_with_cov_stamped_ptr_;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = global_frame_id_;
  pose_msg.pose.pose.position.x = T_map_base.getOrigin().x();
  pose_msg.pose.pose.position.y = T_map_base.getOrigin().y();
  pose_msg.pose.pose.position.z = T_map_base.getOrigin().z();
  pose_msg.pose.pose.orientation = tf2::toMsg(T_map_base.getRotation());

  corrent_pose_with_cov_stamped_ptr_ =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_msg);
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  car_msg::msg::OrinPose current_orin_pose;
  current_orin_pose.header.stamp = msg->header.stamp;
  current_orin_pose.header.frame_id = global_frame_id_;
  current_orin_pose.pose = pose_msg.pose.pose;
  orin_pose_pub_->publish(current_orin_pose);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.stamp = msg->header.stamp;
  pose_stamped.header.frame_id = global_frame_id_;
  pose_stamped.pose = pose_msg.pose.pose;
  path_ptr_->poses.push_back(pose_stamped);
  path_pub_->publish(*path_ptr_);

  last_scan_ptr_ = msg;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 2000,
    "Publish TF: %s->%s. cloud_frame=%s base=%s mode=%s fitness=%.4f",
    global_frame_id_.c_str(),
    odom_frame_id_.c_str(),
    cloud_frame.c_str(),
    base_frame_id_.c_str(),
    localized_ ? "TRACKING" : "INITIALIZING",
    fitness_score);
}

Eigen::Matrix4f PCLLocalization::tf2TransformToEigenMatrix(
  const tf2::Transform & tf) const
{
  Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
  tf2::Matrix3x3 rot_tf = tf.getBasis();
  tf2::Vector3 trans_tf = tf.getOrigin();

  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      mat(r, c) = static_cast<float>(rot_tf[r][c]);
    }
  }

  mat(0, 3) = static_cast<float>(trans_tf.x());
  mat(1, 3) = static_cast<float>(trans_tf.y());
  mat(2, 3) = static_cast<float>(trans_tf.z());

  return mat;
}

tf2::Transform PCLLocalization::eigenMatrixToTf2Transform(
  const Eigen::Matrix4f & mat) const
{
  Eigen::Matrix3d rot_mat = mat.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond q(rot_mat);

  tf2::Transform tf;
  tf.setOrigin(tf2::Vector3(
    static_cast<double>(mat(0, 3)),
    static_cast<double>(mat(1, 3)),
    static_cast<double>(mat(2, 3))));
  tf.setRotation(tf2::Quaternion(q.x(), q.y(), q.z(), q.w()));

  return tf;
}

void PCLLocalization::handleReject(const std::string & reason)
{
  consecutive_success_count_ = 0;
  consecutive_reject_count_++;

  RCLCPP_WARN(get_logger(), "%s", reason.c_str());

  if (localized_ && consecutive_reject_count_ >= tracking_lost_frame_num_) {
    if (allow_tracking_lost_reinit_) {
      localized_ = false;
      consecutive_reject_count_ = 0;
      odom_pose_at_last_accept_initialized_ = false;

      RCLCPP_WARN(
        get_logger(),
        "Tracking lost for %d consecutive frames. Back to INITIALIZING mode.",
        tracking_lost_frame_num_);
    } else {
      consecutive_reject_count_ = tracking_lost_frame_num_;

      RCLCPP_WARN(
        get_logger(),
        "Tracking rejected for %d frames, but keep TRACKING because allow_tracking_lost_reinit=false.",
        tracking_lost_frame_num_);
    }
  }
}

void PCLLocalization::handleSoftReject(const std::string & reason)
{
  consecutive_success_count_ = 0;
  consecutive_reject_count_++;

  RCLCPP_WARN(
    get_logger(),
    "%s. Keep last valid pose and stay in TRACKING.",
    reason.c_str());
}

