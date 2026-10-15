// // params,获取参数，车辆形状
// NodeParam node_param_;
// VehicleShape vehicle_shape_;


// const auto planner_common_param = getPlannerCommonParam();
// const auto algo_name = node_param_.planning_algorithm;
// algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);

// // 设置地图
// algo_->setMap(*occupancy_grid_);

// // 获取在当前occ上的当前位姿和目标位姿
// const auto current_pose_in_costmap_frame = utils::transform_pose(current_pose_.pose,getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));
// const auto goal_pose_in_costmap_frame = utils::transform_pose(goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));


// // 执行局部路径规划
// algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);