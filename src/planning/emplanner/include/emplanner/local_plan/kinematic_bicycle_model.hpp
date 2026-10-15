#ifndef KINEMATIC_BICYCLE_MODEL_HPP_
#define KINEMATIC_BICYCLE_MODEL_HPP_

#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <vector>

namespace kinematic_bicycle_model
{

static constexpr double eps = 0.001;

inline double getTurningRadius(const double base_length, const double steering_angle)
{
  return base_length / std::tan(steering_angle);
}

inline geometry_msgs::msg::Pose getPose(
  const geometry_msgs::msg::Pose & current_pose, const double base_length,
  const double steering_angle, const double distance)
{
  auto pose = current_pose;
  const double yaw = tf2::getYaw(current_pose.orientation);
  if (std::abs(steering_angle) < eps) {
    pose.position.x += distance * std::cos(yaw);
    pose.position.y += distance * std::sin(yaw);
    return pose;
  }

  const double R = getTurningRadius(base_length, steering_angle);
  const double beta = distance / R;
  pose.position.x += (R * std::sin(yaw + beta) - R * std::sin(yaw));
  pose.position.y += (R * std::cos(yaw) - R * std::cos(yaw + beta));
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw + beta);
  pose.orientation = tf2::toMsg(q);
  return pose;
}

}
#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__KINEMATIC_BICYCLE_MODEL_HPP_
