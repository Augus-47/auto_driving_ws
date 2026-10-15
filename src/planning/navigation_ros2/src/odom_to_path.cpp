#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class OdomToPath : public rclcpp::Node
{
public:
  OdomToPath() : Node("odom_to_path")
  {
    // 初始化Path消息
    path_msg_.header.frame_id = "map";

    // 创建发布者和订阅者
    pub_ = this->create_publisher<nav_msgs::msg::Path>("/odom_path", 10);
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&OdomToPath::odom_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "OdomToPath node has been initialized");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = odom->pose.pose.position.x;
    pose_stamped.pose.position.y = odom->pose.pose.position.y;
    pose_stamped.pose.position.z = odom->pose.pose.position.z;

    pose_stamped.pose.orientation = odom->pose.pose.orientation;

    pose_stamped.header.stamp = this->now();
    pose_stamped.header.frame_id = "map";

    path_msg_.poses.push_back(pose_stamped);
    path_msg_.header.stamp = this->now();
    path_msg_.header.frame_id = "map";
    
    pub_->publish(path_msg_);
  }

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Path path_msg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomToPath>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 