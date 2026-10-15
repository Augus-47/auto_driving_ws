#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;

// ros2 run sys_time stamp_relay_node 先给时间戳（如果你还想用的话）
// ros2 run sys_time sync_node          再同步

class SubscriberAndPublisher : public rclcpp::Node
{
public:
    // 同步策略类型定义（放到 public 里方便类型别名）
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::PointCloud2>;

    SubscriberAndPublisher()
    : Node("subscriber_and_publisher"),
      // ✅ 关键改动1：使用 SensorDataQoS，和相机/雷达驱动保持一致
      // 如果你仍然想用 *_stamp 话题，就保持下面两行；如果想直接用原始话题，把名字改成 /image_raw 和 /rslidar_points 即可
      image_sub_(this, "/image_raw_stamp",
                 rclcpp::SensorDataQoS().get_rmw_qos_profile()),
      lidar_sub_(this, "/rslidar_points_stamp",
                 rclcpp::SensorDataQoS().get_rmw_qos_profile())
    {
        // ✅ 关键改动2：队列长度不用 100，20 足够；越大越占内存且增加延迟
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
                    SyncPolicy(20), image_sub_, lidar_sub_);

        // （可选）如果你想限制最大时间间隔，可以放宽一点，比如 0.2s；
        // 也可以先注释掉，看一下真实 diff 再决定
        // sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.2));

        sync_->registerCallback(
            std::bind(&SubscriberAndPublisher::callback, this, _1, _2));

        // ✅ 关键改动3：发布端也用 SensorDataQoS
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "sync/img", rclcpp::SensorDataQoS());
        lidar_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "sync/lidar", rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "Sync node initialized.");
    }

private:
    // 订阅器和同步器
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub_;

    // 回调函数
    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr image,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud)
    {
        // 1. 计算时间差
        const double image_time =
            static_cast<double>(image->header.stamp.sec) +
            static_cast<double>(image->header.stamp.nanosec) * 1e-9;
        const double lidar_time =
            static_cast<double>(pointcloud->header.stamp.sec) +
            static_cast<double>(pointcloud->header.stamp.nanosec) * 1e-9;
        const double time_diff = std::abs(image_time - lidar_time);

        // 2. 设定阈值（这里保留你原来的 0.1 s，可按需要调）
        const double threshold = 0.2;

        if (time_diff > threshold) {
            // 大部分时候只是观察情况，用 DEBUG/WARN 即可
            RCLCPP_WARN(this->get_logger(),
                        "Drop unsynced pair. Diff: %.3fs (Img: %.3f, Lidar: %.3f)",
                        time_diff, image_time, lidar_time);
            return;
        }

        RCLCPP_DEBUG(this->get_logger(),
                     "Synced pair. Diff: %.3fs. Img stamp: %d.%09u, Lidar stamp: %d.%09u",
                     time_diff,
                     image->header.stamp.sec, image->header.stamp.nanosec,
                     pointcloud->header.stamp.sec, pointcloud->header.stamp.nanosec);

        // 3. 发布同步后的数据
        image_pub_->publish(*image);
        lidar_pub_->publish(*pointcloud);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SubscriberAndPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

