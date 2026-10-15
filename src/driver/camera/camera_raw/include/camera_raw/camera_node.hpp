#ifndef IMAGE_PUBLISHER_HPP
#define IMAGE_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <thread>

class ImagePublisher : public rclcpp::Node {

public:
    ImagePublisher();
    ~ImagePublisher();

private:
    void timer_callback();
    void load_parameters();

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    std::chrono::steady_clock::time_point last_time_;
    std::chrono::steady_clock::time_point last_error_time_;

    // 配置数据
    int camera_device_; // 摄像头设备编号
    int width_;         // 图像宽度
    int height_;        // 图像高度
    int fps_;           // 帧率
    double timer_period_; // 定时器周期
    std::string topic_name_; // 话题名称

    bool is_difference; // 是否区分话题名(用于不同摄像头区分)
};

#endif // IMAGE_PUBLISHER_HPP