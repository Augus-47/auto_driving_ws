#include "camera_raw/camera_node.hpp"

// 相机配置初始化
ImagePublisher::ImagePublisher() : Node("camera_pub") {

    // 加载参数
    load_parameters();

    std::string gst_pipeline = 
        "v4l2src device=/dev/video" + std::to_string(camera_device_) + " ! "
        "video/x-raw,format=UYVY,width=" + std::to_string(width_) + ",height=" + std::to_string(height_) + " ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink";

    std::string topic_name_ = "/image" + std::to_string(camera_device_);
    if(is_difference){
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
    }else{
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);
    }
    
    // 配置相机
    cap_ = cv::VideoCapture(gst_pipeline, cv::CAP_GSTREAMER);
    RCLCPP_INFO(this->get_logger(), "使用 GStreamer 管道: %s", gst_pipeline.c_str());
    
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "摄像头未成功打开！");
    } else {
        RCLCPP_INFO(this->get_logger(), "摄像头已成功打开");
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_period_),
        std::bind(&ImagePublisher::timer_callback, this)
    );
    last_time_ = std::chrono::steady_clock::now();
    last_error_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
}

// 参数加载
void ImagePublisher::load_parameters() {
    // 这里可以加载参数，例如摄像头设备编号、图像宽度、高度和帧率等
    this->declare_parameter("camera_device", 0);
    this->declare_parameter("width", 1920);
    this->declare_parameter("height", 1080);
    this->declare_parameter("fps", 30);
    this->declare_parameter("timer_period", 0.1);
    this->declare_parameter("is_difference", false);

    this->get_parameter("camera_device", camera_device_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("fps", fps_);
    this->get_parameter("timer_period", timer_period_);
    this->get_parameter("is_difference", is_difference);

    RCLCPP_INFO(this->get_logger(), "摄像头参数: device=%d, width=%d, height=%d, fps=%d, timer_period=%.2f",
                camera_device_, width_, height_, fps_, timer_period_);
}

ImagePublisher::~ImagePublisher() {
    cap_.release();
    cv::destroyAllWindows();
}

void ImagePublisher::timer_callback() {
    auto current_time = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(current_time - last_time_).count();
    last_time_ = current_time;
    RCLCPP_INFO(this->get_logger(), "当前帧间隔时间: %.4f 秒 ≈ %.1f Hz", elapsed, 1.0/elapsed);

    if (!cap_.isOpened()) {
        if (std::chrono::duration<double>(current_time - last_error_time_).count() > 5.0) {
            RCLCPP_WARN(this->get_logger(), "摄像头仍未打开");
            last_error_time_ = current_time;
        }
        return;
    }

    cv::Mat frame;
    bool ret = cap_.read(frame);
    if (!ret) {
        if (std::chrono::duration<double>(current_time - last_error_time_).count() > 5.0) {
            RCLCPP_WARN(this->get_logger(), "读取图像失败");
            last_error_time_ = current_time;
        }
        return;
    }

    // 发布图像
    auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    img_msg->header.stamp = this->now();
    img_msg->header.frame_id = "camera_raw";
    publisher_->publish(*img_msg);
    RCLCPP_INFO(this->get_logger(), "发布图像帧成功: %dx%d", frame.cols, frame.rows);
}
