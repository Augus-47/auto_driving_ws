#include "car_cmd_interface/vehicle_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

// 说明：
// 1. 这版把速度编码改成 int16_t 有符号速度，单位 0.001 m/s。
//    例如：
//      +0.05 m/s -> +50  -> 0x0032 -> 小端 "3200"
//      -0.05 m/s -> -50  -> 0xFFCE -> 小端 "ceff"
// 2. 不再使用 proto_speed |= 0x8000。
//    你原来的 -0.05 会被打包成 0x8032，如果底盘按 int16_t 解析，等价于 -32718，车就会猛退。
// 3. /cmd_vel 入口不再强制把小速度抬到 0.12m/s，否则 Nav2 终端微调的 -0.02~-0.08 会全部变得太快。

namespace
{

double clamp_speed_for_chassis(double v)
{
  // 按你的巡检车先保守限速。
  // 正常 Nav2 控制器 max_speed 推荐 0.26，这里给前进 0.30 留一点余量。
  // 倒车终端微调推荐 0.05~0.08，这里限制到 0.08，防止误命令猛倒。
  constexpr double kMaxForwardSpeed = 0.70;  // m/s
  constexpr double kMaxReverseSpeed = 0.12;  // m/s

  if (v > kMaxForwardSpeed) {
    return kMaxForwardSpeed;
  }
  if (v < -kMaxReverseSpeed) {
    return -kMaxReverseSpeed;
  }
  return v;
}

}  // namespace

VehicleInterface::VehicleInterface()
: Node("vehicle_interface_node"), speed_str_("0000"), angle_str_("0000")
{
  // 创建订阅器
  vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&VehicleInterface::vel_callback, this, std::placeholders::_1));

  angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
    "/angle", 10, std::bind(&VehicleInterface::angle_callback, this, std::placeholders::_1));

  cmd_sub_ = this->create_subscription<car_msg::msg::VehicleState>(
    "/control_cmd", 10, std::bind(&VehicleInterface::cmd_callback, this, std::placeholders::_1));

  // 创建发布器
  vehicle_info_pub_ = this->create_publisher<car_msg::msg::VehicleState>(
    "/vehicle_info", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&VehicleInterface::timer_callback, this));

  // 初始化 CAN 总线连接
  {
    struct sockaddr_can addr;
    struct ifreq ifr;
    int s;
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    strcpy(ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    // 设置非阻塞模式
    int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);

    if (bind(s, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      perror("连接can0失败");
      return;
    }
    sock_ = s;
  }

  this->declare_parameter<double>("wheel_base", 0.8);
  this->declare_parameter<double>("steering_offset", 0.0);
  this->declare_parameter<double>("cmd_timeout", 0.25);

  wheel_base_ = this->get_parameter("wheel_base").as_double();
  steering_offset_ = this->get_parameter("steering_offset").as_double();
  cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();

  last_cmd_time_ = this->now();
  got_cmd_ = false;
  timeout_sent_ = false;

  RCLCPP_INFO(this->get_logger(), "车辆接口初始化完成");
}

// 阿克曼转向计算
double VehicleInterface::calculate_steering_angle(double v, double w)
{
  const double eps = 1e-3;
  if (std::fabs(v) < eps || std::fabs(w) < 1e-3) {
    return 0.0;
  }

  // 关键：曲率 kappa = w / v。
  // 倒车时 v < 0，这里会自动得到正确的转向符号关系。
  const double kappa = w / v;
  double steering_rad = std::atan(wheel_base_ * kappa);
  double steering_deg = steering_rad * 180.0 / M_PI;

  steering_deg += steering_offset_;

  steering_deg = std::clamp(steering_deg, -40.00, 40.00);
  return steering_deg;
}

void VehicleInterface::vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Nav2 Cmd: v=%.3f w=%.3f",
    msg->linear.x,
    msg->angular.z);

  last_cmd_time_ = this->now();
  got_cmd_ = true;
  timeout_sent_ = false;

  double linear_x = msg->linear.x;

  // 终端微调速度本来就小，不要强制抬到 0.12。
  // 如果电机有死区，可以把 deadband 设到 0.01~0.02，但不要把非零速度强行抬太大。
  constexpr double deadband = 0.01;

  double cmd_v = 0.0;
  if (std::fabs(linear_x) < deadband) {
    cmd_v = 0.0;
  } else {
    cmd_v = clamp_speed_for_chassis(linear_x);
  }

  // ==============================
  // 关键修改：
  // 使用 int16_t 有符号速度编码，不再手动 OR 0x8000。
  // 单位：0.001 m/s。
  // ==============================
  int speed_raw = static_cast<int>(std::round(cmd_v * 1000.0));
  speed_raw = std::clamp(speed_raw, -32767, 32767);
  speed_str_ = int16_to_hex_string_little_endian(speed_raw);

  double steer_angle = calculate_steering_angle(cmd_v, msg->angular.z);

  int current_angle = static_cast<int>(std::round(steer_angle * 100.0));
  angle_str_ = int16_to_hex_string_little_endian(current_angle);

  send_cmd();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    500,
    "Send chassis cmd from /cmd_vel: cmd_v=%.3f speed_raw=%d speed_hex=%s steer=%.2f angle_hex=%s",
    cmd_v,
    speed_raw,
    speed_str_.c_str(),
    steer_angle,
    angle_str_.c_str());
}

void VehicleInterface::angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
  double angle_deg = msg->data + steering_offset_;

  angle_deg = std::clamp(angle_deg, -30.00, 30.00);

  int current_angle = static_cast<int>(std::round(angle_deg * 100.0));
  angle_str_ = int16_to_hex_string_little_endian(current_angle);

  send_cmd();
}

void VehicleInterface::cmd_callback(const car_msg::msg::VehicleState::SharedPtr msg)
{
  // /control_cmd 也必须和 /cmd_vel 使用同一套速度编码。
  double velocity = clamp_speed_for_chassis(msg->velocity);

  // ==============================
  // 关键修改：
  // 旧代码：
  //   speed_abs = abs(v) * 1000;
  //   if (v < 0) proto_speed |= 0x8000;
  //
  // 新代码：
  //   直接 int16_t signed raw = v * 1000。
  // ==============================
  int speed_raw = static_cast<int>(std::round(velocity * 1000.0));
  speed_raw = std::clamp(speed_raw, -32767, 32767);
  speed_str_ = int16_to_hex_string_little_endian(speed_raw);

  double steering = msg->steering + steering_offset_;
  steering = std::clamp(steering, -30.00, 30.00);

  int current_angle = static_cast<int>(std::round(steering * 100.0));
  angle_str_ = int16_to_hex_string_little_endian(current_angle);

  send_cmd();

  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    500,
    "Send chassis cmd from /control_cmd: velocity=%.3f speed_raw=%d speed_hex=%s steering=%.2f angle_hex=%s",
    velocity,
    speed_raw,
    speed_str_.c_str(),
    steering,
    angle_str_.c_str());
}

void VehicleInterface::send_cmd()
{
  // 发送 CAN 命令
  std::string cmd_str = "0101" + speed_str_ + angle_str_ + "0000";
  std::vector<unsigned char> cmd_data;
  for (std::size_t i = 0; i < cmd_str.size(); i += 2) {
    std::string byte_str = cmd_str.substr(i, 2);
    unsigned char byte = std::stoi(byte_str, nullptr, 16);
    cmd_data.push_back(byte);
  }

  struct can_frame frame;
  frame.can_id = 0x00000001;
  frame.can_dlc = cmd_data.size();
  for (std::size_t i = 0; i < cmd_data.size(); i++) {
    frame.data[i] = cmd_data[i];
  }

  int nbytes = send(sock_, &frame, sizeof(struct can_frame), 0);

  if (nbytes == sizeof(struct can_frame)) {
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "已发送,速度：" << speed_str_ << "，角度：" << angle_str_);
  } else {
    RCLCPP_ERROR(this->get_logger(), "fail,发送给小车控制指令失败");
  }
}

void VehicleInterface::timer_callback()
{
  struct can_frame frame;
  bool received_emergency_stop = false;

  while (true) {
    int nbytes = recv(sock_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);

    if (nbytes <= 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("recv error");
      }
      break;
    }

    process_can_frame(frame, received_emergency_stop);
  }

  // cmd_vel watchdog: timeout -> send stop
  if (got_cmd_) {
    const double age = (this->now() - last_cmd_time_).seconds();
    if (age > cmd_timeout_ && !timeout_sent_) {
      RCLCPP_WARN(
        this->get_logger(),
        "cmd_vel timeout (%.3fs > %.3fs), sending STOP!",
        age,
        cmd_timeout_);

      speed_str_ = int16_to_hex_string_little_endian(0);
      angle_str_ = int16_to_hex_string_little_endian(0);

      send_cmd();
      timeout_sent_ = true;
    }
  }
}

void VehicleInterface::process_can_frame(
  const struct can_frame & frame,
  bool & received_emergency_stop)
{
  std::stringstream ss;
  for (int i = 0; i < frame.can_dlc; i++) {
    ss << std::hex << std::setfill('0') << std::setw(2)
       << static_cast<int>(frame.data[i]);
  }
  std::string message = ss.str();

  if (message.substr(0, 4) == "0202") {
    car_msg::msg::VehicleState vehicle_info;

    // 反馈按 int16_t 有符号数解析。
    // 如果底盘反馈也是同一种协议，那么 -0.05m/s 应该反馈 raw=-50，速度=-0.05。
    int16_t raw_speed = static_cast<int16_t>(
      (static_cast<uint16_t>(frame.data[3]) << 8) |
      static_cast<uint16_t>(frame.data[2]));

    int16_t raw_angle = static_cast<int16_t>(
      (static_cast<uint16_t>(frame.data[5]) << 8) |
      static_cast<uint16_t>(frame.data[4]));

    uint8_t battery_status = frame.data[6];
    uint8_t flag = frame.data[7];
    uint8_t bit_3 = (flag >> 3) & 0x01;

    double velocity = static_cast<double>(raw_speed) / 1000.0;
    double degree = static_cast<double>(raw_angle) / 100.0;

    if (bit_3 == 1 && !received_emergency_stop) {
      received_emergency_stop = true;
      RCLCPP_WARN(this->get_logger(), "检测到急停信号！立即停止车辆");

      speed_str_ = int16_to_hex_string_little_endian(0);
      angle_str_ = int16_to_hex_string_little_endian(0);
      send_cmd();
    } else if (bit_3 == 0 && received_emergency_stop) {
      received_emergency_stop = false;
      RCLCPP_WARN(this->get_logger(), "急停解除，可以恢复运动");
    }

    rclcpp::Clock clock;
    vehicle_info.header.stamp = clock.now();
    vehicle_info.header.frame_id = "base_link";

    vehicle_info.steering = degree;
    vehicle_info.velocity = velocity;
    vehicle_info.gears = 0;
    vehicle_info.led = "no";
    vehicle_info.brake = 0;
    vehicle_info.battary = battery_status;

    if (emergency_active_) {
      vehicle_info.velocity = 0.0;
      vehicle_info_pub_->publish(vehicle_info);
      return;
    }

    vehicle_info_pub_->publish(vehicle_info);
  }
}

std::string VehicleInterface::int16_to_hex_string_little_endian(int int_value)
{
  int16_t value16 = static_cast<int16_t>(int_value);
  uint16_t u = static_cast<uint16_t>(value16);

  std::stringstream ss;
  ss << std::hex << std::setfill('0') << std::setw(2) << (u & 0xFF)
     << std::setw(2) << ((u >> 8) & 0xFF);
  return ss.str();
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VehicleInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

