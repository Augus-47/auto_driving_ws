//#include "utility.h"
#include <rclcpp/rclcpp.hpp>
#include <cstring>
#include <cstdlib>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/transforms.hpp>

std::string output_type;
geometry_msgs::msg::TransformStamped transform_stamp;

static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};
static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

// rslidar和velodyne的格式有微小的区别
// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                          (uint16_t, ring, ring)(double, timestamp, timestamp))


//
enum ReturnType : uint8_t {
    INVALID = 0,
    SINGLE_STRONGEST,
    SINGLE_LAST,
    DUAL_STRONGEST_FIRST,
    DUAL_STRONGEST_LAST,
    DUAL_WEAK_FIRST,
    DUAL_WEAK_LAST,
    DUAL_ONLY,
};

struct PointXYZIRC
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  std::uint8_t intensity{0U};
  std::uint8_t return_type{0U};
  std::uint16_t channel{0U};
};       
   
POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRC,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
      std::uint8_t, return_type, return_type)(std::uint16_t, channel, channel))
  
// velodyne的点云格式
struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D;

    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
                                   (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)(float, time, time)
)

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIR,
                                   (float, x, x)(float, y, y)
                                           (float, z, z)(float, intensity, intensity)
                                           (uint16_t, ring, ring)
)

rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subRobosensePC;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRobosensePC;

template<typename T>
bool has_nan(T point) {
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
        return true;
    } else {
        return false;
    }
}

template<typename T>
void publish_points(T &new_pc, const sensor_msgs::msg::PointCloud2::SharedPtr old_msg) {
    // pc properties
    new_pc->is_dense = true;
    // publish
    sensor_msgs::msg::PointCloud2::SharedPtr pc_new_msg(new sensor_msgs::msg::PointCloud2());
    pcl::toROSMsg(*new_pc, *pc_new_msg);
    (*pc_new_msg).header = (*old_msg).header;
    (*pc_new_msg).header.frame_id = "velodyne_top";

    pubRobosensePC->publish(*pc_new_msg);
}

void rsHandler_XYZI(sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_new(new pcl::PointCloud<VelodynePointXYZIR>());
    pcl::fromROSMsg(*pc_msg, *pc);

    // to new pointcloud
    for (int point_id = 0; point_id < pc->points.size(); ++point_id) {
        if (has_nan(pc->points[point_id]))
            continue;

        VelodynePointXYZIR new_point;
        new_point.x = pc->points[point_id].x;
        new_point.y = pc->points[point_id].y;
        new_point.z = pc->points[point_id].z;
        new_point.intensity = pc->points[point_id].intensity;
        // remap ring id
        if (pc->height == 16) {
            new_point.ring = RING_ID_MAP_16[point_id / pc->width];
        } else if (pc->height == 128) {
            new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
        }
        pc_new->points.push_back(new_point);
    }

    publish_points(pc_new, pc_msg);
}


template<typename T_in_p, typename T_out_p>
void handle_pc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
                   const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {

    // to new pointcloud
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        // 判断xyz有无穷大?,有则跳过
        if (has_nan(pc_in->points[point_id]))
            continue;
        // xyzi的赋值
        T_out_p new_point;
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
        // 计算相对于第一个点的相对时间
        pc_out->points.push_back(new_point);
    }
}

template<typename T_in_p, typename T_out_p>
void handle_irc_msg(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in, const typename pcl::PointCloud<T_out_p>::Ptr &pc_out){

    // to new pointcloud
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        // 判断xyz有无穷大?,有则跳过
        if (has_nan(pc_in->points[point_id]))
            continue;
        // xyzirc 的赋值
        T_out_p new_point;
        new_point.x = pc_in->points[point_id].x;
        new_point.y = pc_in->points[point_id].y;
        new_point.z = pc_in->points[point_id].z;
        new_point.intensity = pc_in->points[point_id].intensity;
        new_point.return_type = ReturnType::SINGLE_STRONGEST;
        // 计算相对于第一个点的相对时间
        pc_out->points.push_back(new_point);
    }

}

template<typename T_in_p, typename T_out_p>
void add_c(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        // 跳过nan点
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 不理解
        pc_out->points[valid_point_id++].channel = pc_in->points[point_id].ring;
    }
}

template<typename T_in_p, typename T_out_p>
void add_ring(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        // 跳过nan点
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 
        pc_out->points[valid_point_id++].ring = pc_in->points[point_id].ring;
    }
}

template<typename T_in_p, typename T_out_p>
void add_time(const typename pcl::PointCloud<T_in_p>::Ptr &pc_in,
              const typename pcl::PointCloud<T_out_p>::Ptr &pc_out) {
    // to new pointcloud
    int valid_point_id = 0;
    for (int point_id = 0; point_id < pc_in->points.size(); ++point_id) {
        if (has_nan(pc_in->points[point_id]))
            continue;
        // 跳过nan点
        pc_out->points[valid_point_id++].time = float(pc_in->points[point_id].timestamp - pc_in->points[0].timestamp);
    }
}

void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {

    pcl::PointCloud<RsPointXYZIRT>::Ptr pc_in(new pcl::PointCloud<RsPointXYZIRT>());
    pcl::fromROSMsg((*pc_msg), *pc_in);

    if (output_type == "XYZIRT") {
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIRT>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        add_time<RsPointXYZIRT, VelodynePointXYZIRT>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZIR") {
        pcl::PointCloud<VelodynePointXYZIR>::Ptr pc_out(new pcl::PointCloud<VelodynePointXYZIR>());
        handle_pc_msg<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        add_ring<RsPointXYZIRT, VelodynePointXYZIR>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZI") {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZI>());
        handle_pc_msg<RsPointXYZIRT, pcl::PointXYZI>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);
    } else if (output_type == "XYZIRC"){
        pcl::PointCloud<PointXYZIRC>::Ptr pc_out(new pcl::PointCloud<PointXYZIRC>());
        handle_irc_msg<RsPointXYZIRT, PointXYZIRC>(pc_in, pc_out);// 处理了xyzir
        add_c<RsPointXYZIRT, PointXYZIRC>(pc_in, pc_out);
        publish_points(pc_out, pc_msg);// 发布base_link坐标系点云
    }
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("rs_converter");

    // 从参数服务器读取参数
    node->declare_parameter("input_format", "XYZIRT");
    node->declare_parameter("output_format", "XYZIRT");
    
    std::string input_format = node->get_parameter("input_format").as_string();
    std::string output_format = node->get_parameter("output_format").as_string();
    output_type = output_format;

    if (input_format != "XYZI" && input_format != "XYZIRT") {
        RCLCPP_ERROR(node->get_logger(), "Unsupported input pointcloud type. Currently only support XYZI and XYZIRT.");
        exit(1);
    }

    if (input_format == "XYZI") {
        subRobosensePC = node->create_subscription<sensor_msgs::msg::PointCloud2>("/points_raw", 1, rsHandler_XYZI);
    } else if (input_format == "XYZIRT") {
        subRobosensePC = node->create_subscription<sensor_msgs::msg::PointCloud2>("/points_raw", 1, rsHandler_XYZIRT);
    }
    
    pubRobosensePC = node->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 2);
    RCLCPP_INFO(node->get_logger(), "Listening to /points_raw ......");
    rclcpp::spin(node);
    return 0;
}

