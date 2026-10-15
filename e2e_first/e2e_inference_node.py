#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import cv2
import numpy as np
import torch

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from car_msg.msg import VehicleState

from model import E2ESteeringNet


class E2EInferenceNode(Node):
    def __init__(self, args):
        super().__init__("e2e_inference_node")

        self.args = args
        self.bridge = CvBridge()

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}")

        ckpt = torch.load(args.model, map_location=self.device)

        self.width = int(ckpt.get("width", args.width))
        self.height = int(ckpt.get("height", args.height))
        self.max_steer_deg = float(ckpt.get("max_steer_deg", args.max_steer_deg))
        self.max_speed = float(ckpt.get("max_speed", args.max_speed))

        self.args.crop_top_ratio = float(
            ckpt.get("crop_top_ratio", args.crop_top_ratio)
        )
        self.args.crop_bottom_ratio = float(
            ckpt.get("crop_bottom_ratio", args.crop_bottom_ratio)
        )

        self.model = E2ESteeringNet().to(self.device)
        self.model.load_state_dict(ckpt["model_state"])
        self.model.eval()

        self.last_velocity = 0.0
        self.last_vehicle_time = None
        self.last_steering_cmd = args.initial_steer_deg

        self.image_sub = self.create_subscription(
            Image,
            args.image_topic,
            self.image_callback,
            10
        )

        self.vehicle_sub = self.create_subscription(
            VehicleState,
            args.vehicle_info_topic,
            self.vehicle_info_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            VehicleState,
            args.cmd_topic,
            10
        )

        self.get_logger().info("E2E inference node started.")
        self.get_logger().info(f"Model: {args.model}")
        self.get_logger().info(f"Image topic: {args.image_topic}")
        self.get_logger().info(f"Vehicle info topic: {args.vehicle_info_topic}")
        self.get_logger().info(f"Command topic: {args.cmd_topic}")
        self.get_logger().info(f"Target speed: {args.target_speed}")
        self.get_logger().info(f"Limit steer deg: {args.limit_steer_deg}")
        self.get_logger().info(f"Max delta steer deg: {args.max_delta_steer_deg}")
        self.get_logger().info(f"Steer filter alpha: {args.steer_filter_alpha}")
        self.get_logger().info(f"Steer offset deg: {args.steer_offset_deg}")
        self.get_logger().info(f"Steer scale: {args.steer_scale}")
        self.get_logger().info(f"Invert steering: {args.invert_steering}")
        self.get_logger().info(f"Initial steer deg: {args.initial_steer_deg}")
        self.get_logger().info(f"Input size: {self.width}x{self.height}")
        self.get_logger().info(
            f"Crop top: {self.args.crop_top_ratio}, "
            f"crop bottom: {self.args.crop_bottom_ratio}"
        )

    def vehicle_info_callback(self, msg):
        self.last_velocity = float(msg.velocity)
        self.last_vehicle_time = self.get_clock().now()

    def preprocess_image(self, image_bgr):
        h, w = image_bgr.shape[:2]

        y1 = int(h * self.args.crop_top_ratio)
        y2 = int(h * (1.0 - self.args.crop_bottom_ratio))

        if y2 <= y1:
            y1 = 0
            y2 = h

        image_bgr = image_bgr[y1:y2, :]
        image_bgr = cv2.resize(image_bgr, (self.width, self.height))

        image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
        image = image_rgb.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))

        image_tensor = torch.from_numpy(image).float().unsqueeze(0)

        return image_tensor

    def limit_value(self, value, min_value, max_value):
        return max(min_value, min(max_value, value))

    def smooth_and_limit_steering(self, steering_deg):
        # 1. 限制绝对转角
        steering_deg = self.limit_value(
            steering_deg,
            -self.args.limit_steer_deg,
            self.args.limit_steer_deg
        )

        # 2. 限制每帧变化量
        delta = steering_deg - self.last_steering_cmd
        delta = self.limit_value(
            delta,
            -self.args.max_delta_steer_deg,
            self.args.max_delta_steer_deg
        )

        steering_rate_limited = self.last_steering_cmd + delta

        # 3. 低通滤波
        alpha = self.args.steer_filter_alpha
        steering_cmd = (
            alpha * self.last_steering_cmd
            + (1.0 - alpha) * steering_rate_limited
        )

        self.last_steering_cmd = steering_cmd

        return steering_cmd

    def get_safe_velocity_input(self):
        if self.last_vehicle_time is None:
            return 0.0

        now = self.get_clock().now()
        dt = (now - self.last_vehicle_time).nanoseconds / 1e9

        if dt > self.args.vehicle_timeout:
            if self.args.verbose:
                self.get_logger().warn(
                    f"Vehicle info timeout: {dt:.3f}s, use velocity=0.0"
                )
            return 0.0

        return self.last_velocity

    def build_cmd_msg(self, velocity, steering, led="auto", brake=0.0):
        cmd = VehicleState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        cmd.velocity = float(velocity)
        cmd.steering = float(steering)
        cmd.brake = float(brake)
        cmd.gears = 0.0
        cmd.led = led
        cmd.battary = 0.0

        return cmd

    def publish_stop(self):
        stop_cmd = self.build_cmd_msg(
            velocity=0.0,
            steering=self.last_steering_cmd,
            led="stop",
            brake=self.args.stop_brake
        )

        # 多发几次，避免底盘漏接
        for _ in range(self.args.stop_publish_count):
            self.cmd_pub.publish(stop_cmd)

        self.get_logger().info("Published stop command.")

    def image_callback(self, msg):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8"
            )

            image_tensor = self.preprocess_image(image_bgr).to(self.device)

            velocity_in = self.get_safe_velocity_input()

            velocity_norm = velocity_in / self.max_speed
            velocity_norm = self.limit_value(velocity_norm, -1.0, 1.0)

            velocity_tensor = torch.tensor(
                [[velocity_norm]],
                dtype=torch.float32,
                device=self.device
            )

            with torch.no_grad():
                steering_norm_tensor = self.model(image_tensor, velocity_tensor)
                steering_norm = float(steering_norm_tensor.item())

            # 网络原始输出，单位 deg
            steering_deg_raw = steering_norm * self.max_steer_deg

            # 可选：方向取反
            if self.args.invert_steering:
                steering_deg_raw = -steering_deg_raw

            # 可选：转角放大/缩小
            steering_deg_scaled = steering_deg_raw * self.args.steer_scale

            # 可选：零点补偿
            steering_deg_with_offset = (
                steering_deg_scaled + self.args.steer_offset_deg
            )

            # 限幅 + 限速率 + 滤波
            steering_deg_cmd = self.smooth_and_limit_steering(
                steering_deg_with_offset
            )

            # 固定目标速度
            target_speed = float(self.args.target_speed)

            # 限制速度范围
            target_speed = self.limit_value(
                target_speed,
                0.0,
                self.args.max_cmd_speed
            )

            cmd = self.build_cmd_msg(
                velocity=target_speed,
                steering=steering_deg_cmd,
                led="auto",
                brake=0.0
            )

            self.cmd_pub.publish(cmd)

            if self.args.verbose:
                self.get_logger().info(
                    f"vel_in={velocity_in:.3f}, "
                    f"steer_norm={steering_norm:.3f}, "
                    f"steer_raw={steering_deg_raw:.2f} deg, "
                    f"steer_scaled={steering_deg_scaled:.2f} deg, "
                    f"steer_offset={self.args.steer_offset_deg:.2f} deg, "
                    f"steer_before_filter={steering_deg_with_offset:.2f} deg, "
                    f"steer_cmd={steering_deg_cmd:.2f} deg, "
                    f"target_speed={target_speed:.2f}"
                )

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--model",
        required=True,
        help="Path to best_model.pt"
    )

    parser.add_argument(
        "--image-topic",
        default="/image_raw"
    )

    parser.add_argument(
        "--vehicle-info-topic",
        default="/vehicle_info"
    )

    parser.add_argument(
        "--cmd-topic",
        default="/e2e_debug_cmd",
        help="Use /e2e_debug_cmd for shadow mode, /control_cmd for real control"
    )

    parser.add_argument(
        "--target-speed",
        type=float,
        default=0.0,
        help="Fixed target speed for first version"
    )

    parser.add_argument(
        "--max-cmd-speed",
        type=float,
        default=0.30,
        help="Absolute max speed command allowed"
    )

    parser.add_argument(
        "--limit-steer-deg",
        type=float,
        default=8.0,
        help="Safety steering limit in degrees"
    )

    parser.add_argument(
        "--max-delta-steer-deg",
        type=float,
        default=0.5,
        help="Max steering change per image frame"
    )

    parser.add_argument(
        "--steer-filter-alpha",
        type=float,
        default=0.9,
        help="Low-pass filter alpha, larger means smoother but slower"
    )

    parser.add_argument(
        "--steer-offset-deg",
        type=float,
        default=0.0,
        help="Add steering offset in degrees after network prediction"
    )

    parser.add_argument(
        "--steer-scale",
        type=float,
        default=1.0,
        help="Scale steering output before offset"
    )

    parser.add_argument(
        "--invert-steering",
        action="store_true",
        help="Invert steering sign if control direction is opposite"
    )

    parser.add_argument(
        "--initial-steer-deg",
        type=float,
        default=0.0,
        help="Initial steering command for smoothing"
    )

    parser.add_argument(
        "--vehicle-timeout",
        type=float,
        default=0.5,
        help="If vehicle_info is older than this, use velocity=0.0"
    )

    parser.add_argument(
        "--stop-brake",
        type=float,
        default=1.0,
        help="Brake value in stop command when node exits"
    )

    parser.add_argument(
        "--stop-publish-count",
        type=int,
        default=5,
        help="How many stop commands to publish on exit"
    )

    parser.add_argument(
        "--no-stop-on-exit",
        action="store_true",
        help="Do not publish stop command when node exits"
    )

    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=180)

    parser.add_argument("--max-steer-deg", type=float, default=30.0)
    parser.add_argument("--max-speed", type=float, default=1.0)

    parser.add_argument("--crop-top-ratio", type=float, default=0.25)
    parser.add_argument("--crop-bottom-ratio", type=float, default=0.0)

    parser.add_argument("--verbose", action="store_true")

    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = E2EInferenceNode(args)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if not args.no_stop_on_exit:
            try:
                node.publish_stop()
            except Exception as e:
                node.get_logger().error(f"Failed to publish stop command: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

