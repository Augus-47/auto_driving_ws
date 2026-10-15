#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy
)

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def quat_norm(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x/n, y/n, z/n, w/n)

def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz
    )

def quat_inv(q):
    x, y, z, w = q  # assume normalized
    return (-x, -y, -z, w)

def quat_rotate(q, v):
    x, y, z = v
    qv = (x, y, z, 0.0)
    return quat_mul(quat_mul(q, qv), quat_inv(q))[:3]

def tf_compose(p1, q1, p2, q2):
    q = quat_mul(q1, q2)
    p2r = quat_rotate(q1, p2)
    p = (p1[0] + p2r[0], p1[1] + p2r[1], p1[2] + p2r[2])
    return p, q

def tf_inverse(p, q):
    qi = quat_inv(q)
    pinv = quat_rotate(qi, (-p[0], -p[1], -p[2]))
    return pinv, qi

def yaw_from_quat(q):
    x, y, z, w = q
    siny_cosp = 2.0 * (w*z + x*y)
    cosy_cosp = 1.0 - 2.0 * (y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))


class MapOdomRepublisher(Node):
    def __init__(self):
        super().__init__('map_odom_republisher')

        # parameters
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value

        self.pose_topic = self.declare_parameter('pose_topic', '/pcl_pose').value
        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value

        self.publish_rate = float(self.declare_parameter('publish_rate', 50.0).value)
        self.future_offset = float(self.declare_parameter('future_offset', 0.10).value)
        self.force_2d = bool(self.declare_parameter('force_2d', True).value)

        # QoS match /pcl_pose: RELIABLE + TRANSIENT_LOCAL + depth=1
        self.pose_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # odom: 通常用 RELIABLE/VOLATILE 足够
        self.odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.br = TransformBroadcaster(self)

        self._lock = threading.Lock()
        self._have_map_base = False
        self._have_odom_base = False
        self._have_map_odom = False

        self._map_T_base_p = (0.0, 0.0, 0.0)
        self._map_T_base_q = (0.0, 0.0, 0.0, 1.0)

        self._odom_T_base_p = (0.0, 0.0, 0.0)
        self._odom_T_base_q = (0.0, 0.0, 0.0, 1.0)

        self._map_T_odom_p = (0.0, 0.0, 0.0)
        self._map_T_odom_q = (0.0, 0.0, 0.0, 1.0)

        # subscribe
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self._pose_cb, self.pose_qos)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, self.odom_qos)

        # timer publish
        period = 1.0 / max(self.publish_rate, 1.0)
        self.create_timer(period, self._publish_timer)

        self.get_logger().info(
            f"[map_odom_republisher] pose_topic={self.pose_topic}(PoseWithCovarianceStamped, RELIABLE+TRANSIENT_LOCAL), "
            f"odom_topic={self.odom_topic}, publish_rate={self.publish_rate}Hz, "
            f"future_offset={self.future_offset}s, force_2d={self.force_2d}"
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        # map -> base_link pose
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        with self._lock:
            self._map_T_base_p = (p.x, p.y, p.z)
            self._map_T_base_q = quat_norm((o.x, o.y, o.z, o.w))
            self._have_map_base = True
        self._update_map_odom()

    def _odom_cb(self, msg: Odometry):
        # odom -> base_link pose
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        with self._lock:
            self._odom_T_base_p = (p.x, p.y, p.z)
            self._odom_T_base_q = quat_norm((o.x, o.y, o.z, o.w))
            self._have_odom_base = True
        self._update_map_odom()

    def _update_map_odom(self):
        with self._lock:
            if not (self._have_map_base and self._have_odom_base):
                return
            map_p, map_q = self._map_T_base_p, self._map_T_base_q
            odom_p, odom_q = self._odom_T_base_p, self._odom_T_base_q

        # map_T_odom = map_T_base * inverse(odom_T_base)
        inv_odom_p, inv_odom_q = tf_inverse(odom_p, odom_q)
        map_odom_p, map_odom_q = tf_compose(map_p, map_q, inv_odom_p, inv_odom_q)
        map_odom_q = quat_norm(map_odom_q)

        if self.force_2d:
            yaw = yaw_from_quat(map_odom_q)
            map_odom_p = (map_odom_p[0], map_odom_p[1], 0.0)
            map_odom_q = quat_from_yaw(yaw)

        with self._lock:
            self._map_T_odom_p = map_odom_p
            self._map_T_odom_q = map_odom_q
            self._have_map_odom = True

    def _publish_timer(self):
        with self._lock:
            if not self._have_map_odom:
                return
            p = self._map_T_odom_p
            q = self._map_T_odom_q

        tf = TransformStamped()
        t = self.get_clock().now()
        if self.future_offset > 0.0:
            t = t + rclpy.duration.Duration(seconds=self.future_offset)

        tf.header.stamp = t.to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.odom_frame

        tf.transform.translation.x = float(p[0])
        tf.transform.translation.y = float(p[1])
        tf.transform.translation.z = float(p[2])

        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])

        self.br.sendTransform(tf)


def main():
    rclpy.init()
    node = MapOdomRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

