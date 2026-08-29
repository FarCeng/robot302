import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros


class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        # =============================
        # Robot Parameters
        # =============================
        self.wheel_radius = 0.0290    
        self.wheelbase = 0.19

        # =============================
        # State
        # =============================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_rpm = 0.0
        self.right_rpm = 0.0

        self.left_wheel_pos = 0.0   # rad
        self.right_wheel_pos = 0.0  # rad

        self.last_time = self.get_clock().now()

        # =============================
        # QoS
        # =============================
        encoder_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =============================
        # Subscribers
        # ESP32 main.cpp publishes:
        # /left_encoder  and /right_encoder
        # each as geometry_msgs/Vector3
        # x = raw rpm, y = filtered rpm, z = pwm
        # =============================
        self.sub_left = self.create_subscription(
            Vector3,
            '/left_encoder',
            self.left_encoder_callback,
            encoder_qos
        )

        self.sub_right = self.create_subscription(
            Vector3,
            '/right_encoder',
            self.right_encoder_callback,
            encoder_qos
        )

        # =============================
        # Publishers
        # =============================
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # =============================
        # TF Broadcaster
        # =============================
        #self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # =============================
        # Timer
        # =============================
        self.timer = self.create_timer(0.05, self.update_odometry)

        self.get_logger().info('Encoder Odometry Node Started')

    def left_encoder_callback(self, msg: Vector3):
        # pakai filtered rpm dari ESP32
        self.left_rpm = float(msg.x)

    def right_encoder_callback(self, msg: Vector3):
        # pakai filtered rpm dari ESP32
        self.right_rpm = float(msg.x)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = now

        # RPM -> m/s
        v_l = self.left_rpm * 2.0 * math.pi * self.wheel_radius / 60.0
        v_r = self.right_rpm * 2.0 * math.pi * self.wheel_radius / 60.0

        # Wheel angular velocity -> rad/s
        w_l = self.left_rpm * 2.0 * math.pi / 60.0
        w_r = self.right_rpm * 2.0 * math.pi / 60.0

        # Integrate wheel joint positions
        self.left_wheel_pos += w_l * dt
        self.right_wheel_pos += w_r * dt

        # Differential drive kinematics
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.wheelbase

        # Integrate pose
        dtheta = omega * dt
        theta_mid = self.theta + (dtheta * 0.5)

        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta = self._normalize_angle(self.theta + dtheta)

        # =============================
        # Publish Odometry
        # =============================
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # Covariance: sesuaikan lagi nanti kalau sudah tuning
        odom.pose.covariance = [
            0.05, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.05, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  999.0
        ]

        odom.twist.covariance = [
            0.1,  0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  999.0, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  999.0, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  999.0, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  999.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.02
        ]

        self.odom_pub.publish(odom)

        # =============================
        # Publish Joint States
        # Nama joint harus sama persis dengan URDF
        # =============================
        joint = JointState()
        joint.header.stamp = now.to_msg()
        joint.name = [
            'left_wheel_joint',
            'right_wheel_joint'
        ]
        joint.position = [
            self.left_wheel_pos,
            self.right_wheel_pos
        ]
        joint.velocity = [
            w_l,
            w_r
        ]
        self.joint_pub.publish(joint)

        # =============================
        # TF: odom -> base_footprint
        # =============================
        #t = TransformStamped()
        #t.header.stamp = now.to_msg()
        #t.header.frame_id = 'odom'
        #t.child_frame_id = 'base_footprint'

        #t.transform.translation.x = self.x
        #t.transform.translation.y = self.y
        #t.transform.translation.z = 0.0

        #t.transform.rotation.x = 0.0
        #t.transform.rotation.y = 0.0
        #t.transform.rotation.z = math.sin(self.theta / 2.0)
        #t.transform.rotation.w = math.cos(self.theta / 2.0)

        #self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()