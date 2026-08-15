import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import tf2_ros


class EncoderOdomDebugNode(Node):

    def __init__(self):
        super().__init__('encoder_odometry_debug_node')

        # =====================================================
        # ROBOT PARAMETERS
        # =====================================================
        self.wheel_radius = 0.0290
        self.wheelbase = 0.19

        # =====================================================
        # DEBUG PARAMETERS
        # =====================================================
        self.encoder_timeout = 0.15   # seconds
        self.rpm_deadband = 1.0       # RPM

        # =====================================================
        # STATE
        # =====================================================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_rpm = 0.0
        self.right_rpm = 0.0

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        now = self.get_clock().now()

        self.last_time = now
        self.last_left_msg_time = now
        self.last_right_msg_time = now

        # =====================================================
        # QoS ENCODER
        # =====================================================
        encoder_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =====================================================
        # SUBSCRIBERS
        # =====================================================
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

        # =====================================================
        # PUBLISHERS
        # =====================================================

        # Jangan gunakan /odom agar tidak bentrok dengan
        # encoder_odometry_node normal.
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom_debug',
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states_debug',
            10
        )

        # =====================================================
        # TF
        #
        # odom_debug -> base_footprint
        #
        # Base footprint tetap memakai nama frame URDF normal
        # agar robot_description bisa tersambung.
        # =====================================================
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # =====================================================
        # TIMER
        # =====================================================
        self.timer = self.create_timer(
            0.02,   # 50 Hz
            self.update_odometry
        )

        self.get_logger().info(
            'Encoder Odometry DEBUG started'
        )

        self.get_logger().info(
            'Publishing: /odom_debug'
        )

        self.get_logger().info(
            'TF: odom_debug -> base_footprint'
        )

    # =========================================================
    # ENCODER CALLBACK
    # =========================================================

    def left_encoder_callback(self, msg: Vector3):
        rpm = float(msg.y)

        if not math.isfinite(rpm):
            rpm = 0.0

        if abs(rpm) < self.rpm_deadband:
            rpm = 0.0

        self.left_rpm = rpm
        self.last_left_msg_time = self.get_clock().now()

    def right_encoder_callback(self, msg: Vector3):
        rpm = float(msg.y)

        if not math.isfinite(rpm):
            rpm = 0.0

        if abs(rpm) < self.rpm_deadband:
            rpm = 0.0

        self.right_rpm = rpm
        self.last_right_msg_time = self.get_clock().now()

    # =========================================================
    # ANGLE NORMALIZATION
    # =========================================================

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    # =========================================================
    # ODOMETRY UPDATE
    # =========================================================

    def update_odometry(self):

        now = self.get_clock().now()

        dt = (
            now - self.last_time
        ).nanoseconds / 1e9

        if dt <= 0.0:
            return

        # Lindungi integrasi apabila sistem sempat pause
        if dt > 0.1:
            self.get_logger().warn(
                f'Large dt detected: {dt:.3f}s, '
                'clamping to 0.02s'
            )
            dt = 0.02

        self.last_time = now

        # =====================================================
        # ENCODER TIMEOUT
        # =====================================================

        left_age = (
            now - self.last_left_msg_time
        ).nanoseconds / 1e9

        right_age = (
            now - self.last_right_msg_time
        ).nanoseconds / 1e9

        left_rpm = self.left_rpm
        right_rpm = self.right_rpm

        # Kalau encoder tidak mengirim data,
        # anggap roda berhenti.
        if left_age > self.encoder_timeout:
            left_rpm = 0.0

        if right_age > self.encoder_timeout:
            right_rpm = 0.0

        # =====================================================
        # RPM -> LINEAR VELOCITY
        # =====================================================

        v_l = (
            left_rpm
            * 2.0
            * math.pi
            * self.wheel_radius
            / 60.0
        )

        v_r = (
            right_rpm
            * 2.0
            * math.pi
            * self.wheel_radius
            / 60.0
        )

        # =====================================================
        # RPM -> WHEEL ANGULAR VELOCITY
        # =====================================================

        w_l = (
            left_rpm
            * 2.0
            * math.pi
            / 60.0
        )

        w_r = (
            right_rpm
            * 2.0
            * math.pi
            / 60.0
        )

        # =====================================================
        # WHEEL POSITION
        # =====================================================

        self.left_wheel_pos += w_l * dt
        self.right_wheel_pos += w_r * dt

        # =====================================================
        # DIFFERENTIAL DRIVE
        # =====================================================

        v = (v_r + v_l) / 2.0

        omega = (
            v_r - v_l
        ) / self.wheelbase

        # =====================================================
        # POSE INTEGRATION
        # =====================================================

        dtheta = omega * dt
        theta_mid = self.theta + 0.5 * dtheta

        self.x += (
            v
            * math.cos(theta_mid)
            * dt
        )

        self.y += (
            v
            * math.sin(theta_mid)
            * dt
        )

        self.theta = self.normalize_angle(
            self.theta + dtheta
        )

        # =====================================================
        # ODOMETRY MESSAGE
        # =====================================================

        odom = Odometry()

        odom.header.stamp = now.to_msg()

        # Debug-only frame
        odom.header.frame_id = 'odom_debug'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0

        odom.pose.pose.orientation.z = math.sin(
            self.theta / 2.0
        )

        odom.pose.pose.orientation.w = math.cos(
            self.theta / 2.0
        )

        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = omega

        # =====================================================
        # COVARIANCE
        # =====================================================

        odom.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.5
        ]

        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 999.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.3
        ]

        self.odom_pub.publish(odom)

        # =====================================================
        # JOINT STATES
        # =====================================================

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

        # =====================================================
        # TF
        #
        # odom_debug -> base_footprint
        # =====================================================

        transform = TransformStamped()

        transform.header.stamp = now.to_msg()
        transform.header.frame_id = 'odom_debug'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = math.sin(
            self.theta / 2.0
        )

        transform.transform.rotation.w = math.cos(
            self.theta / 2.0
        )

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):

    rclpy.init(args=args)

    node = EncoderOdomDebugNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()