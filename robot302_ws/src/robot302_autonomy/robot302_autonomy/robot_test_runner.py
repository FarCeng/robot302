#!/usr/bin/env python3

import math
import csv
import argparse
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class RobotTest(Node):

    def __init__(self, command, target, speed):

        super().__init__("robot_test")

        # ====================================================
        # TEST COMMAND
        # ====================================================

        self.command = command
        self.target = target
        self.speed = abs(speed)

        self.finished = False
        self.started = False

        # ====================================================
        # ROS PUBLISHER
        # ====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        # ====================================================
        # ODOM SUBSCRIBER
        # ====================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # ====================================================
        # IMU SUBSCRIBER
        #
        # IMU menggunakan sensor-data QoS.
        # Ini menghindari warning:
        #
        # "offering incompatible QoS"
        # ====================================================

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu/data_raw",
            self.imu_callback,
            qos_profile_sensor_data
        )

        # ====================================================
        # CONTROL LOOP
        # ====================================================

        self.timer = self.create_timer(
            0.05,       # 20 Hz
            self.update
        )

        # ====================================================
        # ODOM DATA
        # ====================================================

        self.odom_ready = False

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # ====================================================
        # IMU DATA
        # ====================================================

        self.imu_ready = False

        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_az = 0.0

        # ====================================================
        # START POSITION
        # ====================================================

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        # ====================================================
        # TIME
        # ====================================================

        self.test_start_time = (
            self.get_clock().now()
        )

        self.previous_time = None
        self.previous_vx = 0.0

        # ====================================================
        # CSV
        # ====================================================

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        self.csv_filename = (
            f"robot_test_{timestamp}.csv"
        )

        self.file = open(
            self.csv_filename,
            "w",
            newline=""
        )

        self.csv = csv.writer(
            self.file
        )

        self.csv.writerow([
            "time_s",

            "x_m",
            "y_m",
            "yaw_deg",

            "vx_mps",
            "vy_mps",
            "wz_radps",

            "acceleration_odom_mps2",

            "imu_ax_mps2",
            "imu_ay_mps2",
            "imu_az_mps2"
        ])

        # ====================================================
        # START INFO
        # ====================================================

        self.get_logger().info(
            "======================================"
        )

        self.get_logger().info(
            "ROBOT302 MOTION TEST"
        )

        self.get_logger().info(
            f"Command : {self.command}"
        )

        self.get_logger().info(
            f"Target  : {self.target}"
        )

        self.get_logger().info(
            f"Speed   : {self.speed}"
        )

        self.get_logger().info(
            f"CSV     : {self.csv_filename}"
        )

        self.get_logger().info(
            "Waiting for /odom ..."
        )


    # ========================================================
    # ODOM CALLBACK
    # ========================================================

    def odom_callback(self, msg):

        self.x = (
            msg.pose.pose.position.x
        )

        self.y = (
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        self.yaw = self.quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        self.vx = (
            msg.twist.twist.linear.x
        )

        self.vy = (
            msg.twist.twist.linear.y
        )

        self.wz = (
            msg.twist.twist.angular.z
        )

        self.odom_ready = True


    # ========================================================
    # IMU CALLBACK
    # ========================================================

    def imu_callback(self, msg):

        self.imu_ax = (
            msg.linear_acceleration.x
        )

        self.imu_ay = (
            msg.linear_acceleration.y
        )

        self.imu_az = (
            msg.linear_acceleration.z
        )

        self.imu_ready = True


    # ========================================================
    # QUATERNION -> YAW
    # ========================================================

    def quaternion_to_yaw(
        self,
        x,
        y,
        z,
        w
    ):

        siny_cosp = (
            2.0 *
            (w * z + x * y)
        )

        cosy_cosp = (
            1.0 -
            2.0 *
            (y * y + z * z)
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )


    # ========================================================
    # ANGLE NORMALIZATION
    # ========================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle


    # ========================================================
    # DISTANCE FROM START
    # ========================================================

    def get_distance(self):

        if self.start_x is None:
            return 0.0

        dx = (
            self.x -
            self.start_x
        )

        dy = (
            self.y -
            self.start_y
        )

        return math.sqrt(
            dx * dx +
            dy * dy
        )


    # ========================================================
    # ANGLE FROM START
    # ========================================================

    def get_angle(self):

        if self.start_yaw is None:
            return 0.0

        return self.normalize_angle(
            self.yaw -
            self.start_yaw
        )


    # ========================================================
    # SEND CMD_VEL
    # ========================================================

    def send_command(self):

        cmd = Twist()

        # ----------------------------------------------------
        # FORWARD
        # ----------------------------------------------------

        if self.command == "F":

            cmd.linear.x = self.speed
            cmd.angular.z = 0.0

        # ----------------------------------------------------
        # BACKWARD
        # ----------------------------------------------------

        elif self.command == "B":

            cmd.linear.x = -self.speed
            cmd.angular.z = 0.0

        # ----------------------------------------------------
        # TURN
        # ----------------------------------------------------

        elif self.command == "T":

            cmd.linear.x = 0.0

            if self.target >= 0.0:

                cmd.angular.z = self.speed

            else:

                cmd.angular.z = -self.speed

        # ----------------------------------------------------
        # STOP
        # ----------------------------------------------------

        elif self.command == "S":

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


    # ========================================================
    # SEND STOP
    # ========================================================

    def send_stop(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        # Kirim beberapa kali agar stop pasti diterima
        for _ in range(3):

            self.cmd_pub.publish(cmd)


    # ========================================================
    # CHECK TARGET
    # ========================================================

    def target_reached(self):

        # ----------------------------------------------------
        # FORWARD
        # ----------------------------------------------------

        if self.command == "F":

            return (
                self.get_distance()
                >= abs(self.target)
            )

        # ----------------------------------------------------
        # BACKWARD
        # ----------------------------------------------------

        if self.command == "B":

            return (
                self.get_distance()
                >= abs(self.target)
            )

        # ----------------------------------------------------
        # TURN
        # ----------------------------------------------------

        if self.command == "T":

            current_angle = abs(
                self.get_angle()
            )

            target_angle = math.radians(
                abs(self.target)
            )

            return (
                current_angle
                >= target_angle
            )

        # ----------------------------------------------------
        # STOP
        # ----------------------------------------------------

        if self.command == "S":

            elapsed = (
                self.get_clock().now()
                -
                self.step_start_time
            ).nanoseconds / 1e9

            return (
                elapsed >=
                abs(self.target)
            )

        return True


    # ========================================================
    # LOG DATA
    # ========================================================

    def log_data(self):

        now = self.get_clock().now()

        time_s = (
            now -
            self.test_start_time
        ).nanoseconds / 1e9

        # ----------------------------------------------------
        # ACCELERATION FROM ODOM
        # ----------------------------------------------------

        acceleration = 0.0

        if self.previous_time is not None:

            dt = (
                now -
                self.previous_time
            ).nanoseconds / 1e9

            if dt > 0.0:

                acceleration = (
                    self.vx -
                    self.previous_vx
                ) / dt

        self.previous_time = now

        self.previous_vx = self.vx

        # ----------------------------------------------------
        # CSV
        # ----------------------------------------------------

        self.csv.writerow([
            time_s,

            self.x,
            self.y,
            math.degrees(
                self.yaw
            ),

            self.vx,
            self.vy,
            self.wz,

            acceleration,

            self.imu_ax,
            self.imu_ay,
            self.imu_az
        ])

        self.file.flush()


    # ========================================================
    # MAIN LOOP
    # ========================================================

    def update(self):

        # ----------------------------------------------------
        # WAIT FOR ODOM
        # ----------------------------------------------------

        if not self.odom_ready:

            return

        # ----------------------------------------------------
        # START TEST
        # ----------------------------------------------------

        if not self.started:

            self.start_x = self.x
            self.start_y = self.y
            self.start_yaw = self.yaw

            self.step_start_time = (
                self.get_clock().now()
            )

            self.started = True

            self.get_logger().info(
                "Starting test..."
            )

        # ----------------------------------------------------
        # TEST ALREADY FINISHED
        # ----------------------------------------------------

        if self.finished:

            return

        # ----------------------------------------------------
        # SEND COMMAND
        # ----------------------------------------------------

        self.send_command()

        # ----------------------------------------------------
        # LOG
        # ----------------------------------------------------

        self.log_data()

        # ----------------------------------------------------
        # CHECK TARGET
        # ----------------------------------------------------

        if self.target_reached():

            self.send_stop()

            self.finished = True

            self.get_logger().info(
                "======================================"
            )

            self.get_logger().info(
                "TEST FINISHED"
            )

            self.get_logger().info(
                f"Distance : "
                f"{self.get_distance():.4f} m"
            )

            self.get_logger().info(
                f"Yaw      : "
                f"{math.degrees(self.get_angle()):.2f} deg"
            )

            self.get_logger().info(
                f"CSV      : "
                f"{self.csv_filename}"
            )

            self.get_logger().info(
                "======================================"
            )

            self.file.close()

            # Stop timer
            self.timer.cancel()

            # Shutdown ROS
            rclpy.shutdown()


    # ========================================================
    # CLEANUP
    # ========================================================

    def cleanup(self):

        self.send_stop()

        if not self.file.closed:

            self.file.close()


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Robot302 simple motion test"
        )
    )

    parser.add_argument(
        "--command",
        required=True,
        choices=[
            "F",
            "B",
            "T",
            "S"
        ],
        help=(
            "F=maju, "
            "B=mundur, "
            "T=putar, "
            "S=stop"
        )
    )

    parser.add_argument(
        "--target",
        required=True,
        type=float,
        help=(
            "F/B = meter, "
            "T = degree, "
            "S = second"
        )
    )

    parser.add_argument(
        "--speed",
        required=True,
        type=float,
        help=(
            "F/B = m/s, "
            "T = rad/s, "
            "S = 0"
        )
    )

    args = parser.parse_args()

    # ========================================================
    # ROS INIT
    # ========================================================

    rclpy.init()

    node = RobotTest(
        args.command,
        args.target,
        args.speed
    )

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print(
            "\nTest interrupted."
        )

        node.cleanup()

    finally:

        if rclpy.ok():

            node.cleanup()

            node.destroy_node()

            rclpy.shutdown()


if __name__ == "__main__":

    main()