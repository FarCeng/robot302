#!/usr/bin/env python3

import math
import statistics
import time

import rclpy
from rclpy.node import Node

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import Imu


TOPIC = "/imu/data_raw"

CALIBRATION_DURATION = 10.0
WAIT_FOR_IMU_TIMEOUT = 5.0
MIN_SAMPLES = 50


class ImuCalibrationNode(Node):

    def __init__(self):
        super().__init__("imu_calibration_node")

        self.samples_x = []
        self.samples_y = []
        self.samples_z = []

        # Waktu script mulai.
        self.program_start_time = time.monotonic()

        # Akan diisi ketika IMU pertama kali diterima.
        self.data_start_time = None

        self.finished = False

        # =====================================================
        # QoS IMU
        # =====================================================

        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.subscription = self.create_subscription(
            Imu,
            TOPIC,
            self.imu_callback,
            imu_qos,
        )

        self.timer = self.create_timer(
            0.1,
            self.check_status,
        )

        self.get_logger().info(
            "IMU calibration started."
        )

        self.get_logger().info(
            "KEEP THE ROBOT COMPLETELY STILL."
        )

        self.get_logger().info(
            f"Waiting for IMU data on {TOPIC}..."
        )

    # =========================================================
    # IMU CALLBACK
    # =========================================================

    def imu_callback(self, msg: Imu):

        if self.finished:
            return

        now = time.monotonic()

        # Message pertama
        if self.data_start_time is None:
            self.data_start_time = now

            self.get_logger().info(
                "IMU data received."
            )

            self.get_logger().info(
                f"Starting {CALIBRATION_DURATION:.1f}s calibration..."
            )

        # Hanya ambil data selama periode kalibrasi.
        if now - self.data_start_time >= CALIBRATION_DURATION:
            return

        gx = float(msg.angular_velocity.x)
        gy = float(msg.angular_velocity.y)
        gz = float(msg.angular_velocity.z)

        if not all(
            math.isfinite(v)
            for v in (gx, gy, gz)
        ):
            return

        self.samples_x.append(gx)
        self.samples_y.append(gy)
        self.samples_z.append(gz)

    # =========================================================
    # STATUS / TIMEOUT
    # =========================================================

    def check_status(self):

        if self.finished:
            return

        now = time.monotonic()

        # -----------------------------------------------------
        # BELUM ADA DATA IMU
        # -----------------------------------------------------

        if self.data_start_time is None:

            elapsed = (
                now - self.program_start_time
            )

            if elapsed >= WAIT_FOR_IMU_TIMEOUT:

                self.finished = True

                print()
                print("=" * 70)
                print("ERROR: NO IMU DATA RECEIVED")
                print("=" * 70)
                print(
                    f"Topic: {TOPIC}"
                )
                print(
                    "The subscriber did not receive "
                    "any IMU message."
                )
                print()
                print(
                    "Check:"
                )
                print(
                    "  ros2 topic echo /imu/data_raw"
                )
                print()
                print(
                    "Also check QoS:"
                )
                print(
                    "  ros2 topic info /imu/data_raw -v"
                )
                print("=" * 70)

                rclpy.shutdown()

            return

        # -----------------------------------------------------
        # SUDAH TERIMA DATA
        # -----------------------------------------------------

        elapsed = (
            now - self.data_start_time
        )

        if elapsed >= CALIBRATION_DURATION:

            self.finished = True

            self.print_results()

            rclpy.shutdown()

    # =========================================================
    # STATISTICS
    # =========================================================

    @staticmethod
    def calculate(values):

        if len(values) < 2:
            raise ValueError(
                "Not enough samples."
            )

        mean = statistics.fmean(values)

        variance = statistics.variance(
            values
        )

        stddev = math.sqrt(
            variance
        )

        return mean, stddev, variance

    # =========================================================
    # FORMAT
    # =========================================================

    @staticmethod
    def fmt(value):

        return f"{value:.10g}"

    # =========================================================
    # RESULT
    # =========================================================

    def print_results(self):

        print()
        print("=" * 70)
        print("             IMU GYRO CALIBRATION RESULT")
        print("=" * 70)

        print(
            f"Topic       : {TOPIC}"
        )

        print(
            f"Duration    : {CALIBRATION_DURATION:.1f} s"
        )

        print(
            f"Samples X   : {len(self.samples_x)}"
        )

        print(
            f"Samples Y   : {len(self.samples_y)}"
        )

        print(
            f"Samples Z   : {len(self.samples_z)}"
        )

        if len(self.samples_z) < MIN_SAMPLES:

            print()
            print("WARNING")
            print("-" * 70)

            print(
                f"Only {len(self.samples_z)} Z samples received."
            )

            print(
                f"Recommended minimum: {MIN_SAMPLES}"
            )

            print(
                "The result may not be reliable."
            )

            print("=" * 70)

            return

        # =====================================================
        # ALL AXES
        # =====================================================

        for axis, values in (
            ("X", self.samples_x),
            ("Y", self.samples_y),
            ("Z", self.samples_z),
        ):

            if len(values) < 2:
                continue

            mean, stddev, variance = (
                self.calculate(values)
            )

            print()
            print(f"Gyro {axis}")
            print("-" * 40)

            print(
                f"Mean / bias   : "
                f"{self.fmt(mean)} rad/s"
            )

            print(
                f"Std deviation : "
                f"{self.fmt(stddev)} rad/s"
            )

            print(
                f"Variance      : "
                f"{self.fmt(variance)} (rad/s)^2"
            )

        # =====================================================
        # Z AXIS
        # =====================================================

        mean_z, std_z, variance_z = (
            self.calculate(
                self.samples_z
            )
        )

        print()
        print("=" * 70)
        print("       RECOMMENDED Z GYRO COVARIANCE")
        print("=" * 70)

        print(
            f"angular_velocity_covariance[8] = "
            f"{self.fmt(variance_z)}"
        )

        print()
        print(
            "Suggested covariance:"
        )

        print("angular_velocity_covariance:")
        print("  - 0.0")
        print("  - 0.0")
        print("  - 0.0")
        print("  - 0.0")
        print("  - 0.0")
        print("  - 0.0")
        print("  - 0.0")
        print("  - 0.0")
        print(
            f"  - {self.fmt(variance_z)}"
        )

        print()
        print(
            "Measured Z bias : "
            f"{self.fmt(mean_z)} rad/s"
        )

        print(
            "Measured Z noise: "
            f"{self.fmt(std_z)} rad/s"
        )

        print(
            "Measured Z variance: "
            f"{self.fmt(variance_z)} (rad/s)^2"
        )

        print()
        print(
            "Do NOT subtract the bias yet."
        )

        print(
            "First verify the bias over multiple runs."
        )

        print("=" * 70)


def main(args=None):

    rclpy.init(args=args)

    node = ImuCalibrationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        print()
        print("Calibration interrupted.")

    finally:

        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()