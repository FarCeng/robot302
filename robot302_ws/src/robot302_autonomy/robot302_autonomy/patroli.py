#!/usr/bin/env python3

import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


STOP_TIME = 3.0
WAYPOINT_TIMEOUT = 120.0
MAX_RETRIES = 2
POLL_INTERVAL = 0.1

WAYPOINTS = [
    {"name": "A", "x": 48.143089294433594, "y": 0.3452625572681427},
    {"name": "B", "x": 46.24986267089844, "y": -3.956498146057129},
    {"name": "C", "x": 22.08811378479004, "y": 2.3347811698913574},
    {"name": "D", "x": 0.49029019474983215, "y": -0.07147882133722305},
    {"name": "A", "x": 48.143089294433594, "y": 0.3452625572681427},
]


def create_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Orientasi netral.
    # Yaw tidak dipaksakan oleh patrol.
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    return pose


def navigate_to_waypoint(navigator, waypoint):
    name = waypoint["name"]
    x = waypoint["x"]
    y = waypoint["y"]

    print()
    print("=" * 60)
    print(f"Menuju Waypoint {name}")
    print(f"X : {x:.3f}")
    print(f"Y : {y:.3f}")
    print("=" * 60)

    total_attempts = MAX_RETRIES + 1

    for attempt in range(1, total_attempts + 1):
        print(f"\nPercobaan {attempt}/{total_attempts} - Waypoint {name}")

        # Buat goal baru setiap percobaan
        goal_pose = create_pose(navigator, x, y)

        navigator.goToPose(goal_pose)

        start_time = time.monotonic()

        while not navigator.isTaskComplete():
            elapsed = time.monotonic() - start_time

            if elapsed > WAYPOINT_TIMEOUT:
                print(
                    f"\nTimeout Waypoint {name} "
                    f"({WAYPOINT_TIMEOUT:.0f} detik)"
                )

                navigator.cancelTask()
                time.sleep(1.0)
                break

            feedback = navigator.getFeedback()

            if feedback is not None:
                distance_remaining = feedback.distance_remaining

                print(
                    f"Waypoint {name} | "
                    f"Distance remaining: {distance_remaining:.2f} m | "
                    f"Time: {elapsed:.0f} s",
                    end="\r",
                    flush=True,
                )

            time.sleep(POLL_INTERVAL)

        result = navigator.getResult()
        print()

        if result == TaskResult.SUCCEEDED:
            print(f"Waypoint {name} berhasil dicapai.")
            print(f"Berhenti selama {STOP_TIME:.1f} detik...")
            time.sleep(STOP_TIME)
            return True

        elif result == TaskResult.CANCELED:
            print(f"Waypoint {name} dibatalkan.")

        elif result == TaskResult.FAILED:
            print(f"Waypoint {name} gagal.")

        else:
            print(f"Waypoint {name} status tidak diketahui.")

        if attempt < total_attempts:
            print("Mencoba ulang dalam 2 detik...")
            time.sleep(2.0)

    print(
        f"\nWaypoint {name} gagal setelah "
        f"{total_attempts} percobaan."
    )

    return False


def main():
    rclpy.init()
    navigator = BasicNavigator()

    try:
        print()
        print("=" * 60)
        print("ROBOT PATROLI")
        print("Menunggu Nav2 aktif...")
        print("=" * 60)

        navigator.waitUntilNav2Active()

        print("Nav2 aktif.")
        print(f"Jumlah waypoint: {len(WAYPOINTS)}")

        for index, waypoint in enumerate(WAYPOINTS, start=1):
            print()
            print(
                f"[{index}/{len(WAYPOINTS)}] "
                f"Waypoint {waypoint['name']}"
            )

            success = navigate_to_waypoint(
                navigator,
                waypoint
            )

            if not success:
                print()
                print("=" * 60)
                print(
                    f"PATROLI DIHENTIKAN pada "
                    f"Waypoint {waypoint['name']}"
                )
                print("=" * 60)
                break

        else:
            print()
            print("=" * 60)
            print("PATROLI SELESAI")
            print("=" * 60)

    except KeyboardInterrupt:
        print("\nPatroli dihentikan oleh user.")

        try:
            navigator.cancelTask()
        except Exception:
            pass

    except Exception as e:
        print()
        print(f"ERROR: {e}")

        try:
            navigator.cancelTask()
        except Exception:
            pass

    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()