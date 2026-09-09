#!/usr/bin/env python3
import math
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

STOP_TIME = 3.0
WAYPOINT_TIMEOUT = 300.0
MAX_RETRIES = 2
POLL_INTERVAL = 0.1

def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

def create_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose

def calculate_yaw(current_point, next_point):
    dx = next_point[0] - current_point[0]
    dy = next_point[1] - current_point[1]
    return math.atan2(dy, dx)

def main():
    rclpy.init()
    navigator = BasicNavigator()

    try:
        print("\n==============================================")
        print("        WAYPOINT NAVIGATION SYSTEM")
        print("==============================================")
        print("\n[INFO] Menunggu Nav2 aktif...")
        navigator.waitUntilNav2Active()
        print("[OK] Nav2 aktif.")
        print("[OK] Sistem siap menerima waypoint.\n")

        waypoint_data = [
            {"name": "Start (A)", "x": 48.143089294433594, "y": 0.3452625572681427},
            {"name": "B", "x": 46.24986267089844, "y": -3.956498146057129},
            {"name": "C", "x": 22.08811378479004, "y": 2.3347811698913574},
            {"name": "D", "x": 0.49029019474983215, "y": -0.07147882133722305},
            {"name": "Finish (A)", "x": 48.143089294433594, "y": 0.3452625572681427},
        ]

        waypoints = []

        print("[INFO] Menghitung orientasi waypoint...\n")

        for i, wp in enumerate(waypoint_data):
            if i < len(waypoint_data) - 1:
                next_wp = waypoint_data[i + 1]
                yaw = calculate_yaw((wp["x"], wp["y"]), (next_wp["x"], next_wp["y"]))
            else:
                prev_wp = waypoint_data[i - 1]
                yaw = calculate_yaw((prev_wp["x"], prev_wp["y"]), (wp["x"], wp["y"]))

            waypoints.append(create_pose(navigator, wp["x"], wp["y"], yaw))

            print(
                f'{wp["name"]:12s} | '
                f'x = {wp["x"]:9.4f} | '
                f'y = {wp["y"]:9.4f} | '
                f'yaw = {yaw:7.3f} rad '
                f'({math.degrees(yaw):7.2f} deg)'
            )

        print(f"\n[INFO] Total waypoint: {len(waypoints)}")
        print("[INFO] Urutan: " + " -> ".join(wp["name"] for wp in waypoint_data))
        print("\n==============================================")
        print("         MEMULAI NAVIGASI WAYPOINT")
        print("==============================================\n")

        all_success = True

        for i, (wp_data, wp_pose) in enumerate(zip(waypoint_data, waypoints)):
            waypoint_number = i + 1
            waypoint_name = wp_data["name"]

            print(f"\n----------------------------------------------")
            print(f"[WAYPOINT {waypoint_number}/{len(waypoints)}] {waypoint_name}")
            print(f"Target: x={wp_data['x']:.4f}, y={wp_data['y']:.4f}")
            print(f"----------------------------------------------")

            waypoint_success = False

            for attempt in range(1, MAX_RETRIES + 2):
                print(f"\n[INFO] Percobaan {attempt}/{MAX_RETRIES + 1}")

                wp_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_accepted = navigator.goToPose(wp_pose)

                if not goal_accepted:
                    print("[ERROR] Goal ditolak oleh Nav2.")
                    if attempt <= MAX_RETRIES:
                        print("[INFO] Mencoba ulang...")
                        time.sleep(1.0)
                        continue
                    break

                start_time = time.monotonic()

                while not navigator.isTaskComplete():
                    feedback = navigator.getFeedback()
                    elapsed_time = time.monotonic() - start_time

                    if feedback is not None:
                        print(
                            f"Jarak tersisa: {feedback.distance_remaining:.2f} m | "
                            f"Waktu: {elapsed_time:.1f} s",
                            end="\r",
                            flush=True
                        )

                    if elapsed_time > WAYPOINT_TIMEOUT:
                        print("\n[WARNING] Timeout waypoint.")
                        navigator.cancelTask()
                        cancel_start = time.monotonic()

                        while (
                            not navigator.isTaskComplete()
                            and time.monotonic() - cancel_start < 5.0
                        ):
                            time.sleep(POLL_INTERVAL)
                        break

                    time.sleep(POLL_INTERVAL)

                result = navigator.getResult()
                print()

                if result == TaskResult.SUCCEEDED:
                    print(f"[SUCCESS] {waypoint_name} berhasil dicapai.")
                    waypoint_success = True
                    break

                elif result == TaskResult.CANCELED:
                    print(f"[CANCELED] Navigasi {waypoint_name} dibatalkan.")

                elif result == TaskResult.FAILED:
                    print(f"[FAILED] Navigasi {waypoint_name} gagal.")

                else:
                    print(f"[UNKNOWN] Status {waypoint_name} tidak diketahui.")

                if attempt <= MAX_RETRIES:
                    print(f"[INFO] Retry waypoint {waypoint_name}...")
                    time.sleep(2.0)
                else:
                    print(
                        f"[ERROR] {waypoint_name} gagal setelah "
                        f"{MAX_RETRIES + 1} percobaan."
                    )

            if not waypoint_success:
                print("\n==============================================")
                print(f"[MISSION FAILED] Robot gagal mencapai {waypoint_name}.")
                print("Autonomous navigation dihentikan.")
                print("==============================================")
                all_success = False
                break

            print(f"[INFO] Robot berhenti {STOP_TIME:.1f} detik...")
            time.sleep(STOP_TIME)

        print("\n==============================================")

        if all_success:
            print("[MISSION COMPLETE]")
            print("Robot berhasil menyelesaikan rute A -> B -> C -> D -> A.")
        else:
            print("[MISSION INCOMPLETE]")
            print("Rute tidak berhasil diselesaikan sepenuhnya.")

        print("==============================================")

    except KeyboardInterrupt:
        print("\n[INFO] Program dihentikan oleh pengguna.")
        navigator.cancelTask()

    except Exception as e:
        print(f"\n[ERROR] Terjadi exception: {e}")
        try:
            navigator.cancelTask()
        except Exception:
            pass

    finally:
        print("[INFO] Shutdown ROS 2...")
        navigator.destroyNode()
        rclpy.shutdown()

if __name__ == '__main__':
    main()