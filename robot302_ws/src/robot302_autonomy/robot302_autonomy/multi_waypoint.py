#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# Fungsi bantuan untuk membuat objek PoseStamped dengan mudah
def create_pose(navigator, x, y, w=1.0):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = float(w)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    navigator.waitUntilNav2Active()

    # 1. Buat daftar (list) kosong untuk menampung waypoint
    waypoints = []
    
    # 2. Tambahkan koordinat titik yang ingin dituju secara berurutan
    # Format: create_pose(navigator, X, Y, Orientasi_W)
    waypoints.append(create_pose(navigator, 2.0, 1.5))  # Waypoint 1
    waypoints.append(create_pose(navigator, 4.0, 1.5))  # Waypoint 2
    waypoints.append(create_pose(navigator, 4.0, -1.0)) # Waypoint 3
    waypoints.append(create_pose(navigator, 0.0, 0.0))  # Kembali ke start

    print(f"Memulai automasi: Mengikuti {len(waypoints)} waypoint...")
    
    # 3. Eksekusi semua waypoint sekaligus
    navigator.followWaypoints(waypoints)

    # 4. Looping feedback selama perjalanan
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            # Menampilkan indeks waypoint yang sedang dituju saat ini
            print(f"Sedang menuju waypoint ke-{feedback.current_waypoint}")

    # 5. Cek hasil akhir
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Selesai! Semua waypoint berhasil dilewati.")
    elif result == TaskResult.CANCELED:
        print("Tugas dibatalkan di tengah jalan.")
    elif result == TaskResult.FAILED:
        print("Gagal menyelesaikan rute waypoint.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()