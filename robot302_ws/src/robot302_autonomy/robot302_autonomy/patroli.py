#!/usr/bin/env python3
import rclpy
import time
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

# Fungsi bantuan yang sudah disesuaikan untuk menerima orientasi Z dan W
def create_pose(navigator, pos_x, pos_y, ori_z, ori_w):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # Menetapkan posisi (X, Y)
    pose.pose.position.x = float(pos_x)
    pose.pose.position.y = float(pos_y)
    pose.pose.position.z = 0.0
    
    # Menetapkan orientasi (Quaternion Z, W)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = float(ori_z)
    pose.pose.orientation.w = float(ori_w)
    
    return pose

def main():
    rclpy.init()
    
    navigator = BasicNavigator()
    
    print("Menunggu sistem Nav2 aktif...")
    navigator.waitUntilNav2Active()

    # Memasukkan urutan koordinat hasil dari RViz
    waypoints = [
        # Titik 1
        create_pose(navigator, 
                    pos_x=1.4357044696807861, pos_y=0.02209070324897766, 
                    ori_z=-0.7071569574575821, ori_w=0.7070566013547539),
        # Titik 2
        create_pose(navigator, 
                    pos_x=1.4732944965362549, pos_y=-0.7633703947067261, 
                    ori_z=-0.7135513693758005, ori_w=0.7006029141117811),
        # Titik 3
        create_pose(navigator, 
                    pos_x=-0.16319847106933594, pos_y=-0.8294948935508728, 
                    ori_z=0.9997872575679483, ori_w=0.020626187353967658),
        # Titik 4
        create_pose(navigator, 
                    pos_x=-0.27052927017211914, pos_y=0.10614210367202759, 
                    ori_z=0.003986024177006744, ori_w=0.9999920557740748)
    ]

    print(f"Memulai eksekusi {len(waypoints)} waypoint...")

    # Looping pergerakan untuk setiap titik
    for i, wp in enumerate(waypoints):
        print(f"\n--- Mengirim robot ke Waypoint {i + 1} ---")
        navigator.goToPose(wp)

        # Looping untuk mengecek status pergerakan robot
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"Jarak tersisa ke Waypoint {i+1}: {feedback.distance_remaining:.2f} meter", end='\r')

        # Cek hasil pencapaian tujuan
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"\n[SUKSES] Berhasil mencapai Waypoint {i + 1}!")
            
            # Beri jeda sejenak sebelum lanjut ke titik berikutnya
            print("Berhenti selama 3 detik sebelum melanjutkan...")
            time.sleep(3.0)
            
        else:
            print(f"\n[GAGAL] Tidak dapat mencapai Waypoint {i + 1}. Menghentikan automasi.")
            break 

    print("\nAlur pergerakan waypoint selesai dieksekusi!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()