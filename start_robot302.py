#!/usr/bin/env python3
import os
import subprocess
import time

def check_usb_permission(microros_port, lidar_port):
    print("="*40)
    print("=== TAHAP 1: CEK FISIK KABEL USB ===")
    print("="*40)
    
    while True:
        micro_ok = os.access(microros_port, os.R_OK | os.W_OK)
        lidar_ok = os.access(lidar_port, os.R_OK | os.W_OK)
        
        if micro_ok and lidar_ok:
            print("[ OK ] Kabel terhubung dan izin akses diberikan.")
            break
            
        print(f"[GAGAL] Akses USB ditolak atau kabel belum terhubung.")
        print(f"Harap cek {microros_port} dan {lidar_port}. Coba lagi dalam 3 detik...")
        time.sleep(3)

def main():
    # 1. Pastikan kabel fisik dicolok
    check_usb_permission('/dev/ttyUSB0', '/dev/ttyUSB1')

    # 2. Nyalakan Hardware di latar belakang
    print("\n" + "="*40)
    print("=== TAHAP 2: SINKRONISASI ROS 2 TOPIC ===")
    print("="*40)
    print("[INFO] Menyalakan Micro-ROS Agent dan LiDAR...")
    
    hw_cmd = [
        "ros2", "launch", "robot302_bringup", "robot302.launch.py",
        "launch_nav:=false", 
        "use_rviz:=false"
    ]
    
    hw_process = subprocess.Popen(hw_cmd)

    # 3. Looping untuk mengecek ALIRAN DATA (Pengecekan Tunggal)
    time.sleep(3) 
    print("\n[INFO] Menunggu aliran data sensor... (Tekan tombol RESET/EN di mikrokontroler SEKARANG!)")
    
    topics_found = False
    while not topics_found:
        try:
            # UJI LiDAR
            scan_check = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', '/scan'], 
                capture_output=True, timeout=2
            )
            
            # UJI MICRO-ROS
            encoder_check = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', '/left_encoder'], 
                capture_output=True, timeout=2
            )
            
            # Jika keduanya berhasil
            if scan_check.returncode == 0 and encoder_check.returncode == 0:
                print("\n[ OK ] Micro-ROS AKTIF! (Data /left_encoder diterima)")
                print("[ OK ] LiDAR AKTIF! (Data /scan diterima)")
                print("Sinkronisasi 100% Berhasil!\n")
                topics_found = True
                time.sleep(1)
            else:
                print("  -> Menunggu aliran data... (Tekan RESET di mikrokontroler jika Micro-ROS stuck)", flush=True)
                
        except subprocess.TimeoutExpired:
            print("  -> Menunggu aliran data... (Tekan RESET di mikrokontroler jika Micro-ROS stuck)", flush=True)
            
    # ---> BLOK KODE ros2 topic list YANG LAMA SUDAH DIHAPUS DARI SINI <---

    # 4. Jika sudah tembus pengecekan, tampilkan Menu Map
    print("="*40)
    print("=== TAHAP 3: MENU NAVIGASI ===")
    print("="*40)

    maps = ["arena_robot302.yaml", "my_map.yaml"]
    for i, m in enumerate(maps):
        print(f"[{i+1}] {m}")
    
    try:
        idx_map = input(f"\nMasukkan nomor map (1-{len(maps)}) [Default: 1]: ")
        selected_map = maps[int(idx_map)-1] if idx_map.isdigit() and 1 <= int(idx_map) <= len(maps) else maps[0]
    except KeyboardInterrupt:
        print("\nPeluncuran dibatalkan.")
        hw_process.terminate()
        return

    home_dir = os.path.expanduser("~")
    map_full_path = f"{home_dir}/ros2_ws/src/robot302_navigation/maps/{selected_map}"

    # 5. Nyalakan Navigasi
    print("\n[INFO] Menjalankan Navigation 2 dan RViz...")
    nav_cmd = [
        "ros2", "launch", "robot302_navigation", "navigation.launch.py",
        f"map:={map_full_path}",
        "use_sim_time:=false"
    ]
    
    rviz_config_path = f"{home_dir}/ros2_ws/src/robot302_description/rviz/nav2_302sim_view.rviz"
    rviz_cmd = ["ros2", "run", "rviz2", "rviz2", "-d", rviz_config_path, "--ros-args", "-p", "use_sim_time:=false"]

    nav_process = subprocess.Popen(nav_cmd, stdout=subprocess.DEVNULL)
    rviz_process = subprocess.Popen(rviz_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    try:
        print("\n[ STATUS ] Sistem Robot Aktif! Tekan Ctrl+C untuk mematikan seluruh sistem.")
        hw_process.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Menutup sistem robot secara aman...")
    finally:
        hw_process.terminate()
        nav_process.terminate()
        rviz_process.terminate()
        print("[INFO] Selesai.")

if __name__ == "__main__":
    main()