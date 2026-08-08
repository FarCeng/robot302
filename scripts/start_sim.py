#!/usr/bin/env python3
import os
import subprocess
import time
import sys
# Modul bawaan ROS 2 untuk mencari lokasi paket secara global
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

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
    # Mengambil lokasi global share directory paket ROS 2
    try:
        nav_share = get_package_share_directory('robot302_navigation')
        desc_share = get_package_share_directory('robot302_description')
    except PackageNotFoundError:
        print("[ ERROR ] Paket 'robot302_navigation' atau 'robot302_description' belum di-source!")
        print("Pastikan sudah menjalankan 'source install/setup.bash' di workspace Anda.")
        sys.exit(1)

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
            scan_check = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', '/scan'], 
                capture_output=True, timeout=2
            )
            encoder_check = subprocess.run(
                ['ros2', 'topic', 'echo', '--once', '/left_encoder'], 
                capture_output=True, timeout=2
            )
            
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

    # 4. Jika sudah tembus pengecekan, tampilkan Menu Map
    print("="*40)
    print("=== TAHAP 3: MENU NAVIGASI ===")
    print("="*40)

    # Mencari peta langsung di dalam folder install paket navigation
    map_dir = os.path.join(nav_share, 'maps')
    if os.path.exists(map_dir):
        maps = [f for f in os.listdir(map_dir) if f.endswith('.yaml')]
    else:
        maps = ["arena_robot302.yaml", "my_map.yaml"] # Fallback jika folder kosong

    for i, m in enumerate(maps):
        print(f"[{i+1}] {m}")
    
    try:
        idx_map = input(f"\nMasukkan nomor map (1-{len(maps)}) [Default: 1]: ")
        selected_map = maps[int(idx_map)-1] if idx_map.isdigit() and 1 <= int(idx_map) <= len(maps) else maps[0]
    except KeyboardInterrupt:
        print("\nPeluncuran dibatalkan.")
        hw_process.terminate()
        return

    # PATH SEKARANG OTOMATIS MENGIKUTI INSTALASI ROS 2
    map_full_path = os.path.join(map_dir, selected_map)

    # 5. Nyalakan Navigasi
    print("\n[INFO] Menjalankan Navigation 2 dan RViz...")
    nav_cmd = [
        "ros2", "launch", "robot302_navigation", "navigation.launch.py",
        f"map:={map_full_path}",
        "use_sim_time:=false"
    ]
    
    rviz_config_path = os.path.join(desc_share, 'rviz', 'nav2_302sim_view.rviz')
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
