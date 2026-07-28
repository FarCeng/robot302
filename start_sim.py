#!/usr/bin/env python3
import os
import subprocess

def main():
    print("\n" + "="*40)
    print("=== MENU PELUNCURAN ROBOT302 ===")
    print("="*40)

    # 1. Menu Pilih World Gazebo
    worlds = ["warehouse.world", "empty.world"]
    print("\nPILIH WORLD:")
    for i, w in enumerate(worlds):
        print(f"[{i+1}] {w}")
    
    idx_world = input(f"Masukkan nomor world (1-{len(worlds)}) [Default: 1]: ")
    selected_world = worlds[int(idx_world)-1] if idx_world.isdigit() and 1 <= int(idx_world) <= len(worlds) else worlds[0]

    # 2. Menu Pilih Map Nav2
    maps = ["arena_robot302.yaml", "my_map.yaml"]
    print("\nPILIH MAP:")
    for i, m in enumerate(maps):
        print(f"[{i+1}] {m}")
    
    idx_map = input(f"Masukkan nomor map (1-{len(maps)}) [Default: 1]: ")
    selected_map = maps[int(idx_map)-1] if idx_map.isdigit() and 1 <= int(idx_map) <= len(maps) else maps[0]

    # 3. Susun Full Path
    home_dir = os.path.expanduser("~")
    map_full_path = f"{home_dir}/ros2_ws/src/robot302_navigation/maps/{selected_map}"
    world_full_path = f"{home_dir}/ros2_ws/src/robot302_gazebo/worlds/{selected_world}"

    print("\n" + "="*40)
    print(f"Menjalankan simulasi dengan:")
    print(f"- World : {selected_world}")
    print(f"- Map   : {selected_map}")
    print("="*40 + "\n")

    # 4. Siapkan Command Launch Master
    launch_cmd = [
        "ros2", "launch", "robot302_bringup", "robot302_sim.launch.py",
        f"map:={map_full_path}",
        f"world:={world_full_path}"
    ]

    # 5. Eksekusi
    try:
        # RViz sekarang ditangani sepenuhnya oleh robot302_sim.launch.py
        print("[INFO] Menjalankan Gazebo, Nav2, dan RViz2...\n")
        subprocess.run(launch_cmd)
        
    except KeyboardInterrupt:
        print("\n[INFO] Menutup simulasi...")
    finally:
        print("[INFO] Selesai.")

if __name__ == "__main__":
    main()