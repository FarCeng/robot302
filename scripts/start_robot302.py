#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from typing import Optional

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

MICROROS_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
LIDAR_PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

STARTUP_TIMEOUT = 30
TOPIC_CHECK_INTERVAL = 2


def get_robot_packages():
    try:
        pkg_nav = get_package_share_directory("robot302_navigation")
        pkg_description = get_package_share_directory("robot302_description")
    except PackageNotFoundError as exc:
        raise RuntimeError(
            "Package ROS 2 tidak ditemukan.\n"
            "Pastikan workspace sudah di-source:\n"
            "source ~/robot302/robot302_ws/install/setup.bash"
        ) from exc

    return pkg_nav, pkg_description


def discover_maps(pkg_nav):
    maps_dir = os.path.join(pkg_nav, "maps")

    if not os.path.isdir(maps_dir):
        raise FileNotFoundError(f"Direktori maps tidak ditemukan:\n{maps_dir}")

    maps = sorted(
        f for f in os.listdir(maps_dir)
        if f.lower().endswith(".yaml")
        and os.path.isfile(os.path.join(maps_dir, f))
    )

    if not maps:
        raise FileNotFoundError(f"Tidak ada file .yaml di:\n{maps_dir}")

    return maps_dir, maps


def validate_map(map_yaml):
    if not os.path.isfile(map_yaml):
        raise FileNotFoundError(f"Map YAML tidak ditemukan:\n{map_yaml}")

    yaml_dir = os.path.dirname(map_yaml)
    image_filename = None

    with open(map_yaml, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line.startswith("image:") and not line.startswith("#"):
                image_filename = line.split(":", 1)[1].strip()
                break

    if not image_filename:
        raise RuntimeError(f'Parameter "image:" tidak ditemukan:\n{map_yaml}')

    pgm_path = (
        image_filename
        if os.path.isabs(image_filename)
        else os.path.join(yaml_dir, image_filename)
    )

    if not os.path.isfile(pgm_path):
        raise FileNotFoundError(
            f"File image map tidak ditemukan:\n{pgm_path}"
        )

    return pgm_path


def select_map(pkg_nav):
    maps_dir, maps = discover_maps(pkg_nav)

    print("\n=== PILIH MAP ===")
    for i, map_name in enumerate(maps, 1):
        print(f"[{i}] {map_name}")

    while True:
        choice = input(f"\nPilih map (1-{len(maps)}) [Default: 1]: ").strip()

        if not choice:
            index = 0
            break

        if choice.isdigit() and 1 <= int(choice) <= len(maps):
            index = int(choice) - 1
            break

        print(f"[ERROR] Pilihan harus 1-{len(maps)}.")

    map_yaml = os.path.join(maps_dir, maps[index])
    pgm_path = validate_map(map_yaml)

    print(f"\n[ OK ] YAML: {map_yaml}")
    print(f"[ OK ] PGM : {pgm_path}")

    return map_yaml


def run_quiet(command, timeout=5):
    try:
        return subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def terminate_process_group(
    process: Optional[subprocess.Popen],
    name: str,
    timeout=3,
):
    if process is None or process.poll() is not None:
        return

    try:
        pgid = os.getpgid(process.pid)
        print(f"[INFO] Menghentikan {name}...")
        os.killpg(pgid, signal.SIGINT)
    except (ProcessLookupError, PermissionError):
        return

    deadline = time.time() + timeout
    while process.poll() is None and time.time() < deadline:
        time.sleep(0.1)

    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            return

        deadline = time.time() + 2
        while process.poll() is None and time.time() < deadline:
            time.sleep(0.1)

    if process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def cleanup_stale_robot_processes():
    patterns = [
        "robot302_bringup",
        "robot302_navigation",
        "micro_ros_agent",
        "nav2_container",
        "component_container",
        "odom_node",
        "encoder_odometry_node",
        "rplidar_node",
        "robot_state_publisher",
        "rviz2",
    ]

    print("\n=== CLEANUP PROSES LAMA ===")

    for pattern in patterns:
        result = run_quiet(["pgrep", "-af", pattern])

        if result is None or not result.stdout.strip():
            continue

        print(f"[INFO] Membersihkan: {pattern}")
        subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
        time.sleep(0.5)

        result = run_quiet(["pgrep", "-af", pattern])
        if result and result.stdout.strip():
            subprocess.run(["pkill", "-KILL", "-f", pattern], check=False)

    time.sleep(1)


def check_usb_permission(microros_port, lidar_port):
    print("\n=== CEK USB ===")

    while True:
        micro_ok = (
            os.path.exists(microros_port)
            and os.access(microros_port, os.R_OK | os.W_OK)
        )
        lidar_ok = (
            os.path.exists(lidar_port)
            and os.access(lidar_port, os.R_OK | os.W_OK)
        )

        if micro_ok and lidar_ok:
            print("[ OK ] Micro-ROS dan LiDAR siap.")
            return

        print("[GAGAL] USB belum siap:")
        print(f"  Micro-ROS: {'OK' if micro_ok else 'TIDAK'}")
        print(f"  LiDAR    : {'OK' if lidar_ok else 'TIDAK'}")
        time.sleep(3)


def topic_has_publisher(topic):
    result = run_quiet(["ros2", "topic", "info", topic])

    if result is None or result.returncode != 0:
        return False

    for line in result.stdout.splitlines():
        if line.strip().lower().startswith("publisher count:"):
            try:
                return int(line.split(":", 1)[1].strip()) > 0
            except (ValueError, IndexError):
                return False

    return False


def topic_has_data(topic, timeout=3):
    result = run_quiet(
        ["ros2", "topic", "echo", "--once", topic],
        timeout=timeout,
    )

    return bool(result and result.stdout.strip())


def start_process(command, name):
    print(f"[INFO] Starting {name}...")
    return subprocess.Popen(command, start_new_session=True)


def main():
    hw_process = None
    nav_process = None
    rviz_process = None

    pkg_nav, pkg_description = get_robot_packages()

    print("\n=== ROBOT302 STARTUP ===")
    print(f"[INFO] Navigation : {pkg_nav}")
    print(f"[INFO] Description: {pkg_description}")

    cleanup_stale_robot_processes()

    run_quiet(["ros2", "daemon", "stop"])
    run_quiet(["ros2", "daemon", "start"])
    time.sleep(1)

    try:
        # 1. USB
        check_usb_permission(MICROROS_PORT, LIDAR_PORT)

        # 2. Hardware
        print("\n=== SINKRONISASI HARDWARE ===")

        hw_cmd = [
            "ros2", "launch",
            "robot302_bringup",
            "robot302.launch.py",
            "launch_nav:=false",
            "use_rviz:=false",
        ]

        hw_process = start_process(hw_cmd, "Robot302 Bringup")
        time.sleep(3)

        print("[INFO] Menunggu publisher sensor...")

        start_time = time.time()
        topics_ready = False

        while not topics_ready:
            if hw_process.poll() is not None:
                raise RuntimeError(
                    f"robot302_bringup berhenti "
                    f"(return code {hw_process.returncode})."
                )

            left_ok = topic_has_publisher("/left_encoder")
            right_ok = topic_has_publisher("/right_encoder")
            scan_ok = topic_has_publisher("/scan")

            elapsed = int(time.time() - start_time)

            print(
                f"  left={'OK' if left_ok else '...'} | "
                f"right={'OK' if right_ok else '...'} | "
                f"scan={'OK' if scan_ok else '...'} "
                f"[{elapsed}s]",
                flush=True,
            )

            if left_ok and right_ok and scan_ok:
                topics_ready = True
                break

            if elapsed >= STARTUP_TIMEOUT:
                print("[WARNING] Timeout sensor. Startup dilanjutkan.")
                break

            time.sleep(TOPIC_CHECK_INTERVAL)

        if topics_ready:
            print("[ OK ] Publisher sensor terdeteksi.")
            print(
                f"[INFO] /left_encoder : "
                f"{'OK' if topic_has_data('/left_encoder') else 'BELUM'}"
            )
            print(
                f"[INFO] /right_encoder: "
                f"{'OK' if topic_has_data('/right_encoder') else 'BELUM'}"
            )
            print(
                f"[INFO] /scan         : "
                f"{'OK' if topic_has_data('/scan') else 'BELUM'}"
            )

        # 3. Map
        map_full_path = select_map(pkg_nav)

        # 4. Nav2
        print("\n=== START NAV2 ===")

        nav_cmd = [
            "ros2", "launch",
            "robot302_navigation",
            "navigation.launch.py",
            f"map:={map_full_path}",
            "use_sim_time:=false",
        ]

        print("[INFO] Map:", map_full_path)
        nav_process = start_process(nav_cmd, "Nav2")

        # 5. RViz
        rviz_config = os.path.join(
            pkg_description,
            "rviz",
            "nav2_302sim_view.rviz",
        )

        if not os.path.isfile(rviz_config):
            raise FileNotFoundError(
                f"RViz config tidak ditemukan:\n{rviz_config}"
            )

        rviz_cmd = [
            "ros2", "run", "rviz2", "rviz2",
            "-d", rviz_config,
            "--ros-args",
            "-p", "use_sim_time:=false",
        ]

        rviz_process = start_process(rviz_cmd, "RViz")

        print("\n[STATUS] Robot302 aktif.")
        print("[STATUS] Ctrl+C untuk menghentikan.")

        while True:
            if hw_process.poll() is not None:
                print("[WARNING] Bringup berhenti.")
                break

            if nav_process.poll() is not None:
                print("[WARNING] Nav2 berhenti.")
                break

            if rviz_process.poll() is not None:
                print("[WARNING] RViz berhenti.")
                break

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C diterima.")

    except Exception as exc:
        print(f"\n[ERROR] {exc}")

    finally:
        print("\n=== CLEANUP ===")

        terminate_process_group(rviz_process, "RViz")
        terminate_process_group(nav_process, "Nav2")
        terminate_process_group(hw_process, "Robot302 Bringup")

        cleanup_stale_robot_processes()

        print("[OK] Sistem dihentikan.")


if __name__ == "__main__":
    main()