#!/usr/bin/env python3
import os
import signal
import subprocess
import time
from typing import Optional


# =========================================================
# CONFIG
# =========================================================
MICROROS_PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
LIDAR_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'

MAPS = [
    'arena_robot302.yaml',
    'my_map.yaml',
]

STARTUP_TIMEOUT = 30
TOPIC_CHECK_INTERVAL = 2


# =========================================================
# PROCESS MANAGEMENT
# =========================================================
def terminate_process_group(process: Optional[subprocess.Popen], name: str, timeout: float = 3.0):
    """Terminate an entire process group, not only the parent process."""
    if process is None or process.poll() is not None:
        return

    try:
        pgid = os.getpgid(process.pid)
        print(f'[INFO] Menghentikan process group {name} (PGID {pgid})...')
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    except PermissionError:
        print(f'[WARNING] Tidak punya izin mengirim SIGINT ke {name}.')
        return

    deadline = time.time() + timeout
    while process.poll() is None and time.time() < deadline:
        time.sleep(0.1)

    if process.poll() is None:
        try:
            print(f'[WARNING] {name} belum berhenti. Mengirim SIGTERM...')
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        except ProcessLookupError:
            return

        deadline = time.time() + 2.0
        while process.poll() is None and time.time() < deadline:
            time.sleep(0.1)

    if process.poll() is None:
        try:
            print(f'[WARNING] {name} masih hidup. Mengirim SIGKILL...')
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass


def run_quiet(command, timeout=5):
    """Run a command quietly and return CompletedProcess or None on timeout/error."""
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


def cleanup_stale_robot_processes():
    """
    Cleanup only processes associated with this robot/Nav2 stack.
    Do NOT kill the generic ROS 2 daemon.
    """
    print('\n' + '=' * 40)
    print('=== CLEANUP PROSES LAMA ROBOT302 ===')
    print('=' * 40)

    # Specific patterns. Avoid `pkill -f ros2` because that is too broad.
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

    found_any = False

    for pattern in patterns:
        # First check if anything matches.
        result = run_quiet(['pgrep', '-af', pattern])
        if result is None:
            continue

        lines = [line for line in result.stdout.splitlines() if line.strip()]
        if not lines:
            continue

        found_any = True
        print(f'[INFO] Menemukan proses: {pattern}')
        for line in lines:
            print(f'       {line}')

        # Kill matching processes. -f matches full command line.
        subprocess.run(['pkill', '-TERM', '-f', pattern], check=False)
        time.sleep(0.5)

        # Escalate only if necessary.
        result_after = run_quiet(['pgrep', '-af', pattern])
        if result_after and result_after.stdout.strip():
            print(f'[WARNING] {pattern} masih hidup, mengirim KILL...')
            subprocess.run(['pkill', '-KILL', '-f', pattern], check=False)

    if not found_any:
        print('[OK] Tidak ada proses robot/Nav2 lama yang perlu dibersihkan.')
    else:
        time.sleep(1)
        print('[OK] Cleanup selesai.')


# =========================================================
# USB / ROS TOPIC CHECK
# =========================================================
def check_usb_permission(microros_port, lidar_port):
    print('=' * 40)
    print('=== TAHAP 1: CEK FISIK KABEL USB ===')
    print('=' * 40)

    while True:
        micro_ok = os.path.exists(microros_port) and os.access(microros_port, os.R_OK | os.W_OK)
        lidar_ok = os.path.exists(lidar_port) and os.access(lidar_port, os.R_OK | os.W_OK)

        if micro_ok and lidar_ok:
            print('[ OK ] Kabel terhubung dan izin akses diberikan.')
            return

        print('[GAGAL] Akses USB ditolak atau perangkat belum terhubung.')
        print(f'        Micro-ROS: {microros_port} -> {"OK" if micro_ok else "TIDAK"}')
        print(f'        LiDAR:     {lidar_port} -> {"OK" if lidar_ok else "TIDAK"}')
        print('        Coba lagi dalam 3 detik...')
        time.sleep(3)


def topic_has_publisher(topic_name):
    """Return True when a ROS 2 topic has at least one publisher."""
    result = run_quiet(['ros2', 'topic', 'info', topic_name], timeout=5)
    if result is None or result.returncode != 0:
        return False

    for line in result.stdout.splitlines():
        if line.strip().lower().startswith('publisher count:'):
            try:
                return int(line.split(':', 1)[1].strip()) > 0
            except (ValueError, IndexError):
                return False

    return False


def topic_has_data(topic_name, timeout=3):
    """Best-effort check. A timeout does not fail the whole startup."""
    result = run_quiet(
        ['ros2', 'topic', 'echo', '--once', topic_name],
        timeout=timeout,
    )
    if result is None or result.returncode != 0:
        return False
    return bool(result.stdout.strip())


# =========================================================
# LAUNCH HELPERS
# =========================================================
def start_process(command, name):
    """
    Start a command in its own process group so Ctrl+C/cleanup can stop
    all descendants spawned by ros2 launch.
    """
    print(f'[INFO] Starting {name}...')
    return subprocess.Popen(
        command,
        start_new_session=True,
    )


# =========================================================
# MAIN
# =========================================================
def main():
    hw_process = None
    nav_process = None
    rviz_process = None

    # -----------------------------------------------------
    # 0. CLEANUP STALE PROCESSES
    # -----------------------------------------------------
    cleanup_stale_robot_processes()

    # Optional: refresh ROS graph daemon state.
    # Do NOT kill the daemon process manually; just refresh its cache.
    run_quiet(['ros2', 'daemon', 'stop'], timeout=5)
    run_quiet(['ros2', 'daemon', 'start'], timeout=5)
    time.sleep(1)

    try:
        # -------------------------------------------------
        # 1. USB
        # -------------------------------------------------
        check_usb_permission(MICROROS_PORT, LIDAR_PORT)

        # -------------------------------------------------
        # 2. HARDWARE BRINGUP
        # -------------------------------------------------
        print('\n' + '=' * 40)
        print('=== TAHAP 2: SINKRONISASI ROS 2 TOPIC ===')
        print('=' * 40)
        print('[INFO] Menyalakan Micro-ROS Agent dan LiDAR...')

        hw_cmd = [
            'ros2', 'launch', 'robot302_bringup', 'robot302.launch.py',
            'launch_nav:=false',
            'use_rviz:=false',
        ]

        hw_process = start_process(hw_cmd, 'Robot302 Bringup')

        time.sleep(3)
        print('\n[INFO] Menunggu publisher sensor...')
        print('[INFO] Tidak perlu menekan RESET kecuali Micro-ROS benar-benar stuck.')

        wait_started = time.time()
        topics_ready = False

        while not topics_ready:
            if hw_process.poll() is not None:
                raise RuntimeError(
                    f'robot302_bringup berhenti sendiri dengan return code {hw_process.returncode}.'
                )

            left_ok = topic_has_publisher('/left_encoder')
            right_ok = topic_has_publisher('/right_encoder')
            scan_ok = topic_has_publisher('/scan')

            elapsed = int(time.time() - wait_started)
            print(
                f'  -> left_encoder={"OK" if left_ok else "..."}, '
                f'right_encoder={"OK" if right_ok else "..."}, '
                f'scan={"OK" if scan_ok else "..."} [{elapsed}s]',
                flush=True,
            )

            if left_ok and right_ok and scan_ok:
                topics_ready = True
                break

            if elapsed >= STARTUP_TIMEOUT:
                print('\n[WARNING] Timeout menunggu publisher sensor.')
                print('[WARNING] Cek Micro-ROS Agent, LiDAR, dan port USB.')
                print('[WARNING] Startup tetap dilanjutkan agar diagnostik bisa dilakukan.')
                break

            time.sleep(TOPIC_CHECK_INTERVAL)

        if topics_ready:
            print('\n[ OK ] Publisher sensor terdeteksi.')
            left_data = topic_has_data('/left_encoder', timeout=3)
            right_data = topic_has_data('/right_encoder', timeout=3)
            scan_data = topic_has_data('/scan', timeout=3)

            print(f'[INFO] Data /left_encoder : {"DITERIMA" if left_data else "BELUM DITERIMA"}')
            print(f'[INFO] Data /right_encoder: {"DITERIMA" if right_data else "BELUM DITERIMA"}')
            print(f'[INFO] Data /scan         : {"DITERIMA" if scan_data else "BELUM DITERIMA"}')
            print('[OK] Sinkronisasi hardware selesai.\n')

        # -------------------------------------------------
        # 3. MAP MENU
        # -------------------------------------------------
        print('=' * 40)
        print('=== TAHAP 3: MENU NAVIGASI ===')
        print('=' * 40)

        for i, map_name in enumerate(MAPS, start=1):
            print(f'[{i}] {map_name}')

        idx_map = input(f'\nMasukkan nomor map (1-{len(MAPS)}) [Default: 1]: ').strip()
        if idx_map.isdigit() and 1 <= int(idx_map) <= len(MAPS):
            selected_map = MAPS[int(idx_map) - 1]
        else:
            selected_map = MAPS[0]

        home_dir = os.path.expanduser('~')
        map_full_path = os.path.join(
            home_dir,
            'ros2_ws', 'src', 'robot302_navigation', 'maps', selected_map,
        )

        if not os.path.isfile(map_full_path):
            raise FileNotFoundError(f'Map tidak ditemukan: {map_full_path}')

        # -------------------------------------------------
        # 4. NAV2
        # -------------------------------------------------
        print('\n[INFO] Menjalankan Navigation 2...')
        nav_cmd = [
            'ros2', 'launch', 'robot302_navigation', 'navigation.launch.py',
            f'map:={map_full_path}',
            'use_sim_time:=false',
        ]
        nav_process = start_process(nav_cmd, 'Nav2')

        # -------------------------------------------------
        # 5. RVIZ
        # -------------------------------------------------
        rviz_config_path = os.path.join(
            home_dir,
            'ros2_ws', 'src', 'robot302_description', 'rviz', 'nav2_302sim_view.rviz',
        )

        if not os.path.isfile(rviz_config_path):
            raise FileNotFoundError(f'RViz config tidak ditemukan: {rviz_config_path}')

        print('[INFO] Menjalankan RViz...')
        rviz_cmd = [
            'ros2', 'run', 'rviz2', 'rviz2',
            '-d', rviz_config_path,
            '--ros-args', '-p', 'use_sim_time:=false',
        ]
        rviz_process = start_process(rviz_cmd, 'RViz')

        # -------------------------------------------------
        # 6. MONITOR
        # -------------------------------------------------
        print('\n[ STATUS ] Sistem Robot Aktif!')
        print('[ STATUS ] Tekan Ctrl+C untuk mematikan seluruh sistem.')

        while True:
            # Detect unexpected death of child processes.
            if hw_process.poll() is not None:
                print(f'\n[WARNING] Bringup berhenti. Return code: {hw_process.returncode}')
                break

            if nav_process.poll() is not None:
                print(f'\n[WARNING] Nav2 berhenti. Return code: {nav_process.returncode}')
                break

            if rviz_process.poll() is not None:
                print(f'\n[WARNING] RViz berhenti. Return code: {rviz_process.returncode}')
                break

            time.sleep(1)

    except KeyboardInterrupt:
        print('\n[INFO] Ctrl+C diterima. Mematikan seluruh sistem...')

    except Exception as exc:
        print(f'\n[ERROR] {exc}')

    finally:
        print('\n' + '=' * 40)
        print('=== CLEANUP SESI SAAT INI ===')
        print('=' * 40)

        # Kill complete process groups created by this Python script.
        terminate_process_group(rviz_process, 'RViz')
        terminate_process_group(nav_process, 'Nav2')
        terminate_process_group(hw_process, 'Robot302 Bringup')

        # Safety net for stale child processes from this session.
        cleanup_stale_robot_processes()

        print('[OK] Sistem robot dihentikan.')


if __name__ == '__main__':
    main()