#!/usr/bin/env python3

import os
import json
import time
import cv2
from ultralytics import YOLO

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ── KONFIGURASI NAVIGASI (Dari patroli.py) ─────────────────────────
STOP_TIME = 3.0           # Waktu diam (detik) saat sampai sebelum kamera buka
WAYPOINT_TIMEOUT = 120.0  # Batas waktu maksimal menuju 1 titik (detik)
MAX_RETRIES = 2
POLL_INTERVAL = 0.1

# Daftar 3 posisi scan (+ opsional kembali ke titik awal jika scan=False)
# Sesuaikan koordinat X dan Y dengan map yang kamu pilih (arena_robot302 / my_map)
WAYPOINTS = [
   {"name": "B", "x": 46.24986267089844, "y": -3.956498146057129, "scan": True},
    {"name": "C", "x": 22.08811378479004, "y": 2.3347811698913574,  "scan": True},
    {"name": "D", "x": 0.49029019474983215, "y": -0.07147882133722305, "scan": True},
    # Titik A: diatur scan=False jika hanya untuk titik kembali/home. 
    # Ubah ke scan=True jika titik A juga memiliki objek untuk di-scan.
    {"name": "A", "x": 48.143089294433594, "y": 0.3452625572681427, "scan": False},
]

# ── KONFIGURASI VISION / YOLO (Dari vision_sensor.py) ──────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(SCRIPT_DIR, "best.pt")

# Lokasi penyimpanan data hasil deteksi (di Home directory agar mudah diakses)
RESULTS_FILE = os.path.expanduser("~/robot302_scan_results.json")

CONFIDENCE = 0.85
HOLD_SECONDS_APPEAR = 2.0   # Objek harus stabil selama 2 detik untuk dikonfirmasi
SCAN_TIMEOUT = 15.0         # Batas waktu maksimal kamera menyala di 1 titik (jika 0 objek)
CAMERA_WARMUP = 5.0         # Waktu tunggu auto-exposure kamera saat baru dibuka
CAMERA_INDEX = 1            # Ganti ke 0 atau 2 jika di Ubuntu kamera eksternal pindah index
SHOW_WINDOW = True          # Ubah ke False jika berjalan tanpa layar (headless)
WINDOW_NAME = "Vision Robot302"
TARGET_CLASSES = ["bleu_cube", "bleu_sphere"]
# ───────────────────────────────────────────────────────────────────


def load_yolo_model(path):
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"File model YOLO tidak ditemukan di: {path}\n"
            f"Pastikan 'best.pt' berada di folder yang sama dengan script ini\n"
            f"dan jalankan 'colcon build --symlink-install'."
        )
    print(f"[VISION] Memuat model YOLO dari: {path}")
    return YOLO(path)


def open_camera(preferred_index):
    """Membuka kamera dengan fallback otomatis jika index berubah di Linux."""
    for idx in [preferred_index, 0, 2, 1]:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            print(f"[VISION] Kamera berhasil dibuka pada index {idx}.")
            return cap
    raise RuntimeError(
        f"Gagal membuka kamera pada index {preferred_index} (maupun 0/2). "
        "Pastikan USB kamera terpasang."
    )


def get_class_counts(result, model):
    """Menghitung jumlah masing-masing class (bleu_cube & bleu_sphere) pada frame."""
    counts = {}
    for c in result.boxes.cls:
        label = model.names[int(c)]
        counts[label] = counts.get(label, 0) + 1
    return counts


def scan_objects_at_waypoint(model, waypoint_name):
    """
    Membuka kamera di waypoint, mendeteksi bleu_cube dan bleu_sphere hingga stabil
    selama HOLD_SECONDS_APPEAR (2.0 detik), lalu menutup kamera dan mengembalikan data.
    """
    print()
    print("-" * 60)
    print(f"[VISION] Memulai pemindaian kamera di {waypoint_name}...")
    print("-" * 60)

    cap = open_camera(CAMERA_INDEX)

    if SHOW_WINDOW:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # Hasil default (0 jika tidak ada objek sampai timeout)
    final_counts = {cls_name: 0 for cls_name in TARGET_CLASSES}
    candidate_counts = {}
    candidate_since = None

    start_time = time.monotonic()
    detection_confirmed = False

    try:
        while cap.isOpened():
            success, frame = cap.read()
            if not success:
                print("[VISION] Gagal membaca frame dari kamera.")
                break

            now = time.monotonic()
            elapsed = now - start_time

            # 1. Cek batas waktu scan (agar robot tidak nyangkut jika area kosong)
            if elapsed >= SCAN_TIMEOUT:
                print(
                    f"[VISION] Waktu scan habis ({SCAN_TIMEOUT:.0f}s). "
                    f"Menyimpan hasil terakhir di {waypoint_name}."
                )
                # Jika ada kandidat yang sedang terlihat saat timeout, simpan itu
                for cls_name in TARGET_CLASSES:
                    final_counts[cls_name] = candidate_counts.get(cls_name, 0)
                break

            # 2. Jalankan prediksi YOLO
            result = model.predict(source=frame, conf=CONFIDENCE, verbose=False)[0]
            annotated_frame = result.plot()

            # 3. Tampilkan status pada layar kamera
            if SHOW_WINDOW:
                status_text = f"{waypoint_name} | Time: {elapsed:.1f}s/{SCAN_TIMEOUT:.0f}s"
                cv2.putText(
                    annotated_frame, status_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
                )
                cv2.imshow(WINDOW_NAME, annotated_frame)
                if cv2.waitKey(30) & 0xFF == ord('q'):
                    print("[VISION] Scan dihentikan manual oleh user ('q').")
                    break

            # Beri waktu 1 detik di awal agar kamera fokus / auto-exposure stabil
            if elapsed < CAMERA_WARMUP:
                continue

            # 4. Cek jumlah objek yang terdeteksi di frame ini
            current_counts = get_class_counts(result, model)

            if current_counts:
                # Ada minimal 1 objek (bleu_cube dan/atau bleu_sphere)
                if current_counts != candidate_counts:
                    # Kombinasi jumlah objek berubah -> reset timer stabilitas
                    candidate_counts = current_counts.copy()
                    candidate_since = now
                else:
                    # Kombinasi jumlah objek sama dengan frame sebelumnya
                    hold_duration = now - candidate_since
                    print(
                        f"[VISION] Mendeteksi {current_counts} "
                        f"(stabil {hold_duration:.1f}s / {HOLD_SECONDS_APPEAR:.1f}s)...",
                        end="\r",
                        flush=True,
                    )
                    if hold_duration >= HOLD_SECONDS_APPEAR:
                        print()
                        for cls_name in TARGET_CLASSES:
                            final_counts[cls_name] = current_counts.get(cls_name, 0)
                        detection_confirmed = True
                        print(
                            f"[+] Deteksi Terkonfirmasi di {waypoint_name}: "
                            f"bleu_cube={final_counts['bleu_cube']}, "
                            f"bleu_sphere={final_counts['bleu_sphere']}"
                        )
                        break  # Objek sudah terkonfirmasi! Keluar loop untuk tutup kamera
            else:
                # Tidak ada objek di frame ini
                candidate_counts = {}
                candidate_since = None

    finally:
        cap.release()
        if SHOW_WINDOW:
            cv2.destroyAllWindows()
            # Penting di Linux/Ubuntu: paksa event loop OpenCV menutup window
            for _ in range(5):
                cv2.waitKey(1)
        print(f"[VISION] Kamera ditutup di {waypoint_name}.")

    return final_counts, detection_confirmed


def save_detection_data(all_results, waypoint_name, counts, confirmed):
    """Menyimpan hasil deteksi setiap area ke dalam file JSON."""
    all_results["locations"][waypoint_name] = {
        "bleu_cube": counts.get("bleu_cube", 0),
        "bleu_sphere": counts.get("bleu_sphere", 0),
        "total_items": sum(counts.values()),
        "status": "confirmed" if confirmed else "timeout_or_empty",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
    }

    with open(RESULTS_FILE, "w", encoding="utf-8") as f:
        json.dump(all_results, f, indent=4)

    print(f"[DATA] Data {waypoint_name} berhasil disimpan ke: {RESULTS_FILE}")


def create_pose(navigator, x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

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
        print(f"\nPercobaan {attempt}/{total_attempts} Waypoint {name}")

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
            print(f"Berhenti selama {STOP_TIME:.1f} detik agar robot tenang...")
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
    # 1. Load model YOLO sekali di awal agar tidak lag di setiap titik
    model = load_yolo_model(MODEL_PATH)

    # Siapkan struktur data untuk misi ini
    mission_data = {
        "mission_start": time.strftime("%Y-%m-%d %H:%M:%S"),
        "locations": {},
    }

    rclpy.init()
    navigator = BasicNavigator()

    try:
        print()
        print("=" * 60)
        print("ROBOT PATROLI + COMPUTER VISION YOLO")
        print("Menunggu Nav2 aktif...")
        print("=" * 60)

        navigator.waitUntilNav2Active()

        print("Nav2 aktif.")
        print(f"Jumlah waypoint: {len(WAYPOINTS)}")

        for index, waypoint in enumerate(WAYPOINTS, start=1):
            name = waypoint["name"]
            should_scan = waypoint.get("scan", True)

            print()
            print(f"[{index}/{len(WAYPOINTS)}] Waypoint {name}")

            # A. Navigasi ke Waypoint
            success = navigate_to_waypoint(navigator, waypoint)

            if not success:
                print()
                print("=" * 60)
                print(f"PATROLI DIHENTIKAN pada Waypoint {name}")
                print("=" * 60)
                break

            # B. Jika sampai dan titik ini perlu di-scan, buka kamera & deteksi
            if should_scan:
                counts, confirmed = scan_objects_at_waypoint(model, name)
                save_detection_data(mission_data, name, counts, confirmed)
                print(f"[INFO] Selesai di {name}, bersiap menuju titik berikutnya...")

        else:
            print()
            print("=" * 60)
            print("PATROLI & DETEKSI VISION SELESAI!")
            print(f"File hasil tersimpan di: {RESULTS_FILE}")
            print("-" * 60)
            for loc, info in mission_data["locations"].items():
                print(
                    f"  {loc}: bleu_cube = {info['bleu_cube']}, "
                    f"bleu_sphere = {info['bleu_sphere']} "
                    f"(Total: {info['total_items']})"
                )
            print("=" * 60)

    except KeyboardInterrupt:
        print("\nPatroli dihentikan oleh user.")
        try:
            navigator.cancelTask()
        except Exception:
            pass

    except Exception as e:
        print(f"\nERROR: {e}")
        try:
            navigator.cancelTask()
        except Exception:
            pass

    finally:
        cv2.destroyAllWindows()
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    main()