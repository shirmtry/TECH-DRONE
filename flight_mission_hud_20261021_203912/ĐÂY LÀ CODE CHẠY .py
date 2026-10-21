import os
import sys
import shutil
import cv2
import numpy as np
import pandas as pd
from datetime import datetime

def generate_perfect_100_coverage_with_hud(
    map_image_path=r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png",
    map_width_m=31.0,
    map_height_m=48.7,
    altitude_m=2.0,          # Độ cao an toàn 2m
    speed_m_s=1.2,           # Tốc độ di chuyển chuẩn 1.2 m/s
    camera_pitch_deg=-20.0,  # Low-angle pitch cho AI
    grid_step_m=2.5,         # Bước nhảy lưới 2.5m
    margin_m=1.5             # Lề sát biên 1.5m
):
    """
    Tạo lộ trình quét phủ 100% kèm giao diện HUD hiển thị rõ Tốc độ, Thời gian (ETA),
    Độ cao và tọa độ chuyển động trực tiếp trên Video mô phỏng và Ảnh bản đồ.
    """
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"flight_mission_hud_{timestamp_str}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"[+] Khởi tạo thư mục kết quả HUD: {output_dir}")

    # Đọc ảnh bản đồ
    if os.path.exists(map_image_path):
        map_img = cv2.imread(map_image_path)
    else:
        img_h, img_w = 974, 620
        map_img = np.ones((img_h, img_w, 3), dtype=np.uint8) * 180

    img_h, img_w = map_img.shape[:2]
    scale_x = img_w / map_width_m
    scale_y = img_h / map_height_m

    x_min, x_max = margin_m, map_width_m - margin_m
    y_min, y_max = margin_m, map_height_m - margin_m

    control_waypoints = []

    # 1. Vành đai ngoại biên khóa 4 góc
    perimeter_ring = [
        [x_min, y_min], [x_max, y_min], 
        [x_max, y_max], [x_min, y_max], [x_min, y_min]
    ]
    control_waypoints.extend(perimeter_ring)

    # 2. Lưới zic-zac dày đặc phủ kín 100%
    x_lanes = np.arange(x_min, x_max + 0.1, grid_step_m)
    going_down = True
    for x in x_lanes:
        if going_down:
            control_waypoints.append([x, y_min])
            control_waypoints.append([x, y_max])
        else:
            control_waypoints.append([x, y_max])
            control_waypoints.append([x, y_min])
        going_down = not going_down

    # Nội suy tuyến tính mượt mà
    smooth_x, smooth_y = [], []
    for i in range(len(control_waypoints) - 1):
        p1 = np.array(control_waypoints[i])
        p2 = np.array(control_waypoints[i+1])
        steps = max(15, int(np.linalg.norm(p2 - p1) * 6))
        xs = np.linspace(p1[0], p2[0], steps)
        ys = np.linspace(p1[1], p2[1], steps)
        smooth_x.extend(xs)
        smooth_y.extend(ys)

    smooth_x = np.clip(np.array(smooth_x), 1.0, map_width_m - 1.0)
    smooth_y = np.clip(np.array(smooth_y), 1.0, map_height_m - 1.0)

    flight_data = []
    total_time = 0.0

    for idx in range(len(smooth_x)):
        x_m = smooth_x[idx]
        y_m = smooth_y[idx]

        if idx == 0:
            dx, dy = 1.0, 0.0
            dist = 0.0
        else:
            prev = flight_data[-1]
            dx = x_m - prev["X_m"]
            dy = y_m - prev["Y_m"]
            dist = np.sqrt(dx**2 + dy**2)
            total_time += dist / speed_m_s

        yaw = np.degrees(np.arctan2(dx, dy)) % 360

        flight_data.append({
            "Step": idx + 1,
            "X_m": round(x_m, 3),
            "Y_m": round(y_m, 3),
            "Altitude_m": altitude_m,
            "Speed_m_s": speed_m_s,
            "Gimbal_Pitch_deg": camera_pitch_deg,
            "Yaw_deg": round(yaw, 1),
            "Time_Sec": round(total_time, 2)
        })

    # Xuất file CSV cấu hình cho DJI
    df_flight = pd.DataFrame(flight_data)
    csv_path = os.path.join(output_dir, "drone_mission_config.csv")
    df_flight.to_csv(csv_path, index=False, encoding="utf-8-sig")

    # Vẽ sơ đồ tuyến đường lên ảnh Map
    base_map = map_img.copy()
    pixel_pts = [(int(r["X_m"] * scale_x), int(r["Y_m"] * scale_y)) for r in flight_data]
    for i in range(len(pixel_pts) - 1):
        cv2.line(base_map, pixel_pts[i], pixel_pts[i+1], (0, 255, 0), 2, cv2.LINE_AA)

    map_out_path = os.path.join(output_dir, "annotated_map_with_path.png")
    cv2.imwrite(map_out_path, base_map)

    # Xuất Video MP4 Mô phỏng có hiển thị Tốc độ & Thời gian trực quan (HUD)
    video_path = os.path.join(output_dir, "flight_simulation_with_hud.mp4")
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (img_w, img_h))

    step_skip = max(1, len(flight_data) // 600)
    for idx in range(0, len(flight_data), step_skip):
        row = flight_data[idx]
        frame = base_map.copy()
        drone_pos = (int(row["X_m"] * scale_x), int(row["Y_m"] * scale_y))

        # Vẽ biểu tượng Drone
        cv2.circle(frame, drone_pos, 8, (255, 0, 0), -1)
        cv2.circle(frame, drone_pos, 12, (0, 255, 255), 2)

        # Bảng thông số HUD hiển thị Tốc độ và Thời gian thực tế ngay trên video
        cv2.rectangle(frame, (15, 15), (440, 130), (0, 0, 0), -1)
        cv2.putText(frame, f"MISSION: 100% FULL COVERAGE GRID", (25, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        cv2.putText(frame, f"TIME (ETA): {row['Time_Sec']:.1f}s / {total_time:.1f}s", (25, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(frame, f"SPEED: {row['Speed_m_s']} m/s  |  ALTITUDE: {row['Altitude_m']} m", (25, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(frame, f"GIMBAL PITCH: {row['Gimbal_Pitch_deg']}deg  |  YAW: {row['Yaw_deg']}deg", (25, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 200, 200), 1)

        video_writer.write(frame)

    video_writer.release()

    # Sao chép file code chạy vào thư mục với tên yêu cầu
    current_script_path = os.path.abspath(__file__) if '__file__' in globals() else sys.argv[0]
    target_code_path = os.path.join(output_dir, "ĐÂY LÀ CODE CHẠY .py")
    if os.path.exists(current_script_path):
        shutil.copy(current_script_path, target_code_path)
    else:
        with open(target_code_path, "w", encoding="utf-8") as f:
            f.write("# Mission script with speed & time HUD\n")

    print(f"[+] Hoàn tất thành công! Thư mục kết quả: {output_dir}")

if __name__ == "__main__":
    generate_perfect_100_coverage_with_hud()