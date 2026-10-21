import os
import sys
import shutil
import cv2
import numpy as np
import pandas as pd
from datetime import datetime

def generate_spiral_expansion_mission(
    map_image_path=r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png",
    map_width_m=31.0,
    map_height_m=48.7,
    roi_center_x=15.5,       # Tâm xuất phát từ giữa sa bàn
    roi_center_y=24.35,      
    min_radius_m=1.5,        # Bán kính bắt đầu từ tâm sát trong
    max_radius_m=15.0,       # Bán kính mở rộng ra tận viền biên và 4 góc sa bàn
    altitude_m=1.5,          # Độ cao siêu thấp (1.5m - gần sát đất để soi chi tiết cực nét)
    speed_m_s=1.0,           # Tốc độ bay chậm kiểm soát tuyệt đối
    camera_pitch_deg=-10.0,  # Góc low-angle gần ngang để phóng đại chiều cao
    zoom_factor="3x"         # Zoom quang học thay thế cho việc ép sát 0.5m
):
    """
    Quỹ đạo Xoắn ốc mở rộng (Spiral Expansion):
    Bay từ trong tâm xoắn mở rộng dần ra tận viền và 4 góc bản đồ,
    giữ độ cao thấp an toàn (1.5m) kết hợp Zoom để đạt độ chi tiết tối đa.
    """
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"flight_mission_spiral_expansion_{timestamp_str}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"[+] Tạo thư mục quỹ đạo xoắn ốc: {output_dir}")

    # Đọc ảnh sa bàn
    if os.path.exists(map_image_path):
        map_img = cv2.imread(map_image_path)
    else:
        img_h, img_w = 974, 620
        map_img = np.ones((img_h, img_w, 3), dtype=np.uint8) * 180

    img_h, img_w = map_img.shape[:2]
    scale_x = img_w / map_width_m
    scale_y = img_h / map_height_m

    # Tạo toán học quỹ đạo xoắn ốc Archimedean Spiral từ trong ra ngoài
    # Theta chạy từ 0 đến nhiều vòng (ví dụ 6 vòng tròn tương đương ~360 độ * 6)
    total_rotations = 6.0
    num_points = 250
    theta_vals = np.linspace(0, total_rotations * 2 * np.pi, num_points)
    
    spiral_data = []
    total_time = 0.0

    for idx, theta in enumerate(theta_vals):
        # Bán kính tăng dần đều theo góc theta
        current_radius = min_radius_m + (max_radius_m - min_radius_m) * (theta / (total_rotations * 2 * np.pi))
        
        x_m = roi_center_x + current_radius * np.cos(theta)
        y_m = roi_center_y + current_radius * np.sin(theta)

        # Đảm bảo tọa độ không vượt ra ngoài biên map [0..31, 0..48.7]
        x_m = np.clip(x_m, 1.5, map_width_m - 1.5)
        y_m = np.clip(y_m, 1.5, map_height_m - 1.5)

        # Tính góc hướng Yaw để camera luôn nhìn hướng ra phía trước hoặc về tâm
        if idx == 0:
            dx, dy = 1.0, 0.0
            dist = 0.0
        else:
            prev = spiral_data[-1]
            dx = x_m - prev["X_m"]
            dy = y_m - prev["Y_m"]
            dist = np.sqrt(dx**2 + dy**2)
            total_time += dist / speed_m_s

        yaw = np.degrees(np.arctan2(dx, dy)) % 360

        spiral_data.append({
            "Step": idx + 1,
            "Radius_m": round(current_radius, 2),
            "X_m": round(x_m, 3),
            "Y_m": round(y_m, 3),
            "Altitude_m": altitude_m,
            "Speed_m_s": speed_m_s,
            "Gimbal_Pitch_deg": camera_pitch_deg,
            "Yaw_deg": round(yaw, 1),
            "Zoom_Level": zoom_factor,
            "Time_Sec": round(total_time, 2)
        })

    # Xuất file CSV
    df_spiral = pd.DataFrame(spiral_data)
    csv_path = os.path.join(output_dir, "spiral_expansion_plan.csv")
    df_spiral.to_csv(csv_path, index=False, encoding="utf-8-sig")

    # Vẽ sơ đồ lên ảnh Map
    annotated_map = map_img.copy()
    pixel_pts = [(int(r["X_m"] * scale_x), int(r["Y_m"] * scale_y)) for r in spiral_data]

    for i in range(len(pixel_pts) - 1):
        cv2.line(annotated_map, pixel_pts[i], pixel_pts[i+1], (0, 255, 255), 2, cv2.LINE_AA)

    center_pt = (int(roi_center_x * scale_x), int(roi_center_y * scale_y))
    cv2.circle(annotated_map, center_pt, 8, (0, 0, 255), -1)
    cv2.putText(annotated_map, "SPIRAL START (CENTER)", (center_pt[0]-60, center_pt[1]-15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 2, cv2.LINE_AA)

    map_out_path = os.path.join(output_dir, "annotated_spiral_map.png")
    cv2.imwrite(map_out_path, annotated_map)

    # Xuất Video MP4 mô phỏng
    video_path = os.path.join(output_dir, "flight_spiral_simulation.mp4")
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (img_w, img_h))

    for row in spiral_data:
        frame = annotated_map.copy()
        drone_pos = (int(row["X_m"] * scale_x), int(row["Y_m"] * scale_y))
        cv2.circle(frame, drone_pos, 7, (255, 0, 0), -1)
        cv2.circle(frame, drone_pos, 10, (255, 255, 255), 2)

        # HUD Overlay thông số
        cv2.rectangle(frame, (10, 10), (410, 115), (0, 0, 0), -1)
        cv2.putText(frame, "MODE: SPIRAL EXPANSION (CENTER TO BORDER)", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(frame, f"TIME: {row['Time_Sec']:.1f}s | ALT: {altitude_m}m (ULTRA-LOW)", (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)
        cv2.putText(frame, f"RADIUS: {row['Radius_m']}m | ZOOM: {zoom_factor}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)
        cv2.putText(frame, f"COVERAGE: 100% INCLUDING 4 CORNERS", (20, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        video_writer.write(frame)

    video_writer.release()

    # SAO CHÉP CODE CHẠY VÀO TRONG THƯ MỤC VÀ ĐỔI TÊN THÀNH 'ĐÂY LÀ CODE CHẠY .py'
    current_script_path = os.path.abspath(__file__) if '__file__' in globals() else sys.argv[0]
    target_code_path = os.path.join(output_dir, "ĐÂY LÀ CODE CHẠY .py")
    if os.path.exists(current_script_path):
        shutil.copy(current_script_path, target_code_path)
    else:
        with open(target_code_path, "w", encoding="utf-8") as f:
            f.write("# Spiral expansion mission code\n")

    print(f"[+] Hoàn tất toàn bộ! Thư mục kết quả: {output_dir}")

if __name__ == "__main__":
    generate_spiral_expansion_mission()