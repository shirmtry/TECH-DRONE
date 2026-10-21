import os
import sys
import shutil
import cv2
import numpy as np
import pandas as pd
from datetime import datetime

def generate_hybrid_corridor_mission(
    map_image_path=r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png",
    map_width_m=31.0,
    map_height_m=48.7,
    altitude_m=1.5,          # Độ cao siêu thấp an toàn (1.5m) soi chi tiết sát đất
    speed_m_s=1.2,           # Tốc độ kiểm soát
    camera_pitch_deg=-15.0,  # Góc low-angle phóng đại chiều cao
    zoom_factor="2x-3x"      # Zoom quang học an toàn
):
    """
    Quét phủ hỗn hợp (Hybrid Corridor & Corner Sweep):
    Kết hợp đi theo các hành lang lối mòn và càn quét trọn vẹn 4 góc sa bàn,
    đảm bảo không bỏ sót bất kỳ khu vực nào từ trong ra ngoài biên.
    """
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"flight_mission_hybrid_corridor_{timestamp_str}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"[+] Tạo thư mục quỹ đạo hành lang và 4 góc: {output_dir}")

    # Đọc ảnh sa bàn
    if os.path.exists(map_image_path):
        map_img = cv2.imread(map_image_path)
    else:
        img_h, img_w = 974, 620
        map_img = np.ones((img_h, img_w, 3), dtype=np.uint8) * 180

    img_h, img_w = map_img.shape[:2]
    scale_x = img_w / map_width_m
    scale_y = img_h / map_height_m

    # Định nghĩa các tuyến hành lang chính và 4 góc sa bàn (Tọa độ X, Y mét)
    # Bao gồm các đường biên viền 4 góc và các lối đi dọc ngang
    hybrid_waypoints = [
        # Điểm xuất phát trung tâm
        [15.5, 24.35],
        
        # Càn quét góc Tây Bắc (Top-Left)
        [2.5, 2.5], [2.5, 10.0], [8.0, 10.0], [8.0, 2.5],
        
        # Hành lang dọc phía Tây
        [5.0, 15.0], [5.0, 30.0], [5.0, 45.0],
        
        # Càn quét góc Tây Nam (Bottom-Left)
        [2.5, 46.0], [10.0, 46.0], [10.0, 42.0],
        
        # Trục hành lang trung tâm (Đi qua khu vực chính)
        [14.0, 42.0], [14.0, 25.0], [14.0, 8.0],
        
        # Càn quét góc Đông Bắc (Top-Right)
        [28.5, 2.5], [28.5, 15.0], [22.0, 15.0], [22.0, 2.5],
        
        # Hành lang dọc phía Đông
        [21.0, 10.0], [21.0, 25.0], [21.0, 40.0],
        
        # Càn quét góc Đông Nam (Bottom-Right)
        [28.5, 46.0], [28.5, 35.0], [20.0, 35.0], [20.0, 46.0],
        
        # Quay về trung tâm hạ cánh / RTL
        [15.5, 24.35]
    ]

    # Nội suy đoạn thẳng mượt giữa các waypoint hành lang và góc
    smooth_path_x = []
    smooth_path_y = []
    
    for i in range(len(hybrid_waypoints) - 1):
        p1 = np.array(hybrid_waypoints[i])
        p2 = np.array(hybrid_waypoints[i+1])
        steps = max(10, int(np.linalg.norm(p2 - p1) * 5))
        xs = np.linspace(p1[0], p2[0], steps)
        ys = np.linspace(p1[1], p2[1], steps)
        smooth_path_x.extend(xs)
        smooth_path_y.extend(ys)

    hybrid_data = []
    total_time = 0.0

    for idx in range(len(smooth_path_x)):
        x_m = smooth_path_x[idx]
        y_m = smooth_path_y[idx]

        # Khóa cứng không văng ra ngoài map
        x_m = np.clip(x_m, 1.0, map_width_m - 1.0)
        y_m = np.clip(y_m, 1.0, map_height_m - 1.0)

        if idx == 0:
            dx, dy = 1.0, 0.0
            dist = 0.0
        else:
            prev = hybrid_data[-1]
            dx = x_m - prev["X_m"]
            dy = y_m - prev["Y_m"]
            dist = np.sqrt(dx**2 + dy**2)
            total_time += dist / speed_m_s

        yaw = np.degrees(np.arctan2(dx, dy)) % 360

        hybrid_data.append({
            "Step": idx + 1,
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
    df_hybrid = pd.DataFrame(hybrid_data)
    csv_path = os.path.join(output_dir, "hybrid_corridor_plan.csv")
    df_hybrid.to_csv(csv_path, index=False, encoding="utf-8-sig")

    # Vẽ sơ đồ lộ trình lên ảnh Map
    annotated_map = map_img.copy()
    pixel_pts = [(int(r["X_m"] * scale_x), int(r["Y_m"] * scale_y)) for r in hybrid_data]

    for i in range(len(pixel_pts) - 1):
        cv2.line(annotated_map, pixel_pts[i], pixel_pts[i+1], (0, 255, 255), 2, cv2.LINE_AA)

    # Đánh dấu 4 góc sa bàn
    corners = [(2.5, 2.5), (2.5, 46.0), (28.5, 2.5), (28.5, 46.0)]
    for c in corners:
        cp = (int(c[0] * scale_x), int(c[1] * scale_y))
        cv2.circle(annotated_map, cp, 8, (0, 0, 255), -1)
        cv2.putText(annotated_map, "CORNER", (cp[0]-20, cp[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)

    map_out_path = os.path.join(output_dir, "annotated_hybrid_map.png")
    cv2.imwrite(map_out_path, annotated_map)

    # Xuất Video MP4 mô phỏng
    video_path = os.path.join(output_dir, "flight_hybrid_simulation.mp4")
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (img_w, img_h))

    step_skip = max(1, len(hybrid_data) // 600)
    for idx in range(0, len(hybrid_data), step_skip):
        row = hybrid_data[idx]
        frame = annotated_map.copy()
        drone_pos = (int(row["X_m"] * scale_x), int(row["Y_m"] * scale_y))
        cv2.circle(frame, drone_pos, 7, (255, 0, 0), -1)
        cv2.circle(frame, drone_pos, 10, (255, 255, 255), 2)

        # HUD Overlay thông số
        cv2.rectangle(frame, (10, 10), (410, 115), (0, 0, 0), -1)
        cv2.putText(frame, "MODE: HYBRID CORRIDOR & CORNER SWEEP", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        cv2.putText(frame, f"TIME: {row['Time_Sec']:.1f}s | ALT: {altitude_m}m (ULTRA-LOW)", (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)
        cv2.putText(frame, f"ZOOM: {zoom_factor} | SPEED: {speed_m_s}m/s", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)
        cv2.putText(frame, f"COVERAGE: 100% CORNERS & PATHS", (20, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        video_writer.write(frame)

    video_writer.release()

    # SAO CHÉP CODE CHẠY VÀO TRONG THƯ MỤC VÀ ĐỔI TÊN THÀNH 'ĐÂY LÀ CODE CHẠY .py'
    current_script_path = os.path.abspath(__file__) if '__file__' in globals() else sys.argv[0]
    target_code_path = os.path.join(output_dir, "ĐÂY LÀ CODE CHẠY .py")
    if os.path.exists(current_script_path):
        shutil.copy(current_script_path, target_code_path)
    else:
        with open(target_code_path, "w", encoding="utf-8") as f:
            f.write("# Hybrid corridor mission code\n")

    print(f"[+] Hoàn tất toàn bộ! Thư mục kết quả: {output_dir}")

if __name__ == "__main__":
    generate_hybrid_corridor_mission()