import os
import sys
import shutil
import cv2
import numpy as np
import pandas as pd
from datetime import datetime

def generate_multitier_focus_mission(
    map_image_path=r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png",
    map_width_m=31.0,
    map_height_m=48.7,
    roi_center_x=15.5,       # Tâm trọng điểm sa bàn
    roi_center_y=24.35,      
    orbit_radii=[5.0, 10.0, 14.0],  # 3 tầng vòng tròn: Cận tâm, Trung bình, và Sát viền biên
    altitude_m=3.5,          # Độ cao an toàn (1m < Z < 5m)
    speed_m_s=1.5,           # Tốc độ bay mượt
    camera_pitch_deg=-18.0,  # Góc low-angle AI
    zoom_factor="2x-3x"      # Zoom linh hoạt
):
    """
    Tạo quỹ đạo đa tầng đồng tâm (Multi-Tier Orbits) giúp drone quét từ tâm 
    mở rộng ra toàn bộ viền biên sa bàn, giải quyết triệt để góc khuất.
    """
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"flight_mission_multitier_{timestamp_str}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"[+] Tạo thư mục đa tầng: {output_dir}")

    # Đọc ảnh map
    if os.path.exists(map_image_path):
        map_img = cv2.imread(map_image_path)
    else:
        img_h, img_w = 974, 620
        map_img = np.ones((img_h, img_w, 3), dtype=np.uint8) * 180

    img_h, img_w = map_img.shape[:2]
    scale_x = img_w / map_width_m
    scale_y = img_h / map_height_m

    all_mission_data = []
    total_time = 0.0
    global_step = 1

    # Tạo chuỗi quỹ đạo qua từng tầng bán kính từ trong ra ngoài
    for tier_idx, radius in enumerate(orbit_radii):
        angles_deg = np.linspace(0, 360, 90) # 90 điểm mỗi vòng
        for angle in angles_deg:
            rad = np.radians(angle)
            x_m = roi_center_x + radius * np.cos(rad)
            y_m = roi_center_y + radius * np.sin(rad)

            # Khóa góc nhìn luôn ngắm vào tâm ROI
            dx = roi_center_x - x_m
            dy = roi_center_y - y_m
            yaw = np.degrees(np.arctan2(dx, dy)) % 360

            if global_step == 1:
                dist = 0.0
            else:
                prev = all_mission_data[-1]
                dist = np.sqrt((x_m - prev["X_m"])**2 + (y_m - prev["Y_m"])**2)
                total_time += dist / speed_m_s

            all_mission_data.append({
                "Step": global_step,
                "Tier_Level": tier_idx + 1,
                "Radius_m": radius,
                "X_m": round(x_m, 3),
                "Y_m": round(y_m, 3),
                "Altitude_m": altitude_m,
                "Speed_m_s": speed_m_s,
                "Gimbal_Pitch_deg": camera_pitch_deg,
                "Yaw_deg": round(yaw, 1),
                "Zoom_Level": zoom_factor,
                "Time_Sec": round(total_time, 2)
            })
            global_step += 1

    # Xuất file CSV
    df_mission = pd.DataFrame(all_mission_data)
    csv_path = os.path.join(output_dir, "multitier_orbit_plan.csv")
    df_mission.to_csv(csv_path, index=False, encoding="utf-8-sig")

    # Vẽ sơ đồ lên ảnh Map
    annotated_map = map_img.copy()
    for row in all_mission_data:
        px = int(row["X_m"] * scale_x)
        py = int(row["Y_m"] * scale_y)
        color_tier = (255, 0, 0) if row["Tier_Level"]==1 else ((0, 255, 255) if row["Tier_Level"]==2 else (0, 255, 0))
        cv2.circle(annotated_map, (px, py), 3, color_tier, -1)

    center_pt = (int(roi_center_x * scale_x), int(roi_center_y * scale_y))
    cv2.circle(annotated_map, center_pt, 10, (0, 0, 255), -1)
    cv2.putText(annotated_map, "MULTI-TIER FOCUS ZONE", (center_pt[0]-70, center_pt[1]-20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2, cv2.LINE_AA)

    map_out_path = os.path.join(output_dir, "annotated_multitier_map.png")
    cv2.imwrite(map_out_path, annotated_map)

    # Xuất Video MP4 mô phỏng
    video_path = os.path.join(output_dir, "flight_multitier_simulation.mp4")
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (img_w, img_h))

    for row in all_mission_data:
        frame = annotated_map.copy()
        drone_pos = (int(row["X_m"] * scale_x), int(row["Y_m"] * scale_y))
        cv2.circle(frame, drone_pos, 8, (255, 0, 255), -1)
        cv2.line(frame, drone_pos, center_pt, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.rectangle(frame, (10, 10), (400, 110), (0, 0, 0), -1)
        cv2.putText(frame, f"MODE: MULTI-TIER ORBIT (TIER {row['Tier_Level']})", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        cv2.putText(frame, f"TIME: {row['Time_Sec']:.1f}s | RADIUS: {row['Radius_m']}m", (20, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(frame, f"COVERAGE: CENTER TO MAP BORDER", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)
        video_writer.write(frame)

    video_writer.release()

    # SAO CHÉP CODE CHẠY VÀO TRONG THƯ MỤC
    current_script_path = os.path.abspath(__file__) if '__file__' in globals() else sys.argv[0]
    target_code_path = os.path.join(output_dir, "ĐÂY LÀ CODE CHẠY .py")
    if os.path.exists(current_script_path):
        shutil.copy(current_script_path, target_code_path)
    else:
        with open(target_code_path, "w", encoding="utf-8") as f:
            f.write("# Multi-tier mission code\n")

    print(f"[+] Hoàn tất! Folder kết quả: {output_dir}")

if __name__ == "__main__":
    generate_multitier_focus_mission()