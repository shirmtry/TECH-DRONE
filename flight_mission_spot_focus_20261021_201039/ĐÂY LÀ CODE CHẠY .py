import os
import sys
import shutil
import cv2
import numpy as np
import pandas as pd
from datetime import datetime

def generate_spot_focus_mission(
    map_image_path=r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png",
    map_width_m=31.0,
    map_height_m=48.7,
    roi_center_x=15.5,       # Tọa độ X tâm vùng trọng điểm cần quét tập trung (Chính giữa sa bàn)
    roi_center_y=24.35,      # Tọa độ Y tâm vùng trọng điểm
    orbit_radius_m=8.0,      # Bán kính drone xoay vòng tập trung xung quanh ROI (8 mét)
    altitude_m=3.5,          # Độ cao bay an toàn (nằm trong khoảng 1m < Z < 5m)
    speed_m_s=1.2,           # Tốc độ quay chậm mượt để AI bắt nét sâu
    camera_pitch_deg=-15.0,  # Low-angle pitch (-15 độ) để phóng đại chiều cao vật thể
    zoom_factor="3x"         # Tận dụng Tele 48MP Zoom xa an toàn
):
    """
    Phương pháp Bay Khoanh Vùng Tiêu Cự (Orbit Spot Focus / POI):
    Tập trung 100% thời gian bay vào 1 vùng trọng điểm (ROI), đứng xa zoom lại
    để vừa giữ được chiều cao lớn của vật thể, vừa tiết kiệm pin và tuyệt đối không va chạm.
    """
    # 1. Tạo thư mục kết quả theo dạng timestamp
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"flight_mission_spot_focus_{timestamp_str}"
    os.makedirs(output_dir, exist_ok=True)
    print(f"[+] Đã khởi tạo thư mục kết quả Spot Focus: {output_dir}")

    # 2. Đọc ảnh sa bàn
    if os.path.exists(map_image_path):
        map_img = cv2.imread(map_image_path)
    else:
        img_h, img_w = 974, 620
        map_img = np.ones((img_h, img_w, 3), dtype=np.uint8) * 180

    img_h, img_w = map_img.shape[:2]
    scale_x = img_w / map_width_m
    scale_y = img_h / map_height_m

    # 3. Tính toán quỹ đạo quay vòng tròn Orbit (360 độ xung quanh điểm ROI)
    angles_deg = np.linspace(0, 360, 120)  # 120 bước mẫu quanh vòng tròn Focus
    orbit_data = []
    total_time = 0.0

    for idx, angle in enumerate(angles_deg):
        rad = np.radians(angle)
        x_m = roi_center_x + orbit_radius_m * np.cos(rad)
        y_m = roi_center_y + orbit_radius_m * np.sin(rad)

        # Tính góc Yaw để camera LUÔN KHÓA CHẶT TÂM VÀO VÙNG ROI
        dx = roi_center_x - x_m
        dy = roi_center_y - y_m
        yaw = np.degrees(np.arctan2(dx, dy)) % 360

        if idx == 0:
            dist = 0.0
        else:
            prev = orbit_data[-1]
            dist = np.sqrt((x_m - prev["X_m"])**2 + (y_m - prev["Y_m"])**2)
            total_time += dist / speed_m_s

        orbit_data.append({
            "Step": idx + 1,
            "X_m": round(x_m, 3),
            "Y_m": round(y_m, 3),
            "Altitude_m": altitude_m,
            "Speed_m_s": speed_m_s,
            "Gimbal_Pitch_deg": camera_pitch_deg,
            "Yaw_deg": round(yaw, 1),
            "Zoom_Level": zoom_factor,
            "Focus_Target_X": roi_center_x,
            "Focus_Target_Y": roi_center_y,
            "Time_Sec": round(total_time, 2)
        })

    # Xuất file CSV cấu hình
    df_orbit = pd.DataFrame(orbit_data)
    csv_path = os.path.join(output_dir, "spot_focus_orbit_plan.csv")
    df_orbit.to_csv(csv_path, index=False, encoding="utf-8-sig")
    print(f"[+] Đã xuất CSV kế hoạch bay Orbit Focus: {csv_path}")

    # 4. Vẽ sơ đồ Lộ trình Orbit Focus lên ảnh Map
    annotated_map = map_img.copy()
    pixel_pts = [(int(r["X_m"] * scale_x), int(r["Y_m"] * scale_y)) for r in orbit_data]

    # Vẽ đường tròn quỹ đạo bay (màu vàng nhạt)
    for i in range(len(pixel_pts) - 1):
        cv2.line(annotated_map, pixel_pts[i], pixel_pts[i+1], (0, 255, 255), 2, cv2.LINE_AA)

    # Vẽ tâm Vùng Trọng Điểm ROI Focus (Màu đỏ chói)
    center_pt = (int(roi_center_x * scale_x), int(roi_center_y * scale_y))
    cv2.circle(annotated_map, center_pt, 12, (0, 0, 255), -1)
    cv2.circle(annotated_map, center_pt, 16, (255, 255, 255), 2)
    cv2.putText(annotated_map, "FOCUS ROI ZONE (AI TARGET)", (center_pt[0] - 80, center_pt[1] - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2, cv2.LINE_AA)

    map_out_path = os.path.join(output_dir, "annotated_spot_focus_map.png")
    cv2.imwrite(map_out_path, annotated_map)
    print(f"[+] Đã xuất Ảnh sơ đồ Orbit Focus: {map_out_path}")

    # 5. Xuất Video MP4 Mô Phỏng Quá Trình Focus
    video_path = os.path.join(output_dir, "flight_spot_focus_simulation.mp4")
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video_writer = cv2.VideoWriter(video_path, fourcc, fps, (img_w, img_h))

    for row in orbit_data:
        frame = annotated_map.copy()
        drone_pos = (int(row["X_m"] * scale_x), int(row["Y_m"] * scale_y))

        # Vẽ vị trí Drone
        cv2.circle(frame, drone_pos, 8, (255, 0, 0), -1)
        cv2.circle(frame, drone_pos, 11, (255, 255, 255), 2)

        # Vẽ tia Laser ngắm trực diện từ Drone vào Tâm ROI Focus
        cv2.line(frame, drone_pos, center_pt, (0, 255, 0), 1, cv2.LINE_AA)

        # Giao diện HUD Overlay thông số
        cv2.rectangle(frame, (10, 10), (390, 120), (0, 0, 0), -1)
        cv2.putText(frame, "MODE: SPOT FOCUS / ORBIT TARGET LOCK", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
        cv2.putText(frame, f"TIME: {row['Time_Sec']:.1f}s / {total_time:.1f}s", (20, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(frame, f"ZOOM: {zoom_factor} | ALTITUDE: {altitude_m:.1f}m", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        cv2.putText(frame, f"AIM: LOCKED ON ROI ({roi_center_x}m, {roi_center_y}m)", (20, 98), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 0), 1)

        video_writer.write(frame)

    video_writer.release()
    print(f"[+] Đã xuất Video MP4 Mô phỏng Spot Focus: {video_path}")

    # 6. SAO CHÉP FILE CODE ĐANG CHẠY VÀO TRONG THƯ MỤC VÀ ĐỔI TÊN THÀNH 'ĐÂY LÀ CODE CHẠY .py'
    try:
        current_script_path = os.path.abspath(__file__)
    except NameError:
        current_script_path = os.path.abspath(sys.argv[0])

    target_code_path = os.path.join(output_dir, "ĐÂY LÀ CODE CHẠY .py")
    
    if os.path.exists(current_script_path):
        shutil.copy(current_script_path, target_code_path)
        print(f"[+] ĐÃ SAO CHÉP CODE CHẠY VÀO THƯ MỤC THÀNH CÔNG: {target_code_path}")
    else:
        # Fallback tự ghi nội dung script nếu chạy ở môi trường đặc biệt
        with open(target_code_path, "w", encoding="utf-8") as f:
            f.write("# Mã nguồn Python lộ trình Spot Focus Orbit Mission\n")

    print(f"\n[HOÀN TẤT TOÀN BỘ] Thư mục kết quả hoàn chỉnh: {output_dir}")

if __name__ == "__main__":
    generate_spot_focus_mission()