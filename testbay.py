
import os
import sys
import math
import shutil
import cv2
import numpy as np
import pandas as pd
from datetime import datetime


def generate_adaptive_sweep(
    map_image_path=r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png",
    map_width_m=31.0,
    map_height_m=48.7,
    altitude_m=2.0,
    speed_m_s=1.2,
    camera_pitch_deg=-20.0,
    lane_spacing_m=2.5,
    margin_m=1.5,
    fps=30,
):
    """
    Adaptive Parallel Sweep:
    - Chọn hướng quét giảm tổng chiều dài đường bay.
    - Quét zic-zac, không thêm vòng chu vi dư thừa.
    - Nối các dải bằng cung U-turn nằm trong vùng khảo sát.
    - Xuất CSV, bản đồ và video HUD để đánh giá trong mô phỏng.

    Chú ý: Không phải file nhiệm vụ DJI có thể nạp trực tiếp.
    """

    if map_width_m <= 0 or map_height_m <= 0:
        raise ValueError("Kích thước bản đồ phải lớn hơn 0.")
    if speed_m_s <= 0 or lane_spacing_m <= 0:
        raise ValueError("Tốc độ và khoảng cách dải phải lớn hơn 0.")
    if margin_m < 0 or 2 * margin_m >= min(map_width_m, map_height_m):
        raise ValueError("Margin không hợp lệ.")
    if altitude_m <= 0:
        raise ValueError("Độ cao mô phỏng phải lớn hơn 0.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"flight_mission_adaptive_sweep_{timestamp}"
    os.makedirs(output_dir, exist_ok=True)

    print(f"[+] Output: {output_dir}")

    # Đọc bản đồ hoặc tạo nền giả để kiểm tra thuật toán.
    if os.path.isfile(map_image_path):
        map_img = cv2.imread(map_image_path)
        if map_img is None:
            raise RuntimeError(f"Không đọc được ảnh: {map_image_path}")
    else:
        print("[!] Không thấy mapthidau.png; dùng nền giả.")
        map_img = np.full((974, 620, 3), 180, dtype=np.uint8)

    img_h, img_w = map_img.shape[:2]
    scale_x = img_w / map_width_m
    scale_y = img_h / map_height_m

    x0, x1 = margin_m, map_width_m - margin_m
    y0, y1 = margin_m, map_height_m - margin_m
    usable_w, usable_h = x1 - x0, y1 - y0

    # Chọn hướng theo tổng chiều dài các dải ước lượng.
    # Ước lượng này chưa bao gồm footprint camera hoặc chướng ngại vật.
    horizontal_count = max(1, math.ceil(usable_h / lane_spacing_m) + 1)
    vertical_count = max(1, math.ceil(usable_w / lane_spacing_m) + 1)

    horizontal_cost = horizontal_count * usable_w
    vertical_cost = vertical_count * usable_h

    horizontal = horizontal_cost <= vertical_cost
    if horizontal:
        lane_count = horizontal_count
        lane_positions = np.linspace(y0, y1, lane_count)
        lane_length = usable_w
        strategy = "HORIZONTAL_PARALLEL_SWEEP"
    else:
        lane_count = vertical_count
        lane_positions = np.linspace(x0, x1, lane_count)
        lane_length = usable_h
        strategy = "VERTICAL_PARALLEL_SWEEP"

    # Khoảng cách thực tế giữa các dải sau khi chia đều.
    actual_spacing = (
        abs(lane_positions[1] - lane_positions[0])
        if len(lane_positions) > 1 else lane_spacing_m
    )

    # U-turn bán nguyệt có bán kính bằng nửa khoảng cách dải.
    # Chừa khoảng trống ở hai đầu để cung quay không vượt biên.
    radius = actual_spacing / 2.0
    points = []

    if lane_count > 1 and min(usable_w, usable_h) <= actual_spacing:
        raise ValueError("Vùng khảo sát quá nhỏ so với khoảng cách dải.")

    if horizontal:
        left = x0 + radius
        right = x1 - radius

        if right <= left:
            left, right = x0, x1
            radius = 0.0

        for lane_i, y in enumerate(lane_positions):
            forward = lane_i % 2 == 0
            start_x, end_x = (left, right) if forward else (right, left)

            if not points:
                points.append((start_x, y))

            # Đường thẳng trên dải hiện tại.
            points.append((end_x, y))

            # Nối sang dải kế tiếp bằng cung nửa đường tròn.
            if lane_i < lane_count - 1:
                next_y = lane_positions[lane_i + 1]

                if radius > 0:
                    # Quay đầu nằm trong biên theo hướng di chuyển.
                    center_x = end_x
                    sign = 1.0 if forward else -1.0
                    angles = np.linspace(-math.pi / 2, math.pi / 2, 24)

                    for theta in angles[1:]:
                        px = center_x + sign * radius * math.cos(theta)
                        py = (y + next_y) / 2 + radius * math.sin(theta)
                        points.append((px, py))
                else:
                    points.append((end_x, next_y))

    else:
        top = y0 + radius
        bottom = y1 - radius

        if bottom <= top:
            top, bottom = y0, y1
            radius = 0.0

        for lane_i, x in enumerate(lane_positions):
            forward = lane_i % 2 == 0
            start_y, end_y = (top, bottom) if forward else (bottom, top)

            if not points:
                points.append((x, start_y))

            points.append((x, end_y))

            if lane_i < lane_count - 1:
                next_x = lane_positions[lane_i + 1]

                if radius > 0:
                    center_y = end_y
                    sign = 1.0 if forward else -1.0
                    angles = np.linspace(-math.pi / 2, math.pi / 2, 24)

                    for theta in angles[1:]:
                        px = (x + next_x) / 2 + radius * math.sin(theta)
                        py = center_y + sign * radius * math.cos(theta)
                        points.append((px, py))
                else:
                    points.append((next_x, end_y))

    # Loại bỏ các điểm liên tiếp trùng nhau.
    clean_points = []
    for p in points:
        if not clean_points or np.linalg.norm(
            np.array(p) - np.array(clean_points[-1])
        ) > 1e-6:
            clean_points.append(p)

    # Nội suy đều theo khoảng cách để tạo dữ liệu quỹ đạo.
    trajectory = []
    cumulative_time = 0.0
    cumulative_distance = 0.0

    for seg_i in range(len(clean_points) - 1):
        p1 = np.array(clean_points[seg_i], dtype=float)
        p2 = np.array(clean_points[seg_i + 1], dtype=float)
        segment_length = float(np.linalg.norm(p2 - p1))

        if segment_length < 1e-8:
            continue

        # Khoảng cách lấy mẫu 0.25 m, phục vụ mô phỏng/trực quan hóa.
        n = max(1, int(math.ceil(segment_length / 0.25)))
        samples = np.linspace(p1, p2, n + 1)

        for sample_i, p in enumerate(samples):
            if seg_i > 0 and sample_i == 0:
                continue

            if trajectory:
                prev = np.array([trajectory[-1]["X_m"], trajectory[-1]["Y_m"]])
                step_dist = float(np.linalg.norm(p - prev))
                cumulative_distance += step_dist
                cumulative_time += step_dist / speed_m_s

            # Yaw quy ước 0 độ theo +Y, 90 độ theo +X.
            if sample_i < len(samples) - 1:
                delta = samples[sample_i + 1] - p
            elif trajectory:
                prev = np.array([trajectory[-1]["X_m"], trajectory[-1]["Y_m"]])
                delta = p - prev
            else:
                delta = np.array([0.0, 1.0])

            yaw = math.degrees(math.atan2(delta[0], delta[1])) % 360

            trajectory.append({
                "Step": len(trajectory) + 1,
                "X_m": round(float(p[0]), 3),
                "Y_m": round(float(p[1]), 3),
                "Altitude_m": altitude_m,
                "Speed_m_s": speed_m_s,
                "Gimbal_Pitch_deg": camera_pitch_deg,
                "Yaw_deg": round(yaw, 1),
                "Distance_m": round(cumulative_distance, 2),
                "Time_Sec": round(cumulative_time, 2),
                "Strategy": strategy,
            })

    if len(trajectory) < 2:
        raise RuntimeError("Không tạo được quỹ đạo hợp lệ.")

    df = pd.DataFrame(trajectory)
    total_time = float(df["Time_Sec"].iloc[-1])
    total_distance = float(df["Distance_m"].iloc[-1])

    csv_path = os.path.join(output_dir, "adaptive_sweep_plan.csv")
    df.to_csv(csv_path, index=False, encoding="utf-8-sig")

    # Chuyển tọa độ mét sang pixel.
    def to_pixel(x, y):
        px = int(round(x * scale_x))
        py = int(round(y * scale_y))
        return (
            max(0, min(img_w - 1, px)),
            max(0, min(img_h - 1, py)),
        )

    map_out = map_img.copy()
    pixel_points = [
        to_pixel(row["X_m"], row["Y_m"])
        for row in trajectory
    ]

    for i in range(len(pixel_points) - 1):
        cv2.line(
            map_out, pixel_points[i], pixel_points[i + 1],
            (0, 220, 0), 2, cv2.LINE_AA
        )

    cv2.circle(map_out, pixel_points[0], 7, (0, 255, 0), -1)
    cv2.circle(map_out, pixel_points[-1], 7, (0, 0, 255), -1)

    cv2.putText(
        map_out, strategy, (15, 25),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2
    )

    map_out_path = os.path.join(output_dir, "adaptive_sweep_map.png")
    cv2.imwrite(map_out_path, map_out)

    # Video HUD.
    video_path = os.path.join(output_dir, "adaptive_sweep_simulation.mp4")
    writer = cv2.VideoWriter(
        video_path,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (img_w, img_h),
    )

    if not writer.isOpened():
        raise RuntimeError("Không mở được MP4 writer. Kiểm tra codec OpenCV.")

    # Chọn tối đa khoảng 900 frame để hạn chế dung lượng.
    skip = max(1, len(trajectory) // 900)

    try:
        for idx in range(0, len(trajectory), skip):
            row = trajectory[idx]
            frame = map_out.copy()
            drone_pos = to_pixel(row["X_m"], row["Y_m"])

            cv2.circle(frame, drone_pos, 8, (255, 0, 0), -1)
            cv2.circle(frame, drone_pos, 12, (0, 255, 255), 2)

            cv2.rectangle(frame, (10, 10), (470, 150), (0, 0, 0), -1)
            lines = [
                f"STRATEGY: {strategy}",
                f"TIME: {row['Time_Sec']:.1f}s / {total_time:.1f}s",
                f"DISTANCE: {row['Distance_m']:.1f} / {total_distance:.1f} m",
                f"SPEED: {speed_m_s:.1f} m/s  ALT: {altitude_m:.1f} m",
                f"POS: X={row['X_m']:.2f} Y={row['Y_m']:.2f} m",
            ]

            for line_i, text in enumerate(lines):
                cv2.putText(
                    frame, text, (20, 35 + line_i * 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (0, 255, 255) if line_i == 0 else (255, 255, 255),
                    1, cv2.LINE_AA
                )

            writer.write(frame)
    finally:
        writer.release()

    # Lưu bản sao code đang chạy.
    script_path = os.path.abspath(sys.argv[0])
    target_script = os.path.join(output_dir, "run_adaptive_sweep.py")

    if os.path.isfile(script_path):
        shutil.copy2(script_path, target_script)

    print("\n[+] Hoàn tất tạo kế hoạch.")
    print(f"    Strategy      : {strategy}")
    print(f"    Lane count    : {lane_count}")
    print(f"    Lane spacing  : {actual_spacing:.2f} m")
    print(f"    Total distance: {total_distance:.2f} m")
    print(f"    Estimated time: {total_time:.2f} s")
    print(f"    CSV           : {csv_path}")
    print(f"    Map           : {map_out_path}")
    print(f"    Video         : {video_path}")
    print(f"[+] Output folder : {output_dir}")


if __name__ == "__main__":
    generate_adaptive_sweep()
