# ==========================================================
# CELL: Phân tích lộ trình bay từ video .mp4 (Colab / Jupyter)
# - Phát hiện vật thể bay chuyển động (drone, chim, máy bay, bóng...)
# - Lọc nhiễu bằng Kalman, vẽ quỹ đạo lên video, xuất CSV + biểu đồ
# Giả định: camera đứng yên (nếu camera rung/di chuyển, kết quả sẽ kém)
# ==========================================================
import os, sys, subprocess
try:
    import cv2
except ImportError:
    subprocess.check_call([sys.executable, "-m", "pip", "install", "-q", "opencv-python-headless"])
    import cv2

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------- CẤU HÌNH ----------------
OUT_VIDEO    = "flight_tracked.mp4"
OUT_CSV      = "flight_path.csv"
OUT_PLOT     = "flight_path.png"
MIN_AREA     = 30        # diện tích contour tối thiểu (px^2) - vật nhỏ thì giảm xuống
MAX_AREA     = 20000     # diện tích tối đa, loại bỏ thay đổi sáng toàn khung
HISTORY      = 300       # số frame học nền
VAR_THRESH   = 25        # độ nhạy nền (nhỏ = nhạy hơn)
TRAIL_LEN    = 200       # số điểm đuôi quỹ đạo hiển thị
PIXELS_PER_METER = None  # ví dụ 50.0 nếu biết tỉ lệ -> tốc độ tính bằng m/s
MAX_JUMP     = 150       # px, bỏ qua phát hiện nhảy quá xa so với dự đoán
# ------------------------------------------

# --- Chọn / tải video lên (không cần gõ tên file) ---
def pick_video():
    # 1) Google Colab: hiện nút "Choose Files" để tải video lên
    try:
        from google.colab import files
        print("👉 Bấm 'Choose Files' bên dưới và chọn video .mp4 của bạn...")
        up = files.upload()
        if not up:
            raise SystemExit("Bạn chưa chọn video nào.")
        return next(iter(up.keys()))
    except ImportError:
        pass
    # 2) Jupyter / Python chạy trên máy của bạn: mở hộp thoại chọn file
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk(); root.withdraw(); root.attributes("-topmost", True)
        path = filedialog.askopenfilename(
            title="Chọn video", filetypes=[("Video", "*.mp4 *.mov *.avi *.mkv"), ("Tất cả", "*.*")])
        root.destroy()
        if not path:
            raise SystemExit("Bạn chưa chọn video nào.")
        return path
    except Exception as e:
        raise SystemExit(f"Không mở được hộp chọn file ({e}). Hãy chạy trên Colab.")

VIDEO_PATH = pick_video()
print("Đã chọn:", VIDEO_PATH)

cap = cv2.VideoCapture(VIDEO_PATH)
assert cap.isOpened(), f"Không mở được video: {VIDEO_PATH}"
fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
N = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
print(f"Video: {W}x{H}, {fps:.1f} fps, ~{N} frame")

writer = cv2.VideoWriter(OUT_VIDEO, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
bg = cv2.createBackgroundSubtractorMOG2(history=HISTORY, varThreshold=VAR_THRESH, detectShadows=False)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# --- Kalman filter (trạng thái: x, y, vx, vy) ---
kf = cv2.KalmanFilter(4, 2)
kf.transitionMatrix = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
kf.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
kf_init = False

rows, trail = [], []
frame_idx = 0
WARMUP = 15  # bỏ qua vài frame đầu để nền ổn định

while True:
    ok, frame = cap.read()
    if not ok:
        break

    mask = bg.apply(frame)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    det = None
    if frame_idx >= WARMUP:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cands = []
        for c in cnts:
            a = cv2.contourArea(c)
            if MIN_AREA <= a <= MAX_AREA:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cands.append((a, M["m10"]/M["m00"], M["m01"]/M["m00"], c))
        if cands:
            if kf_init:
                pred = kf.predict()[:2].ravel()
                # chọn ứng viên gần vị trí dự đoán nhất
                best = min(cands, key=lambda t: (t[1]-pred[0])**2 + (t[2]-pred[1])**2)
                if np.hypot(best[1]-pred[0], best[2]-pred[1]) <= MAX_JUMP:
                    det = best
            else:
                det = max(cands, key=lambda t: t[0])  # lấy vật lớn nhất làm điểm khởi tạo

    if det is not None:
        a, cx, cy, c = det
        meas = np.array([[cx], [cy]], np.float32)
        if not kf_init:
            kf.statePost = np.array([[cx], [cy], [0], [0]], np.float32)
            kf_init = True
        else:
            kf.correct(meas)
        st = kf.statePost.ravel()
        x, y, vx, vy = float(st[0]), float(st[1]), float(st[2]), float(st[3])
        rows.append(dict(frame=frame_idx, t=frame_idx/fps, x=x, y=y, vx=vx*fps, vy=vy*fps, area=a, detected=True))
        trail.append((int(x), int(y)))
        bx, by, bw, bh = cv2.boundingRect(c)
        cv2.rectangle(frame, (bx, by), (bx+bw, by+bh), (0, 255, 0), 2)
    elif kf_init:
        # mất dấu: dùng dự đoán Kalman
        st = kf.predict().ravel()
        x, y = float(st[0]), float(st[1])
        rows.append(dict(frame=frame_idx, t=frame_idx/fps, x=x, y=y, vx=float(st[2])*fps, vy=float(st[3])*fps, area=np.nan, detected=False))
        trail.append((int(x), int(y)))

    # vẽ đuôi quỹ đạo
    pts = trail[-TRAIL_LEN:]
    for i in range(1, len(pts)):
        cv2.line(frame, pts[i-1], pts[i], (0, 0, 255), 2)
    cv2.putText(frame, f"frame {frame_idx}  t={frame_idx/fps:.2f}s", (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    writer.write(frame)
    frame_idx += 1

cap.release(); writer.release()

# --- Tổng hợp số liệu ---
df = pd.DataFrame(rows)
if df.empty:
    raise SystemExit("Không phát hiện được vật thể. Thử giảm MIN_AREA / VAR_THRESH.")

scale = (1.0 / PIXELS_PER_METER) if PIXELS_PER_METER else 1.0
unit = "m" if PIXELS_PER_METER else "px"
df["speed"] = np.hypot(df.vx, df.vy) * scale                      # đơn vị/giây
df["heading_deg"] = (np.degrees(np.arctan2(-df.vy, df.vx))) % 360  # 0°=sang phải, 90°=lên
df["dist"] = np.hypot(df.x.diff(), df.y.diff()).fillna(0) * scale
total = df.dist.sum()
df.to_csv(OUT_CSV, index=False)

print("\n===== KẾT QUẢ =====")
print(f"Số frame có quỹ đạo : {len(df)} (phát hiện thật: {int(df.detected.sum())})")
print(f"Thời gian theo dõi  : {df.t.iloc[-1]-df.t.iloc[0]:.2f} s")
print(f"Quãng đường         : {total:.1f} {unit}")
print(f"Tốc độ TB / tối đa  : {df.speed.mean():.1f} / {df.speed.max():.1f} {unit}/s")

# --- Biểu đồ ---
fig, ax = plt.subplots(1, 3, figsize=(18, 5))
sc = ax[0].scatter(df.x, df.y, c=df.t, cmap="viridis", s=8)
ax[0].plot(df.x, df.y, alpha=0.3)
ax[0].invert_yaxis(); ax[0].set_aspect("equal")
ax[0].set_xlim(0, W); ax[0].set_ylim(H, 0)
ax[0].set_title("Quỹ đạo (màu = thời gian)"); plt.colorbar(sc, ax=ax[0], label="t (s)")
ax[1].plot(df.t, df.speed); ax[1].set_title(f"Tốc độ ({unit}/s)"); ax[1].set_xlabel("t (s)"); ax[1].grid(alpha=.3)
ax[2].plot(df.t, df.heading_deg); ax[2].set_title("Hướng bay (độ)"); ax[2].set_xlabel("t (s)"); ax[2].grid(alpha=.3)
plt.tight_layout(); plt.savefig(OUT_PLOT, dpi=150); plt.show()

# --- Tải kết quả về (Colab) ---
try:
    from google.colab import files
    for f in (OUT_VIDEO, OUT_CSV, OUT_PLOT):
        files.download(f)
except ImportError:
    print(f"Đã lưu: {OUT_VIDEO}, {OUT_CSV}, {OUT_PLOT}")