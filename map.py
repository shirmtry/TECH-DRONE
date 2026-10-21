# map.py — FULL FIXED
import pandas as pd
import numpy as np
import math
import cv2
import io

# ==========================================================
# CONFIG
# ==========================================================
OUT_CSV = r"C:\Users\dinht\Downloads\261011-kit\route_analyzed.csv"

# ⚠️ Kích thước romap.png — kiểm tra bằng:
#   python -c "import cv2; print(cv2.imread(r'...romap.png').shape)"
IMG_W = 1280
IMG_H = 832

GPS_CORNERS = np.float32([
    [109.21927800, 13.80447200],
    [109.21947200, 13.80394400],
    [109.21991700, 13.80408300],
    [109.21972200, 13.80461100],
])

# ==========================================================
# DATA INLINE
# ==========================================================
CSV_RAW = """Step,Timestamp_UTC7,Pixel_X,Pixel_Y,Altitude,Action
1,2026-10-19 16:34:37.825,687,559,10.0,Hover_Scan
2,2026-10-19 16:34:38.629,654,654,10.0,Fly_Through
3,2026-10-19 16:34:39.557,656,770,10.0,Fly_Through
4,2026-10-19 16:34:39.861,668,806,10.0,Hover_Scan
5,2026-10-19 16:34:40.546,732,749,10.0,Fly_Through
6,2026-10-19 16:34:40.831,751,779,10.0,Fly_Through
7,2026-10-19 16:34:42.848,526,665,10.0,Hover_Scan
8,2026-10-19 16:34:45.421,238,808,10.0,Fly_Through
9,2026-10-19 16:34:46.902,71,728,10.0,Fly_Through
10,2026-10-19 16:34:47.178,77,694,10.0,Hover_Scan
11,2026-10-19 16:34:49.294,174,448,10.0,Fly_Through
12,2026-10-19 16:34:49.847,243,452,10.0,Fly_Through
13,2026-10-19 16:34:51.592,372,276,10.0,Hover_Scan
14,2026-10-19 16:34:52.493,391,165,10.0,Fly_Through
15,2026-10-19 16:34:53.463,275,130,10.0,Fly_Through
16,2026-10-19 16:34:54.245,201,194,10.0,Hover_Scan
17,2026-10-19 16:34:55.116,101,237,10.0,Fly_Through
18,2026-10-19 16:34:58.836,566,239,10.0,Fly_Through
19,2026-10-19 16:34:59.670,611,145,10.0,Hover_Scan
20,2026-10-19 16:35:01.357,818,185,10.0,Fly_Through
21,2026-10-19 16:35:02.300,839,301,10.0,Fly_Through
22,2026-10-19 16:35:02.959,921,309,10.0,Hover_Scan
23,2026-10-19 16:35:03.905,1017,378,10.0,Fly_Through
24,2026-10-19 16:35:04.644,1106,403,10.0,Fly_Through
25,2026-10-19 16:35:05.227,1062,461,10.0,Hover_Scan
26,2026-10-19 16:35:05.881,1073,542,10.0,Fly_Through
27,2026-10-19 16:35:05.889,1073,543,10.0,Fly_Through
28,2026-10-19 16:35:06.325,1122,519,10.0,Hover_Scan
29,2026-10-19 16:35:07.298,1243,531,10.0,Fly_Through
30,2026-10-19 16:35:08.445,1233,674,10.0,Fly_Through
31,2026-10-19 16:35:09.111,1151,689,10.0,Hover_Scan
32,2026-10-19 16:35:10.168,1075,797,10.0,Fly_Through
33,2026-10-19 16:35:11.933,946,618,10.0,Fly_Through
34,2026-10-19 16:35:13.306,840,483,10.0,Hover_Scan
35,2026-10-19 16:35:16.819,1147,169,10.0,Fly_Through
36,2026-10-19 16:35:18.050,1031,68,10.0,Fly_Through
"""

df = pd.read_csv(io.StringIO(CSV_RAW))
df['ts'] = pd.to_datetime(df['Timestamp_UTC7'])
print(f"Loaded {len(df)} waypoints  |  {df['ts'].min()} → {df['ts'].max()}")

# ==========================================================
# PIXEL → GPS
# ==========================================================
M = cv2.getPerspectiveTransform(
    np.float32([[0, 0], [0, IMG_H], [IMG_W, IMG_H], [IMG_W, 0]]),
    GPS_CORNERS,
)

def px2gps(px, py):
    pt = np.array([[[float(px), float(py)]]], dtype=np.float32)
    out = cv2.perspectiveTransform(pt, M)
    return float(out[0][0][1]), float(out[0][0][0])

gps_list = [px2gps(row.Pixel_X, row.Pixel_Y) for row in df.itertuples()]
df['Lat'] = [g[0] for g in gps_list]
df['Lon'] = [g[1] for g in gps_list]

# ==========================================================
# HAVERSINE
# ==========================================================
def hav(lat1, lon1, lat2, lon2):
    R = 6371000
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1)*math.cos(p2)*math.sin(dl/2)**2
    return 2*R*math.asin(math.sqrt(a))

# ==========================================================
# METRICS
# ==========================================================
df['dt_s'] = df['ts'].diff().dt.total_seconds().fillna(0)

dists = [0.0]
for i in range(1, len(df)):
    dists.append(hav(df.Lat[i-1], df.Lon[i-1], df.Lat[i], df.Lon[i]))
df['dist_m']   = dists
df['speed']    = np.where(df['dt_s'] > 0, df['dist_m'] / df['dt_s'], 0)
df['cum_dist'] = df['dist_m'].cumsum()
df['cum_time'] = df['dt_s'].cumsum()

dx = df['Pixel_X'].diff().fillna(0)
dy = df['Pixel_Y'].diff().fillna(0)
df['heading'] = (np.degrees(np.arctan2(dx, -dy)) + 360) % 360
turn = [0.0]
for i in range(1, len(df)):
    d = df.heading[i] - df.heading[i-1]
    if d > 180: d -= 360
    if d < -180: d += 360
    turn.append(d)
df['turn'] = turn

# ==========================================================
# REPORT
# ==========================================================
start, end = df.iloc[0], df.iloc[-1]
total_t = (end.ts - start.ts).total_seconds()
total_d = df.dist_m.sum()
net_d   = hav(start.Lat, start.Lon, end.Lat, end.Lon)
eff     = net_d / total_d * 100 if total_d else 0
avg_v   = total_d / total_t if total_t else 0

print("\n" + "="*62)
print("TỔNG QUAN LỘ TRÌNH")
print("="*62)
print(f"Bắt đầu          : {start.Timestamp_UTC7}")
print(f"Kết thúc         : {end.Timestamp_UTC7}")
print(f"Thời gian        : {total_t:.1f}s")
print(f"Số waypoints     : {len(df)}")
print(f"Độ cao           : {df.Altitude.min():.1f}–{df.Altitude.max():.1f} m")
print(f"Tổng quãng đường : {total_d:.1f} m")
print(f"Dịch chuyển ròng : {net_d:.1f} m")
print(f"Hiệu quả đường đi: {eff:.1f}%  (100% = đường thẳng)")
print(f"Tốc độ TB        : {avg_v:.2f} m/s  ({avg_v*3.6:.1f} km/h)")
print(f"Tốc độ max       : {df.speed.max():.2f} m/s  ({df.speed.max()*3.6:.1f} km/h)")
if (df.speed > 0).any():
    print(f"Tốc độ min (bay) : {df.speed[df.speed>0].min():.2f} m/s")

hov = df[df.Action == 'Hover_Scan']
fly = df[df.Action == 'Fly_Through']
print(f"\nHover_Scan       : {len(hov)} điểm")
print(f"Fly_Through      : {len(fly)} điểm")

n90  = int((df.turn.abs() > 90).sum())
n135 = int((df.turn.abs() > 135).sum())
print(f"Cua gắt  (>90°)  : {n90}")
print(f"Quay đầu (>135°): {n135}")

# ==========================================================
# COVERAGE
# ==========================================================
print("\n" + "="*62)
print("COVERAGE")
print("="*62)
print(f"X_px  : {df.Pixel_X.min():.0f} – {df.Pixel_X.max():.0f}  "
      f"({df.Pixel_X.max()-df.Pixel_X.min():.0f} px)")
print(f"Y_px  : {df.Pixel_Y.min():.0f} – {df.Pixel_Y.max():.0f}  "
      f"({df.Pixel_Y.max()-df.Pixel_Y.min():.0f} px)")
print(f"Lat   : {df.Lat.min():.6f} – {df.Lat.max():.6f}")
print(f"Lon   : {df.Lon.min():.6f} – {df.Lon.max():.6f}")

bbox_pct = ((df.Pixel_X.max()-df.Pixel_X.min())/IMG_W
            * (df.Pixel_Y.max()-df.Pixel_Y.min())/IMG_H * 100)
print(f"Bounding box chiếm : {bbox_pct:.1f}% diện tích map")

# ==========================================================
# QUADRANT
# ==========================================================
mid_x, mid_y = IMG_W/2, IMG_H/2
def quad(px, py):
    ns = "T" if py < mid_y else "B"
    ew = "L" if px < mid_x else "R"
    return f"{ns}{ew}"
df['Quad'] = [quad(r.Pixel_X, r.Pixel_Y) for r in df.itertuples()]

print("\nSố waypoint mỗi góc:")
print(df.Quad.value_counts().to_string())

# ==========================================================
# HOVER SEQUENCE
# ==========================================================
print("\n" + "="*62)
print("THỨ TỰ HOVER_SCAN (điểm quét)")
print("="*62)
for _, row in hov.iterrows():
    step  = int(row['Step'])
    px    = int(row['Pixel_X'])
    py    = int(row['Pixel_Y'])
    q     = row['Quad']
    lat   = row['Lat']
    lon   = row['Lon']
    print(f"  Step {step:2d}  ({px:4d},{py:3d})  quad={q}  "
          f"lat={lat:.6f} lon={lon:.6f}")

# ==========================================================
# TOP SEGMENTS
# ==========================================================
print("\n" + "="*62)
print("5 SEGMENT DÀI NHẤT (có thể là 'jump' bất thường)")
print("="*62)
for _, row in df.nlargest(5, 'dist_m').iterrows():
    print(f"  Step {int(row['Step']):2d}: "
          f"{row['dist_m']:5.1f}m in {row['dt_s']:4.2f}s "
          f"→ {row['speed']:5.2f} m/s")

# ==========================================================
# PATTERN DETECTION
# ==========================================================
print("\n" + "="*62)
print("NHẬN DIỆN PATTERN")
print("="*62)

hq = list(hov['Quad'])
print(f"Thứ tự góc của Hover: {' → '.join(hq)}")

if len(set(hq)) == 4:
    print("✓ Đã thăm ĐỦ 4 góc (quadrant coverage đầy đủ)")
else:
    print(f"⚠ Chỉ thăm {len(set(hq))}/4 góc")

cx, cy = IMG_W/2, IMG_H/2
radial = np.hypot(df.Pixel_X - cx, df.Pixel_Y - cy)
corr = np.corrcoef(np.arange(len(df)), radial)[0, 1]
if   corr < -0.35: print(f"✓ SPIRAL VÀO (r = {corr:.2f})")
elif corr >  0.35: print(f"✓ SPIRAL RA  (r = {corr:.2f})")
else:              print(f"~ Không rõ spiral (r = {corr:.2f})")

if eff < 20:
    print(f"✓ LOOP kín (hiệu quả {eff:.1f}% < 20%)")
elif eff > 70:
    print(f"✓ GẦN NHƯ ĐƯỜNG THẲNG (hiệu quả {eff:.1f}% > 70%)")
else:
    print(f"~ Pattern lai (hiệu quả {eff:.1f}%)")

sig = np.sign(dx.values)
flips = int((np.diff(sig) != 0).sum())
print(f"Số lần đổi hướng ngang: {flips}")
if flips >= 4 and eff < 40:
    print("  → Có dấu hiệu boustrophedon (zig-zag)")

# ==========================================================
# SAVE
# ==========================================================
cols = ['Step','Timestamp_UTC7','Pixel_X','Pixel_Y','Lat','Lon',
        'Altitude','Action','dt_s','dist_m','cum_dist',
        'cum_time','speed','heading','turn','Quad']
df[cols].to_csv(OUT_CSV, index=False, float_format="%.7f")
print(f"\n✓ Đã lưu CSV phân tích: {OUT_CSV}")

# ==========================================================
# PLOT
# ==========================================================
try:
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(17, 8))

    # ---- Plot 1: path ----
    ax = axes[0]
    for act, col in [('Fly_Through', 'tab:blue'), ('Hover_Scan', 'tab:red')]:
        sub = df[df.Action == act]
        ax.plot(sub.Pixel_X, sub.Pixel_Y, 'o-', color=col,
                label=act, markersize=7, lw=1.4)
    for _, row in df.iterrows():
        ax.annotate(int(row['Step']),
                    (row['Pixel_X'], row['Pixel_Y']),
                    fontsize=7, alpha=0.7,
                    xytext=(4, 4), textcoords='offset points')
    ax.plot(start.Pixel_X, start.Pixel_Y, 'g^', ms=15, label='START')
    ax.plot(end.Pixel_X,   end.Pixel_Y,   'ks', ms=12, label='END')
    ax.axhline(mid_y, color='gray', ls='--', alpha=0.4)
    ax.axvline(mid_x, color='gray', ls='--', alpha=0.4)
    ax.set_xlim(0, IMG_W); ax.set_ylim(IMG_H, 0)
    ax.set_xlabel("X_px"); ax.set_ylabel("Y_px")
    ax.set_title(f"Quỹ đạo (px) — {len(df)} waypoints, "
                 f"{total_d:.0f}m, {total_t:.0f}s")
    ax.grid(True, alpha=0.3); ax.legend(loc='upper right')

    # ---- Plot 2: speed profile ----
    ax = axes[1]
    ax.plot(df.cum_time, df.speed, 'b-o', markersize=4, lw=1.3)
    ax.axhline(avg_v, color='r', ls='--', label=f'TB {avg_v:.2f} m/s')
    ax.fill_between(df.cum_time, 0, df.speed, alpha=0.15, color='blue')
    ax.set_xlabel("Thời gian (s)")
    ax.set_ylabel("Tốc độ (m/s)")
    ax.set_title("Speed profile (GPS-based)")
    ax.grid(True, alpha=0.3); ax.legend()

    plt.tight_layout()
    png = OUT_CSV.replace('.csv', '_plot.png')
    plt.savefig(png, dpi=110, bbox_inches='tight')
    print(f"✓ Plot: {png}")

except ImportError:
    print("(matplotlib chưa cài — bỏ qua plot)")
    print("  Cài bằng: pip install matplotlib")