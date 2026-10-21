#!/usr/bin/env python3
"""
Phân tích đường bay drone theo mặt trời (góc chiếu, chói/glint, bóng râm) và gió.

Đầu vào : Flight_Plan_*.csv (Pixel_X/Pixel_Y) + 4 góc GPS của bản đồ
Đầu ra  : thư mục out/ gồm sun_table.csv, windows.csv, flight_legs.csv,
          4 hình PNG và report.md
Chỉ cần numpy, pandas, matplotlib (tự cài công thức vị trí mặt trời NOAA).
"""
import glob
import os
from datetime import date

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# ============================ CẤU HÌNH ============================
GPS_CORNERS = np.float32([          # [lon, lat]
    [109.21927800, 13.80447200],
    [109.21947200, 13.80394400],
    [109.21991700, 13.80408300],
    [109.21972200, 13.80461100],
])
# Ý nghĩa 4 điểm trên theo vị trí trong ảnh. Mặc định: BL, BR, TR, TL
# (cạnh dài 62 m nằm ngang -> khớp ảnh dạng landscape). NẾU SAI, đổi thứ tự này.
CORNER_ORDER = ("BL", "BR", "TR", "TL")
IMG_W, IMG_H = 1279, 816
FLIGHT_CSV = "Flight_Plan_20261019_163437.csv"
TZ = 7                               # UTC+7
DAYS = [date(2026, 11, d) for d in range(7, 13)]
ALT = 10.0                           # m
HFOV, VFOV = 82.0, 52.0              # độ, FOV camera (chỉnh theo camera thật)
PITCHES = [-90, -60, -45]            # góc gimbal: -90 = nhìn thẳng xuống
OBJ_HEIGHTS = [3, 8, 15]             # m, chiều cao vật thể tính bóng
Q_MIN = 25.0                         # độ, ngưỡng "an toàn chói"
EL_MIN = 30.0                        # độ, mặt trời đủ cao (bóng không quá dài)
WIND_FROM_DEG = 45.0                 # hướng GIÓ THỔI TỪ (NE) - GIẢ ĐỊNH, thay bằng dự báo
WIND_SPEED = 4.5                     # m/s - GIẢ ĐỊNH
CRUISE = 6.0                         # m/s tốc độ bay ngang
WIND_LIMIT = 10.0                    # m/s giới hạn gió của drone
ONE_WAY_OBLIQUE = True               # camera nghiêng: yaw gimbal cố định, không đổi chiều theo dải bay
OUT = "out"
# ==================================================================

os.makedirs(OUT, exist_ok=True)
LON0, LAT0 = float(GPS_CORNERS[:, 0].mean()), float(GPS_CORNERS[:, 1].mean())


# ---------- hình học bản đồ ----------
def to_enu(lon, lat):
    return np.array([(lon - LON0) * 111320.0 * np.cos(np.radians(LAT0)),
                     (lat - LAT0) * 110574.0])


C = {n: to_enu(*GPS_CORNERS[i]) for i, n in enumerate(CORNER_ORDER)}
TL, TR, BR, BL = C["TL"], C["TR"], C["BR"], C["BL"]


def px_to_enu(x, y):
    u = (np.asarray(x, float) / IMG_W)[:, None]
    v = (np.asarray(y, float) / IMG_H)[:, None]
    return (1 - u) * (1 - v) * TL + u * (1 - v) * TR + u * v * BR + (1 - u) * v * BL


def bearing(vec):
    return float(np.degrees(np.arctan2(vec[0], vec[1])) % 360)


AX = (TR - TL) / IMG_W               # m / px theo trục x ảnh
AY = (BL - TL) / IMG_H               # m / px theo trục y ảnh (xuống)
W_M, H_M = np.linalg.norm(TR - TL), np.linalg.norm(BL - TL)
GEO = dict(
    width_m=W_M, height_m=H_M, ratio_geo=W_M / H_M, ratio_img=IMG_W / IMG_H,
    right_bearing=bearing(TR - TL), down_bearing=bearing(BL - TL),
    mpp_x=np.linalg.norm(AX), mpp_y=np.linalg.norm(AY),
    area_m2=abs((TR - TL)[0] * (BL - TL)[1] - (TR - TL)[1] * (BL - TL)[0]),
)


def enu_vec_to_px(vec):
    return np.linalg.solve(np.column_stack([AX, AY]), vec)


# ---------- vị trí mặt trời (NOAA) ----------
def solar_position(lat, lon, times_utc):
    t = pd.DatetimeIndex(times_utc)
    jd = np.asarray(t.to_julian_date())
    T = (jd - 2451545.0) / 36525.0
    L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360
    M = 357.52911 + T * (35999.05029 - 0.0001537 * T)
    e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T)
    Mr = np.radians(M)
    Cc = (np.sin(Mr) * (1.914602 - T * (0.004817 + 0.000014 * T))
          + np.sin(2 * Mr) * (0.019993 - 0.000101 * T) + np.sin(3 * Mr) * 0.000289)
    lam = L0 + Cc - 0.00569 - 0.00478 * np.sin(np.radians(125.04 - 1934.136 * T))
    eps0 = 23 + (26 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60) / 60
    eps = eps0 + 0.00256 * np.cos(np.radians(125.04 - 1934.136 * T))
    decl = np.arcsin(np.sin(np.radians(eps)) * np.sin(np.radians(lam)))
    y = np.tan(np.radians(eps) / 2) ** 2
    L0r = np.radians(L0)
    eqt = 4 * np.degrees(y * np.sin(2 * L0r) - 2 * e * np.sin(Mr)
                         + 4 * e * y * np.sin(Mr) * np.cos(2 * L0r)
                         - 0.5 * y * y * np.sin(4 * L0r) - 1.25 * e * e * np.sin(2 * Mr))
    minutes = t.hour * 60 + t.minute + t.second / 60.0
    tst = (minutes + eqt + 4 * lon) % 1440
    ha = np.radians(tst / 4 - 180)
    la = np.radians(lat)
    sin_el = np.sin(la) * np.sin(decl) + np.cos(la) * np.cos(decl) * np.cos(ha)
    el = np.degrees(np.arcsin(np.clip(sin_el, -1, 1)))
    az = (np.degrees(np.arctan2(np.sin(ha), np.cos(ha) * np.sin(la) - np.tan(decl) * np.cos(la))) + 180) % 360
    return az, el


def sun_at(local_times):
    utc = pd.DatetimeIndex(local_times) - pd.Timedelta(hours=TZ)
    return solar_position(LAT0, LON0, utc)


def sun_vec(az, el):
    a, e = np.radians(az), np.radians(el)
    return np.array([np.cos(e) * np.sin(a), np.cos(e) * np.cos(a), np.sin(e)])


# ---------- mô hình camera / chói ----------
def fov_grid(n=9):
    tx = np.tan(np.radians(np.linspace(-HFOV / 2, HFOV / 2, n)))
    ty = np.tan(np.radians(np.linspace(-VFOV / 2, VFOV / 2, n)))
    X, Y = np.meshgrid(tx, ty)
    return X.ravel(), Y.ravel()


TX, TY = fov_grid()


def camera_rays(heading, pitch):
    h, p = np.radians(heading), np.radians(pitch)
    a = np.array([np.cos(p) * np.sin(h), np.cos(p) * np.cos(h), np.sin(p)])
    r = np.array([np.cos(h), -np.sin(h), 0.0])
    u = np.cross(r, a)
    rays = a + TX[:, None] * r + TY[:, None] * u
    return rays / np.linalg.norm(rays, axis=1, keepdims=True)


HEADS = np.arange(0, 360, 10)
RAY = {(p, h): camera_rays(h, p) for p in PITCHES for h in HEADS}


def hazard(rays, az, el):
    """(góc nhỏ nhất tới tia phản xạ gương, góc nhỏ nhất tới mặt trời) - độ."""
    s = sun_vec(az, el)
    spec = np.array([-s[0], -s[1], s[2]])
    down = rays[:, 2] < -1e-6
    g = np.degrees(np.arccos(np.clip(-rays[down] @ spec, -1, 1))).min() if down.any() else 90.0
    sa = np.degrees(np.arccos(np.clip(rays @ s, -1, 1))).min()
    return g, sa


def quality(p, h, az, el):
    h = int(round(h / 10.0) * 10) % 360
    g, sa = hazard(RAY[(p, h)], az, el)
    return min(g, sa)


def axis_dist(a, b):
    d = abs(a - b) % 180
    return min(d, 180 - d)


# ---------- thời gian ----------
def local_range(d, start="00:00", end="23:50", freq="10min"):
    return pd.date_range(f"{d} {start}", f"{d} {end}", freq=freq)


def contiguous(mask):
    out, st = [], None
    for i, m in enumerate(mask):
        if m and st is None:
            st = i
        if st is not None and (not m or i == len(mask) - 1):
            out.append((st, i if m else i - 1))
            st = None
    return out


# ================= 1. KẾ HOẠCH BAY =================
df = pd.read_csv(FLIGHT_CSV, parse_dates=["Timestamp_UTC7"])
xy = px_to_enu(df.Pixel_X, df.Pixel_Y)
d = np.diff(xy, axis=0)
dist = np.hypot(d[:, 0], d[:, 1])
head = np.degrees(np.arctan2(d[:, 0], d[:, 1])) % 360
head = pd.Series(np.where(dist < 0.3, np.nan, head)).ffill().bfill().values
dt = df.Timestamp_UTC7.diff().dt.total_seconds().values[1:]
wp_head = np.append(head, head[-1])

rel = np.radians(head - WIND_FROM_DEG)
legs = pd.DataFrame({
    "from_step": df.Step.values[:-1], "to_step": df.Step.values[1:],
    "action_to": df.Action.values[1:], "dist_m": dist.round(2), "dur_s": dt.round(2),
    "speed_mps": (dist / np.maximum(dt, 1e-3)).round(2), "heading_deg": head.round(1),
    "headwind_mps": (WIND_SPEED * np.cos(rel)).round(2),
    "crosswind_mps": (WIND_SPEED * np.sin(rel)).round(2),
})
legs["crab_deg"] = np.degrees(np.arcsin(np.clip(legs.crosswind_mps / CRUISE, -1, 1))).round(1)
legs["gs_mps"] = (np.sqrt(np.maximum(CRUISE ** 2 - legs.crosswind_mps ** 2, 0)) - legs.headwind_mps).round(2)
flight_t0 = df.Timestamp_UTC7.iloc[0]
az_f, el_f = sun_at([flight_t0])
legs.to_csv(f"{OUT}/flight_legs.csv", index=False)

flight_now = {p: np.array([quality(p, h, az_f[0], el_f[0]) for h in wp_head]) for p in PITCHES}

# ================= 2. QUÉT MẶT TRỜI 7-12/11 =================
rows, Qbest, Qplan, Rplan, TIMES = [], {}, {}, {}, {}
wind_axis = WIND_FROM_DEG % 180
daily = []
for dy in DAYS:
    full = local_range(dy)
    az_all, el_all = sun_at(full)
    day_idx = np.where(el_all > 0)[0]
    sr, ss = full[day_idx[0]], full[day_idx[-1]]
    noon = int(np.argmax(el_all))
    daily.append(dict(date=str(dy), sunrise=sr.strftime("%H:%M"), sunset=ss.strftime("%H:%M"),
                      noon=full[noon].strftime("%H:%M"), noon_el=round(el_all[noon], 1),
                      noon_az=round(az_all[noon], 1)))
    for i, t in enumerate(full):
        el, az = el_all[i], az_all[i]
        row = dict(date=str(dy), time=t.strftime("%H:%M"), az=round(az, 1), el=round(el, 1))
        if el > 0.5:
            row["shadow_per_m"] = round(1 / np.tan(np.radians(el)), 2)
            row["shadow_dir_deg"] = round((az + 180) % 360, 1)
        rows.append(row)

    ft = local_range(dy, "05:30", "18:30")
    az, el = sun_at(ft)
    TIMES[dy] = (ft, az, el)
    for p in PITCHES:
        qb = np.full(len(ft), np.nan)
        bh = np.full(len(ft), np.nan)
        pr = np.full(len(ft), np.nan)
        for i in range(len(ft)):
            if el[i] < 3:
                continue
            if p == -90 or not ONE_WAY_OBLIQUE:
                # camera quay theo hướng bay: hai chiều của dải bay đều phải an toàn
                qs = np.array([min(quality(p, h, az[i], el[i]), quality(p, h + 180, az[i], el[i]))
                               for h in HEADS])
            else:
                # camera nghiêng, yaw gimbal giữ cố định (quay lưng về phía mặt trời)
                qs = np.array([quality(p, h, az[i], el[i]) for h in HEADS])
            ok = np.where(qs >= qs.max() - 3)[0]
            k = ok[np.argmin([axis_dist(HEADS[j], wind_axis) for j in ok])]
            qb[i], bh[i] = qs[k], HEADS[k]
            qp = np.array([quality(p, h, az[i], el[i]) for h in wp_head])
            pr[i] = (qp < Q_MIN).mean()
        Qbest[(dy, p)], Qplan[(dy, p)], Rplan[(dy, p)] = qb, bh, pr

sun_tab = pd.DataFrame(rows)
sun_tab.to_csv(f"{OUT}/sun_table.csv", index=False)

# cửa sổ bay tốt
win_rows = []
for dy in DAYS:
    ft, az, el = TIMES[dy]
    for p in PITCHES:
        mask = (Qbest[(dy, p)] >= Q_MIN) & (el >= EL_MIN)
        for a, b in contiguous(mask):
            m = (a + b) // 2
            win_rows.append(dict(
                date=str(dy), pitch=p, start=ft[a].strftime("%H:%M"), end=ft[b].strftime("%H:%M"),
                el_start=round(el[a], 1), el_end=round(el[b], 1),
                heading_start=int(Qplan[(dy, p)][a]), heading_mid=int(Qplan[(dy, p)][m]),
                heading_end=int(Qplan[(dy, p)][b]),
                min_quality=round(np.nanmin(Qbest[(dy, p)][a:b + 1]), 1)))
wins = pd.DataFrame(win_rows)
wins.to_csv(f"{OUT}/windows.csv", index=False)

# ================= 3. HÌNH =================
colors = plt.cm.viridis(np.linspace(0, 0.9, len(DAYS)))

fig = plt.figure(figsize=(13, 5))
ax = fig.add_subplot(1, 2, 1, projection="polar")
ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
for c, dy in zip(colors, DAYS):
    ft, az, el = TIMES[dy]
    m = el > 0
    ax.plot(np.radians(az[m]), 90 - el[m], color=c, label=str(dy))
ax.set_ylim(0, 90)
ax.set_title("Đường đi mặt trời (tâm = thiên đỉnh)")
ax.legend(fontsize=7, loc="lower left", bbox_to_anchor=(-0.25, -0.1))
ax2 = fig.add_subplot(1, 2, 2)
for c, dy in zip(colors, DAYS):
    ft, az, el = TIMES[dy]
    ax2.plot(ft.hour + ft.minute / 60, el, color=c, label=str(dy))
ax2.axhline(EL_MIN, color="r", ls="--", lw=0.8)
ax2.set_xlabel("Giờ địa phương (UTC+7)")
ax2.set_ylabel("Độ cao mặt trời (°)")
ax2.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(f"{OUT}/fig1_sun_path.png", dpi=130)
plt.close()

fig, ax = plt.subplots(figsize=(9, 4.5))
dy = DAYS[0]
ft, az, el = TIMES[dy]
hrs = ft.hour + ft.minute / 60
for hobj, ls in zip(OBJ_HEIGHTS, [":", "-", "--"]):
    L = np.where(el > 5, hobj / np.tan(np.radians(np.clip(el, 5, 90))), np.nan)
    ax.plot(hrs, L, ls, label=f"vật cao {hobj} m")
ax.set_ylim(0, 40)
ax.set_xlabel("Giờ địa phương")
ax.set_ylabel("Chiều dài bóng (m)")
ax.set_title(f"Chiều dài bóng ngày {dy} (các ngày 8–12/11 gần như trùng, lệch <1 m)")
ax.legend()
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig(f"{OUT}/fig2_shadow.png", dpi=130)
plt.close()

fig, axes = plt.subplots(len(PITCHES), 2, figsize=(14, 3.2 * len(PITCHES)), squeeze=False)
for r, p in enumerate(PITCHES):
    for c, (store, title, cmap, vmin, vmax) in enumerate([
            (Qbest, f"Gimbal {p}°: góc an toàn chói (heading tốt nhất)", "RdYlGn", 0, 60),
            (Rplan, f"Gimbal {p}°: % waypoint của plan hiện tại bị chói", "RdYlGn_r", 0, 1)]):
        M = np.array([store[(dy, p)] for dy in DAYS])
        im = axes[r, c].imshow(M, aspect="auto", cmap=cmap, vmin=vmin, vmax=vmax,
                               extent=[5.5, 18.5, len(DAYS), 0])
        axes[r, c].set_yticks(np.arange(len(DAYS)) + 0.5)
        axes[r, c].set_yticklabels([str(x) for x in DAYS], fontsize=7)
        axes[r, c].set_title(title, fontsize=9)
        axes[r, c].set_xlabel("Giờ địa phương")
        plt.colorbar(im, ax=axes[r, c])
plt.tight_layout()
plt.savefig(f"{OUT}/fig3_heatmap.png", dpi=130)
plt.close()

imgs = [f for f in sorted(glob.glob("*.png") + glob.glob("*.jpg") + glob.glob("*.jpeg"))]
fig, ax = plt.subplots(figsize=(11, 7))
if imgs:
    ax.imshow(plt.imread(imgs[0]), cmap="gray")
else:
    ax.set_xlim(0, IMG_W)
    ax.set_ylim(IMG_H, 0)
    ax.set_facecolor("#dddddd")
ax.plot(df.Pixel_X, df.Pixel_Y, "-", color="tab:blue", lw=1.2)
hov = df[df.Action == "Hover_Scan"]
ax.scatter(df.Pixel_X, df.Pixel_Y, s=14, c="tab:blue")
ax.scatter(hov.Pixel_X, hov.Pixel_Y, s=60, facecolors="none", edgecolors="orange", lw=1.5, label="Hover_Scan")
for _, rw in df.iterrows():
    ax.text(rw.Pixel_X + 6, rw.Pixel_Y - 6, str(rw.Step), color="yellow", fontsize=7)
for tstr, col in [("09:00", "red"), ("15:00", "magenta")]:
    t = pd.Timestamp(f"{DAYS[2]} {tstr}")
    a_, e_ = sun_at([t])
    sh = (np.array([np.sin(np.radians(a_[0] + 180)), np.cos(np.radians(a_[0] + 180))])
          * 8 / np.tan(np.radians(e_[0])))
    pv = enu_vec_to_px(sh)
    ax.annotate("", xy=(IMG_W / 2 + pv[0], IMG_H / 2 + pv[1]), xytext=(IMG_W / 2, IMG_H / 2),
                arrowprops=dict(arrowstyle="->", color=col, lw=2))
    ax.text(IMG_W / 2 + pv[0], IMG_H / 2 + pv[1], f"bóng cây 8 m {tstr}\n(el {e_[0]:.0f}°)", color=col, fontsize=8)
wv = enu_vec_to_px(np.array([-np.sin(np.radians(WIND_FROM_DEG)), -np.cos(np.radians(WIND_FROM_DEG))]) * 8)
ax.annotate("", xy=(150 + wv[0], 120 + wv[1]), xytext=(150, 120), arrowprops=dict(arrowstyle="->", color="cyan", lw=3))
ax.text(150, 150, f"gió {WIND_SPEED} m/s (giả định)", color="cyan", fontsize=8)
ax.legend(loc="lower right")
ax.set_title("Flight plan + hướng bóng (9/11) + gió")
plt.tight_layout()
plt.savefig(f"{OUT}/fig4_map.png", dpi=130)
plt.close()

# ================= 4. BÁO CÁO =================
L = []
L.append("# Phân tích đường bay - mặt trời, chói, bóng râm, gió\n")
L.append("## 1. Hình học bản đồ")
L.append(f"- Kích thước thực: {GEO['width_m']:.1f} m x {GEO['height_m']:.1f} m "
         f"(tỉ lệ {GEO['ratio_geo']:.2f}) | ảnh {IMG_W}x{IMG_H} (tỉ lệ {GEO['ratio_img']:.2f})")
L.append(f"- GSD ~ {GEO['mpp_x'] * 100:.1f} cm/px (ngang), {GEO['mpp_y'] * 100:.1f} cm/px (dọc), diện tích ~ {GEO['area_m2']:.0f} m²")
L.append(f"- Với CORNER_ORDER={CORNER_ORDER}: mép phải ảnh quay về hướng {GEO['right_bearing']:.0f}°, "
         f"mép dưới ảnh quay về {GEO['down_bearing']:.0f}° (0°=Bắc)")
if abs(GEO['ratio_geo'] / GEO['ratio_img'] - 1) > 0.1:
    L.append("- ⚠ Tỉ lệ ảnh và tỉ lệ 4 góc GPS lệch >10%: ảnh có thể bị crop/kéo giãn hoặc thứ tự 4 góc chưa đúng.")
L.append("\n## 2. Mặt trời mỗi ngày (giờ UTC+7)")
L.append("| Ngày | Mọc | Lặn | Giữa trưa | Độ cao | Azimuth |\n|---|---|---|---|---|---|")
for x in daily:
    L.append(f"| {x['date']} | {x['sunrise']} | {x['sunset']} | {x['noon']} | {x['noon_el']}° | {x['noon_az']}° |")
L.append("\n## 3. Cửa sổ bay tốt "
         f"(góc an toàn chói ≥ {Q_MIN}° và mặt trời ≥ {EL_MIN}°; heading đã ưu tiên song song gió)")
for p in PITCHES:
    L.append(f"\n**Gimbal {p}°**\n")
    L.append("| Ngày | Từ | Đến | Heading đầu/giữa/cuối |\n|---|---|---|---|")
    sub = wins[wins.pitch == p]
    for _, w in sub.iterrows():
        L.append(f"| {w.date} | {w.start} | {w.end} | {w.heading_start}° / {w.heading_mid}° / {w.heading_end}° |")
    if sub.empty:
        L.append("| - | không có | | |")
L.append("\n## 4. Plan hiện tại (36 bước)")
L.append(f"- Tổng đường bay {dist.sum():.0f} m trong {df.Timestamp_UTC7.iloc[-1] - flight_t0}, "
         f"tốc độ TB {dist.sum() / dt.sum():.1f} m/s, {int((df.Action == 'Hover_Scan').sum())} điểm Hover_Scan")
L.append(f"- Bay lúc {flight_t0:%d/%m %H:%M}: mặt trời cao {el_f[0]:.1f}°, azimuth {az_f[0]:.0f}° "
         f"-> bóng cây 8 m dài {8 / np.tan(np.radians(max(el_f[0], 1))):.0f} m")
for p in PITCHES:
    L.append(f"- Gimbal {p}°: {(flight_now[p] < Q_MIN).sum()}/{len(wp_head)} waypoint có nguy cơ chói tại thời điểm bay đó")
L.append("\n## 5. Gió (GIẢ ĐỊNH, chưa phải dự báo)")
L.append(f"- Gió {WIND_SPEED} m/s từ {WIND_FROM_DEG:.0f}°; tốc độ bay {CRUISE} m/s; giới hạn drone {WIND_LIMIT} m/s")
L.append(f"- Crosswind TB |{legs.crosswind_mps.abs().mean():.1f}| m/s, crab angle lớn nhất {legs.crab_deg.abs().max():.0f}°, "
         f"headwind lớn nhất {legs.headwind_mps.max():.1f} m/s")
L.append(f"- Trục bay song song gió: {wind_axis:.0f}°/{(wind_axis + 180) % 360:.0f}°")
open(f"{OUT}/report.md", "w", encoding="utf-8").write("\n".join(L))
print("\n".join(L))
print("\nFiles:", sorted(os.listdir(OUT)))
