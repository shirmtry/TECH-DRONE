#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drone_planner.py - Lập đường bay drone THEO TỪNG GIỜ (UTC+7) cho 7/11 -> 17/11/2026

Mỗi khung giờ (vd 06h-07h) script tự:
  1. Tính vị trí mặt trời (3 mốc trong giờ), lấy gió/mây từ weather_hourly.csv
  2. Dựng bản đồ độ cao (DSM) từ depth_map.png -> bóng râm + góc khuất (line-of-sight)
  3. Thử nhiều đường bay (hướng dải, độ cao, góc gimbal), mô phỏng "% đã quét"
     như trong game nhưng nhìn từ trên cao (cây che khuất, chói, bóng tối, nhòe)
  4. Ghi folder  flights/YYYY-MM-DD_HHMM-HHMM_UTC+7/  gồm:
       flight_plan.csv   (GPS X_lon / Y_lat / Z_alt_m, góc bay, gimbal, % đã quét)
       conditions.json   (mặt trời, gió, mây, kết quả, cảnh báo, 4 góc GPS)
       coverage_map.png  (bản đồ vùng quét / khuất / chói / tối + đường bay)
       candidates.csv    (mọi phương án đã thử và điểm)
  5. Ghi flights_index.csv, correlation.csv, heatmap_scanned.png ở thư mục gốc

Chạy:   python drone_planner.py
        python drone_planner.py --fetch-weather      (lấy dự báo Open-Meteo, cần mạng)
        python drone_planner.py --cloud 0            (ép trời quang: kiểm tra chói xấu nhất)
        python drone_planner.py --only "2026-11-09 08:00"
Chỉ cần numpy, pandas, matplotlib.
"""
import argparse
import glob
import json
import math
import os
import urllib.parse
import urllib.request
from datetime import date

import matplotlib
import numpy as np
import pandas as pd

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

# ============================== CẤU HÌNH ==============================
HERE = os.path.dirname(os.path.abspath(__file__))
START_DATE, END_DATE = date(2026, 11, 7), date(2026, 11, 17)
FIRST_HOUR, LAST_HOUR = 6, 16          # khung 06-07h ... 16-17h (giờ UTC+7)
TZ = 7
FLIGHT_CSV = "Flight_Plan_20261019_163437.csv"   # plan cũ -> làm mốc so sánh
WEATHER_CSV = "weather_hourly.csv"
OUT_DIR = "flights"

# 4 góc bản đồ (độ-phút-giây) đúng như trên ảnh map. Điểm gốc chuẩn = TL.
def dms(d, m, s):
    return d + m / 60.0 + s / 3600.0


CORNERS = {                                   # (lat, lon)
    "TL": (dms(13, 48, 14.34), dms(109, 13, 10.42)),
    "TR": (dms(13, 48, 15.82), dms(109, 13, 9.86)),
    "BL": (dms(13, 48, 14.67), dms(109, 13, 11.38)),
    "BR": (dms(13, 48, 16.16), dms(109, 13, 10.86)),
}

# Bản đồ độ cao: sáng = cao (cây). Chiều cao thật chưa có -> GIẢ ĐỊNH, chỉnh tại đây.
HEIGHT_MAX = 8.0          # m, chiều cao ứng với pixel sáng nhất (tán cây)
GROUND_PCT = 40           # phần trăm độ sáng coi là mặt đất
DEPTH_INVERT = False      # True nếu depth map sáng = thấp
GRID_W, GRID_H = 80, 51   # lưới mô phỏng (~0.6 m/ô)
TARGET_H = 0.5            # m, độ cao vật cần phát hiện trên mặt đất (người/vật nằm dưới tán)
CANOPY_MIN = 1.5          # m, cao hơn mức này mới coi là cây/vật cản (thấp hơn = cỏ, nhìn xuyên)
CROWN_BASE = 0.35         # tán cây chiếm từ 35% đến 100% chiều cao; dưới đó là gầm thân, nhìn xuyên được

# Camera (GIẢ ĐỊNH - thay bằng thông số thật)
HFOV, VFOV = 82.0, 52.0   # độ
IMG_PX_W = 4000           # số pixel ngang của ảnh chụp
RANGE_MAX = 60.0          # m, xa hơn coi như không đủ nét

# Ứng viên đường bay
ALT_NADIR = (12.0, 18.0)          # m AGL, camera nhìn thẳng xuống
ALT_OBLIQUE = 15.0                # m AGL, camera nghiêng
PITCH_OBLIQUE = (-60, -45, -30)
SIDE_OVERLAP, FWD_OVERLAP = 0.35, 0.60

# Drone / gió
V_CRUISE = 6.0            # m/s tối đa khi quét
VA_MAX = 12.0             # m/s tốc độ so với không khí tối đa
WIND_LIMIT = 10.0         # m/s gió trung bình tối đa
GUST_LIMIT = 12.0         # m/s giật tối đa
BATTERY_MIN = 15.0        # phút bay dùng được
GEOFENCE_M = 12.0         # m, drone không bay xa hơn mức này ngoài ranh giới bản đồ
HOVER_S = 1.0             # giây đứng yên tại điểm Hover_Scan

# Ánh sáng / chói / nhòe
GLINT_DEG = 15.0          # tia phản chiếu gương gần hướng camera < góc này = chói
FLARE_MARGIN = 10.0       # mặt trời nằm trong FOV + biên này = frame bị lóa
DIRECT_MIN = 0.30         # nắng trực tiếp tối thiểu mới tính chói/bóng
SHUTTER_REF = 1500.0      # 1/1500 s khi nắng quang giữa trưa
ISO_GAIN = 2.0            # cho phép tăng ISO x2 để giữ tốc độ màn trập nhanh
BLUR_PX_MAX = 1.0         # nhòe tối đa cho phép (pixel)
E_DARK = 0.03             # độ sáng tương đối dưới mức này = quá tối
E_LOWLIGHT = 0.10         # cảnh báo thiếu sáng
MIN_SUN_EL = 3.0          # mặt trời thấp hơn mức này (giữa khung giờ) -> bỏ qua

# Thời tiết mặc định (KHÔNG phải dự báo) nếu chưa có weather_hourly.csv
CLOUD_DEFAULT, WIND_DEFAULT, WIND_FROM_DEFAULT, GUST_DEFAULT = 60.0, 4.5, 45.0, 7.0
# =====================================================================

# ---------------- ảnh / DSM ----------------
def find_depth_map():
    for pat in ("depth_map.png", "*depth*map*.png", "*depth*.png"):
        hits = sorted(glob.glob(os.path.join(HERE, pat)))
        if hits:
            return hits[0]
    return None


def load_depth():
    path = find_depth_map()
    if path is None:
        print("! Không thấy depth_map.png -> dùng mặt phẳng (không có cây/che khuất)")
        return np.zeros((816, 1279))
    g = plt.imread(path).astype(float)
    if g.ndim == 3:
        g = g[..., :3].mean(axis=2)
    if g.max() > 1.5:
        g = g / 255.0
    return 1.0 - g if DEPTH_INVERT else g


RAW = load_depth()
IMG_H, IMG_W = RAW.shape


def block_mean(a, gh, gw):
    ye = np.linspace(0, a.shape[0], gh + 1).astype(int)
    xe = np.linspace(0, a.shape[1], gw + 1).astype(int)
    s = np.add.reduceat(np.add.reduceat(a, ye[:-1], axis=0), xe[:-1], axis=1)
    return s / np.outer(np.diff(ye), np.diff(xe))


def build_dsm():
    g = block_mean(RAW, GRID_H, GRID_W)
    lo, hi = np.percentile(g, [1, 99])
    gn = np.clip((g - lo) / max(hi - lo, 1e-6), 0, 1)
    ground = np.percentile(gn, GROUND_PCT)
    h = np.clip((gn - ground) / max(1 - ground, 1e-6), 0, 1) * HEIGHT_MAX
    pad = np.pad(h, 1, mode="edge")
    sm = sum(pad[i:i + GRID_H, j:j + GRID_W] for i in range(3) for j in range(3)) / 9.0
    return sm


# ---------------- hình học GPS <-> ENU <-> pixel ----------------
LAT0, LON0 = CORNERS["TL"]
_p = math.radians(LAT0)
M_LAT = 111132.92 - 559.82 * math.cos(2 * _p) + 1.175 * math.cos(4 * _p)
M_LON = 111412.84 * math.cos(_p) - 93.5 * math.cos(3 * _p)


def gps_to_enu(lat, lon):
    return np.array([(lon - LON0) * M_LON, (lat - LAT0) * M_LAT])


def enu_to_gps(e, n):
    return LAT0 + n / M_LAT, LON0 + e / M_LON


CE = {k: gps_to_enu(*v) for k, v in CORNERS.items()}
TL, TR, BR, BL = CE["TL"], CE["TR"], CE["BR"], CE["BL"]
# affine pixel->ENU khớp bình phương tối thiểu trên 4 góc
_px = np.array([[0, 0], [IMG_W, 0], [IMG_W, IMG_H], [0, IMG_H]], float)
_A = np.column_stack([np.ones(4), _px])
_coef, *_ = np.linalg.lstsq(_A, np.array([TL, TR, BR, BL]), rcond=None)
ORG = _coef[0]
AX, AY = _coef[1], _coef[2]                   # m/px theo x ảnh, y ảnh
MINV = np.linalg.inv(np.column_stack([AX, AY]))


def bearing(vec):
    return float(np.degrees(np.arctan2(vec[0], vec[1])) % 360)


def bvec(b):
    r = math.radians(b)
    return np.array([math.sin(r), math.cos(r)])


GEO = dict(width_m=float(np.linalg.norm(TR - TL)), height_m=float(np.linalg.norm(BL - TL)),
           right_bearing=bearing(TR - TL), down_bearing=bearing(BL - TL))


def px_to_enu(px, py):
    px, py = np.asarray(px, float), np.asarray(py, float)
    return ORG + px[..., None] * AX + py[..., None] * AY


def enu_to_px(e, n):
    d = np.stack([np.asarray(e) - ORG[0], np.asarray(n) - ORG[1]], -1)
    q = d @ MINV.T
    return q[..., 0], q[..., 1]


# ---------------- lưới mô phỏng ----------------
DSM = build_dsm()
NC = GRID_W * GRID_H
_gi, _gj = np.meshgrid(np.arange(GRID_W), np.arange(GRID_H))
_cxy = px_to_enu((_gi.ravel() + 0.5) * IMG_W / GRID_W, (_gj.ravel() + 0.5) * IMG_H / GRID_H)
CX, CY, HC = _cxy[:, 0], _cxy[:, 1], DSM.ravel()
C3 = np.column_stack([CX, CY, np.full(NC, TARGET_H)])   # mục tiêu nằm trên MẶT ĐẤT
TANH, TANV = math.tan(math.radians(HFOV / 2)), math.tan(math.radians(VFOV / 2))


def enu_to_grid(x, y):
    px, py = enu_to_px(x, y)
    ix = np.floor(px * GRID_W / IMG_W).astype(int)
    iy = np.floor(py * GRID_H / IMG_H).astype(int)
    inside = (ix >= 0) & (ix < GRID_W) & (iy >= 0) & (iy < GRID_H)
    return np.clip(ix, 0, GRID_W - 1), np.clip(iy, 0, GRID_H - 1), inside


# ---------------- mặt trời (NOAA) ----------------
def solar_position(lat, lon, times_utc):
    t = pd.DatetimeIndex(times_utc)
    T = (np.asarray(t.to_julian_date()) - 2451545.0) / 36525.0
    L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360
    M = 357.52911 + T * (35999.05029 - 0.0001537 * T)
    e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T)
    Mr = np.radians(M)
    Cc = (np.sin(Mr) * (1.914602 - T * (0.004817 + 0.000014 * T))
          + np.sin(2 * Mr) * (0.019993 - 0.000101 * T) + np.sin(3 * Mr) * 0.000289)
    om = np.radians(125.04 - 1934.136 * T)
    lam = L0 + Cc - 0.00569 - 0.00478 * np.sin(om)
    eps0 = 23 + (26 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60) / 60
    eps = eps0 + 0.00256 * np.cos(om)
    decl = np.arcsin(np.sin(np.radians(eps)) * np.sin(np.radians(lam)))
    y = np.tan(np.radians(eps) / 2) ** 2
    L0r = np.radians(L0)
    eqt = 4 * np.degrees(y * np.sin(2 * L0r) - 2 * e * np.sin(Mr)
                         + 4 * e * y * np.sin(Mr) * np.cos(2 * L0r)
                         - 0.5 * y * y * np.sin(4 * L0r) - 1.25 * e * e * np.sin(2 * Mr))
    minutes = t.hour * 60 + t.minute + t.second / 60.0
    ha = np.radians(((minutes + eqt + 4 * lon) % 1440) / 4 - 180)
    la = np.radians(lat)
    sin_el = np.sin(la) * np.sin(decl) + np.cos(la) * np.cos(decl) * np.cos(ha)
    el = np.degrees(np.arcsin(np.clip(sin_el, -1, 1)))
    az = (np.degrees(np.arctan2(np.sin(ha), np.cos(ha) * np.sin(la) - np.tan(decl) * np.cos(la))) + 180) % 360
    return az, el


def sun_at(local_times):
    return solar_position(LAT0, LON0, pd.DatetimeIndex(local_times) - pd.Timedelta(hours=TZ))


def sun_vec(az, el):
    a, e = math.radians(az), math.radians(el)
    return np.array([math.cos(e) * math.sin(a), math.cos(e) * math.cos(a), math.sin(e)])


def illum(el, cloud):
    """(độ sáng tương đối E, tỉ lệ nắng trực tiếp)"""
    s = max(math.sin(math.radians(el)), 0.0)
    return s * (1 - 0.7 * cloud), max(0.0, 1 - cloud) ** 1.5


def crown_hit(h, z):
    """điểm (độ cao z) nằm trong tán cây có chiều cao h"""
    return (h > CANOPY_MIN) & (z >= CROWN_BASE * h) & (z <= h)


def shadow_mask(az, el, ds=0.6, steps=80):
    if el <= 0.5:
        return np.ones(NC, bool)
    sd, tan_el = bvec(az), math.tan(math.radians(el))
    blocked = np.zeros(NC, bool)
    for k in range(1, steps + 1):
        s = k * ds
        ix, iy, inside = enu_to_grid(CX + sd[0] * s, CY + sd[1] * s)
        hh = np.where(inside, DSM[iy, ix], 0.0)
        blocked |= crown_hit(hh, TARGET_H + s * tan_el)
    return blocked


# ---------------- camera, tầm nhìn, che khuất ----------------
def camera_basis(yaw, pitch):
    y, p = np.radians(yaw), np.radians(pitch)
    a = np.stack([np.cos(p) * np.sin(y), np.cos(p) * np.cos(y), np.sin(p)], 1)
    r = np.stack([np.cos(y), -np.sin(y), np.zeros_like(y)], 1)
    u = np.cross(r, a)
    return r, u, a


def los_blocked(Pp, Cc, ns=32):
    out = np.zeros(len(Pp), bool)
    for t in np.linspace(0.06, 0.96, ns):
        pos = Cc + t * (Pp - Cc)
        ix, iy, inside = enu_to_grid(pos[:, 0], pos[:, 1])
        out |= crown_hit(np.where(inside, DSM[iy, ix], 0.0), pos[:, 2])
    return out


def prep_geometry(P, yaw, pitch):
    r, u, a = camera_basis(yaw, pitch)
    V = C3[None, :, :] - P[:, None, :]
    dist = np.linalg.norm(V, axis=2)
    xc, yc, zc = (np.einsum("pcj,pj->pc", V, q) for q in (r, u, a))
    inv = (zc > 1e-6) & (np.abs(xc) <= zc * TANH) & (np.abs(yc) <= zc * TANV) & (dist <= RANGE_MAX)
    pi, ci = np.nonzero(inv)
    Vp, d = V[pi, ci], dist[pi, ci]
    return dict(P=P, r=r, u=u, a=a, pi=pi, ci=ci, n_p=len(P),
                los=~los_blocked(P[pi], C3[ci]),
                to_cam=(-Vp / d[:, None]).astype(np.float32))


STATE_NAMES = ["Đã quét tốt", "Bị khuất (cây che)", "Chói/lóa", "Quá tối (bóng)", "Ngoài tầm camera"]


def evaluate(g, az, el, cloud, shadow, cum=False):
    E, direct = illum(el, cloud)
    ci, pi = g["ci"], g["pi"]
    if direct >= DIRECT_MIN:
        s = sun_vec(az, el)
        spec = np.array([-s[0], -s[1], s[2]], np.float32)
        glint = np.degrees(np.arccos(np.clip(g["to_cam"] @ spec, -1, 1))) < GLINT_DEG
        sr, su, sa = g["r"] @ s, g["u"] @ s, g["a"] @ s
        flare = (sa > 0) & (np.degrees(np.arctan2(np.abs(sr), sa)) < HFOV / 2 + FLARE_MARGIN) \
            & (np.degrees(np.arctan2(np.abs(su), sa)) < VFOV / 2 + FLARE_MARGIN)
        glare_pair = glint | flare[pi]
    else:
        glare_pair = np.zeros(len(pi), bool)
    inview = np.zeros(NC, bool)
    inview[ci] = True
    losany = np.zeros(NC, bool)
    losany[ci[g["los"]]] = True
    clear = g["los"] & ~glare_pair
    clearany = np.zeros(NC, bool)
    clearany[ci[clear]] = True
    lit = E * np.where(shadow, 1 - 0.8 * direct, 1.0) >= E_DARK
    scanned = clearany & lit
    res = dict(scanned=scanned.mean() * 100, outside=(~inview).mean() * 100,
               occluded=(inview & ~losany).mean() * 100, glare=(losany & ~clearany).mean() * 100,
               dark=(clearany & ~lit).mean() * 100, E=E, direct=direct)
    if cum:
        first = np.full(NC, np.inf)
        good = clear & lit[ci]
        np.minimum.at(first, ci[good], pi[good].astype(float))
        cnt = np.bincount(first[np.isfinite(first)].astype(int), minlength=g["n_p"])
        res["cum"] = cnt.cumsum() / NC * 100
        res["state"] = np.select([scanned, ~inview, inview & ~losany, losany & ~clearany, clearany & ~lit],
                                 [0, 4, 1, 2, 3], default=4).reshape(GRID_H, GRID_W)
    return res


# ---------------- sinh đường bay ----------------
def clamp_geofence(P):
    px, py = enu_to_px(P[:, 0], P[:, 1])
    mx, my = GEOFENCE_M / np.linalg.norm(AX), GEOFENCE_M / np.linalg.norm(AY)
    xy = px_to_enu(np.clip(px, -mx, IMG_W + mx), np.clip(py, -my, IMG_H + my))
    return np.column_stack([xy, P[:, 2]])


def make_waypoints(th, alt, pitch, yaw_obl, ymode="fixed"):
    d = bvec(th)
    n = np.array([d[1], -d[0]])
    F = np.array([TL, TR, BR, BL])
    dc, nc = F @ d, F @ n
    d0, d1, n0, n1 = dc.min(), dc.max(), nc.min(), nc.max()
    if pitch == -90:
        Wf, Df = 2 * alt * TANH, 2 * alt * TANV
    else:
        Rc = alt / math.sin(math.radians(-pitch))
        Wf = Df = 2 * Rc * TANV
    ns = max(1, math.ceil((n1 - n0) / (Wf * (1 - SIDE_OVERLAP))))
    lats = n0 + (np.arange(ns) + 0.5) * (n1 - n0) / ns
    m = max(2, math.ceil((d1 - d0) / max(2.0, Df * (1 - FWD_OVERLAP))) + 1)
    P, yaw, act, strip = [], [], [], []
    for k, lat in enumerate(lats):
        ts = np.linspace(d0, d1, m)
        hdg = th
        if k % 2:
            ts, hdg = ts[::-1], (th + 180) % 360
        for i, t in enumerate(ts):
            xy = t * d + lat * n
            if pitch == -90:
                y_ = hdg
            else:
                y_ = yaw_obl if ymode == "fixed" else hdg
                xy = xy - (alt / math.tan(math.radians(-pitch))) * bvec(y_)
            P.append([xy[0], xy[1], alt])
            yaw.append(y_)
            act.append("Hover_Scan" if i in (0, m - 1) else "Fly_Through")
            strip.append(k)
    P = clamp_geofence(np.array(P))
    keep = [0] + [i for i in range(1, len(P)) if np.hypot(*(P[i, :2] - P[i - 1, :2])) > 0.3]
    return dict(P=P[keep], yaw=np.array(yaw, float)[keep], pitch=np.full(len(keep), float(pitch)),
                action=[act[i] for i in keep], strip=np.array(strip)[keep], th=th, alt=alt)


def v_blur_max(alt, pitch, E):
    Rc = alt / math.sin(math.radians(-pitch))
    gsd = Rc * 2 * TANH / IMG_PX_W
    t = float(np.clip((1 / SHUTTER_REF) / (max(E, 0.02) * ISO_GAIN), 1 / 4000, 1 / 100))
    return BLUR_PX_MAX * gsd / t, t, gsd


def schedule(P, wind, wfrom, vcap):
    n = len(P)
    seg = P[1:, :2] - P[:-1, :2]
    L = np.hypot(seg[:, 0], seg[:, 1])
    hd = np.array([bearing(s) for s in seg])
    head = wind * np.cos(np.radians(hd - wfrom))
    cross = wind * np.sin(np.radians(hd - wfrom))
    gs_cap = min(V_CRUISE, vcap)
    gs, va, bad = np.zeros(n - 1), np.zeros(n - 1), 0
    for i in range(n - 1):
        need = math.hypot(gs_cap + head[i], cross[i])
        if need <= VA_MAX:
            gs[i], va[i] = gs_cap, need
        elif VA_MAX ** 2 > cross[i] ** 2:
            gs[i], va[i] = math.sqrt(VA_MAX ** 2 - cross[i] ** 2) - head[i], VA_MAX
        else:
            gs[i], va[i] = 0.0, VA_MAX
        if gs[i] < 0.3:
            bad += 1
            gs[i] = 0.3
    crab = np.degrees(np.arcsin(np.clip(cross / np.maximum(va, 1e-6), -1, 1)))
    return dict(L=L, hd=hd, head=head, cross=cross, gs=gs, crab=crab, bad=bad)


# ---------------- thời tiết ----------------
def weather_table(d0, d1, fetch):
    idx = pd.date_range(f"{d0} 00:00", f"{d1} 23:00", freq="h")
    path = os.path.join(HERE, WEATHER_CSV)
    if os.path.exists(path):
        wx = pd.read_csv(path, parse_dates=["time"]).set_index("time")
    else:
        wx = pd.DataFrame(columns=["cloud_pct", "wind_mps", "wind_from_deg", "gust_mps", "source"])
    wx = wx.reindex(idx.union(wx.index))
    miss = wx["source"].isna()
    wx.loc[miss, ["cloud_pct", "wind_mps", "wind_from_deg", "gust_mps"]] = [
        CLOUD_DEFAULT, WIND_DEFAULT, WIND_FROM_DEFAULT, GUST_DEFAULT]
    wx.loc[miss, "source"] = "climatology"
    if fetch:
        try:
            q = dict(latitude=f"{LAT0:.5f}", longitude=f"{LON0:.5f}", wind_speed_unit="ms",
                     hourly="cloud_cover,wind_speed_10m,wind_direction_10m,wind_gusts_10m",
                     timezone="Asia/Bangkok", start_date=str(d0), end_date=str(d1))
            url = "https://api.open-meteo.com/v1/forecast?" + urllib.parse.urlencode(q)
            with urllib.request.urlopen(url, timeout=30) as r:
                h = json.load(r)["hourly"]
            f = pd.DataFrame({"time": pd.to_datetime(h["time"]), "cloud_pct": h["cloud_cover"],
                              "wind_mps": h["wind_speed_10m"], "wind_from_deg": h["wind_direction_10m"],
                              "gust_mps": h["wind_gusts_10m"]}).dropna().set_index("time")
            n = 0
            for t, row in f.iterrows():
                if t in wx.index and wx.at[t, "source"] != "manual":
                    wx.loc[t, ["cloud_pct", "wind_mps", "wind_from_deg", "gust_mps"]] = row.values
                    wx.at[t, "source"] = "open-meteo"
                    n += 1
            print(f"Open-Meteo: cập nhật {n} giờ")
        except Exception as ex:  # noqa: BLE001
            print(f"! Không lấy được dự báo ({ex}); dùng file/mặc định")
    wx.index.name = "time"
    wx.reset_index().to_csv(path, index=False, float_format="%.1f")
    return wx


def slot_weather(wx, t0, cloud_override):
    a, b = wx.loc[t0], wx.loc[t0 + pd.Timedelta(hours=1)]
    ang = np.radians([a.wind_from_deg, b.wind_from_deg])
    wf = math.degrees(math.atan2(np.sin(ang).mean(), np.cos(ang).mean())) % 360
    cloud = (a.cloud_pct + b.cloud_pct) / 2 if cloud_override is None else cloud_override
    src = a.source if a.source == b.source else f"{a.source}/{b.source}"
    return dict(cloud=float(cloud), wind=float((a.wind_mps + b.wind_mps) / 2), wfrom=float(wf),
                gust=float(max(a.gust_mps, b.gust_mps)), source=src)


# ---------------- 1 khung giờ ----------------
NADIR_CACHE = {}


def candidate_specs(az_mid, wfrom):
    base = [GEO["right_bearing"] % 180, GEO["down_bearing"] % 180, wfrom % 180, (az_mid + 90) % 180]
    thetas = []
    for b in base:
        if all(min(abs(b - t), 180 - abs(b - t)) > 12 for t in thetas):
            thetas.append(b)
    specs = []
    for th in thetas:
        specs += [dict(th=th, alt=a, pitch=-90, ymode="track") for a in ALT_NADIR]
        for p in PITCH_OBLIQUE:
            specs += [dict(th=th, alt=ALT_OBLIQUE, pitch=p, ymode="fixed"),
                      dict(th=th, alt=ALT_OBLIQUE, pitch=p, ymode="track")]
    return specs


def plan_slot(t0, wx_slot, baseline):
    mins = (10, 30, 50)
    stimes = [t0 + pd.Timedelta(minutes=m) for m in mins]
    az, el = sun_at(stimes)
    cloud = wx_slot["cloud"] / 100.0
    shadows = [shadow_mask(az[i], el[i]) for i in range(3)]
    E_min = min(illum(e, cloud)[0] for e in el)
    wind, wfrom = wx_slot["wind"], wx_slot["wfrom"]
    yaw_obl = (az[1] + 180) % 360                      # camera nghiêng quay lưng về mặt trời
    rows, best = [], None
    for spec in candidate_specs(az[1], wfrom):
        th, alt, pitch, ymode = spec["th"], spec["alt"], spec["pitch"], spec["ymode"]
        key = (round(th), alt, pitch, ymode)
        if ymode == "track" and key in NADIR_CACHE:       # hình học không phụ thuộc mặt trời -> dùng lại
            wp, g = NADIR_CACHE[key]
        else:
            wp = make_waypoints(th, alt, pitch, yaw_obl, ymode)
            g = prep_geometry(wp["P"], wp["yaw"], wp["pitch"])
            if ymode == "track":
                NADIR_CACHE[key] = (wp, g)
        rs = [evaluate(g, az[i], el[i], cloud, shadows[i]) for i in range(3)]
        mean = float(np.mean([r["scanned"] for r in rs]))
        worst = float(np.min([r["scanned"] for r in rs]))
        vb, texp, gsd = v_blur_max(alt, pitch, E_min)
        sch = schedule(wp["P"], wind, wfrom, vb)
        n_hover = sum(a == "Hover_Scan" for a in wp["action"])
        dur = float((sch["L"] / sch["gs"]).sum() + n_hover * HOVER_S)
        crab = float(np.abs(sch["crab"]).mean())
        feas = (wind <= WIND_LIMIT and wx_slot["gust"] <= GUST_LIMIT and dur <= BATTERY_MIN * 60
                and sch["bad"] == 0 and el[1] >= MIN_SUN_EL)
        score = (0.5 * mean + 0.5 * worst - 0.3 * max(0.0, dur / 60 - 6) - 0.1 * crab
                 - (0 if feas else 50))
        row = dict(mode="nadir" if pitch == -90 else f"oblique-{ymode}", pitch=pitch, alt=alt,
                   strip_bearing=round(th, 1), waypoints=len(wp["P"]), duration_min=round(dur / 60, 2),
                   speed_cap_mps=round(min(V_CRUISE, vb), 2), shutter=f"1/{1 / texp:.0f}",
                   crab_deg=round(crab, 1), scanned_mean=round(mean, 1), scanned_worst=round(worst, 1),
                   occluded=round(np.mean([r["occluded"] for r in rs]), 1),
                   glare=round(np.mean([r["glare"] for r in rs]), 1),
                   dark=round(np.mean([r["dark"] for r in rs]), 1),
                   outside=round(np.mean([r["outside"] for r in rs]), 1),
                   feasible=feas, score=round(score, 2))
        rows.append(row)
        if best is None or score > best[0]:
            best = (score, row, wp, g, sch, vb, dur, rs)
    base = None
    if baseline is not None:
        rb = [evaluate(baseline, az[i], el[i], cloud, shadows[i]) for i in range(3)]
        base = float(np.mean([r["scanned"] for r in rb]))
    return dict(az=az, el=el, E_min=E_min, cands=pd.DataFrame(rows).sort_values("score", ascending=False),
                best=best, shadows=shadows, base=base, cloud=cloud)


# ---------------- ghi kết quả ----------------
def write_slot(root, t0, wx_slot, res, plot=True):
    score, row, wp, g, sch, vb, dur, rs = res["best"]
    t1 = t0 + pd.Timedelta(hours=1)
    folder = os.path.join(root, f"{t0:%Y-%m-%d_%H%M}-{t1:%H%M}_UTC+7")
    os.makedirs(folder, exist_ok=True)
    P, n = wp["P"], len(wp["P"])
    mid = evaluate(g, res["az"][1], res["el"][1], res["cloud"], res["shadows"][1], cum=True)
    gs = np.append(sch["gs"], sch["gs"][-1])
    hd = np.append(sch["hd"], sch["hd"][-1])
    head, cross, crab = (np.append(x, x[-1]) for x in (sch["head"], sch["cross"], sch["crab"]))
    dt = np.zeros(n)
    for i in range(1, n):
        dt[i] = dt[i - 1] + sch["L"][i - 1] / sch["gs"][i - 1] + (HOVER_S if wp["action"][i - 1] == "Hover_Scan" else 0)
    lat, lon = enu_to_gps(P[:, 0], P[:, 1])
    px, py = enu_to_px(P[:, 0], P[:, 1])
    ts = [t0 + pd.Timedelta(seconds=float(s)) for s in dt]
    df = pd.DataFrame({
        "Step": np.arange(1, n + 1),
        "Timestamp_UTC7": [t.strftime("%Y-%m-%d %H:%M:%S.") + f"{t.microsecond // 1000:03d}" for t in ts],
        "X_lon": np.round(lon, 8), "Y_lat": np.round(lat, 8), "Z_alt_m": P[:, 2],
        "Heading_deg": np.round(hd, 1) + 0.0, "Camera_yaw_deg": np.round(wp["yaw"], 1) + 0.0,
        "Gimbal_pitch_deg": wp["pitch"], "Speed_mps": np.round(gs, 2), "Action": wp["action"],
        "Scanned_pct": np.round(mid["cum"], 2),
        "Pixel_X": np.round(px).astype(int), "Pixel_Y": np.round(py).astype(int), "Altitude": P[:, 2],
        "Wind_head_mps": np.round(head, 2) + 0.0, "Wind_cross_mps": np.round(cross, 2) + 0.0,
        "Crab_deg": np.round(crab, 1) + 0.0})
    df.to_csv(os.path.join(folder, "flight_plan.csv"), index=False)
    res["cands"].to_csv(os.path.join(folder, "candidates.csv"), index=False)

    warn = []
    if wx_slot["wind"] > WIND_LIMIT or wx_slot["gust"] > GUST_LIMIT:
        warn.append("gió/giật vượt giới hạn")
    if res["el"][1] < 10:
        warn.append("mặt trời thấp (<10°): bóng dài, thiếu sáng")
    if res["E_min"] < E_LOWLIGHT:
        warn.append("thiếu sáng: phải bay chậm để không nhòe")
    if dur > BATTERY_MIN * 60:
        warn.append("thời gian bay vượt pin")
    if sch["bad"]:
        warn.append("gió ngược vượt khả năng ở một số chặng")
    cond = dict(
        slot_start_utc7=str(t0), slot_end_utc7=str(t1),
        sun=[dict(time=str(t), azimuth=round(float(a), 1), elevation=round(float(e), 1))
             for t, a, e in zip([t0 + pd.Timedelta(minutes=m) for m in (10, 30, 50)], res["az"], res["el"])],
        weather=dict(cloud_pct=round(wx_slot["cloud"], 1), wind_mps=round(wx_slot["wind"], 1),
                     wind_from_deg=round(wx_slot["wfrom"], 0), gust_mps=round(wx_slot["gust"], 1),
                     source=wx_slot["source"]),
        plan=dict(mode=row["mode"], gimbal_pitch_deg=row["pitch"], altitude_m_AGL=row["alt"],
                  strip_bearing_deg=row["strip_bearing"], waypoints=row["waypoints"],
                  duration_min=row["duration_min"], speed_cap_mps=row["speed_cap_mps"],
                  shutter=row["shutter"], feasible=bool(row["feasible"]), geofence_margin_m=GEOFENCE_M),
        result=dict(scanned_mean_pct=row["scanned_mean"], scanned_worst_pct=row["scanned_worst"],
                    occluded_pct=row["occluded"], glare_pct=row["glare"], dark_pct=row["dark"],
                    outside_pct=row["outside"],
                    baseline_old_plan_scanned_pct=None if res["base"] is None else round(res["base"], 1)),
        warnings=warn,
        coordinates=dict(reference_corner="TL", altitude_reference="AGL so với mặt đất tại điểm cất cánh",
                         corners={k: dict(lat=round(v[0], 8), lon=round(v[1], 8)) for k, v in CORNERS.items()},
                         image_right_bearing_deg=round(GEO["right_bearing"], 1),
                         image_down_bearing_deg=round(GEO["down_bearing"], 1)))
    with open(os.path.join(folder, "conditions.json"), "w", encoding="utf-8") as f:
        json.dump(cond, f, ensure_ascii=False, indent=2)
    if plot:
        plot_slot(os.path.join(folder, "coverage_map.png"), t0, cond, mid, px, py, wp)
    return folder, cond, warn


def plot_slot(path, t0, cond, mid, px, py, wp):
    fig, ax = plt.subplots(figsize=(8.2, 5.4))
    ax.imshow(DSM, cmap="gray", extent=[0, GRID_W, GRID_H, 0], vmin=0, vmax=HEIGHT_MAX)
    cmap = ListedColormap(["#2ca02c", "#d62728", "#ffcc00", "#1f77b4", "#888888"])
    ax.imshow(mid["state"], cmap=cmap, vmin=0, vmax=4, alpha=0.5, extent=[0, GRID_W, GRID_H, 0])
    gx, gy = px * GRID_W / IMG_W, py * GRID_H / IMG_H
    ax.plot(gx, gy, "-", color="white", lw=1.0)
    hov = np.array([a == "Hover_Scan" for a in wp["action"]])
    ax.scatter(gx, gy, s=8, c="black", zorder=3)
    ax.scatter(gx[hov], gy[hov], s=34, facecolors="none", edgecolors="orange", zorder=3)
    m_ = GEOFENCE_M / GEO["height_m"] * GRID_H + 3
    ax.set_xlim(-m_, GRID_W + m_)
    ax.set_ylim(GRID_H + m_, -m_)
    ax.set_xticks([])
    ax.set_yticks([])
    pl, rs, wxx = cond["plan"], cond["result"], cond["weather"]
    ax.set_title(f"{t0:%d/%m/%Y %H:00}-{(t0 + pd.Timedelta(hours=1)):%H:00} UTC+7 | {pl['mode']} gimbal {pl['gimbal_pitch_deg']}° "
                 f"alt {pl['altitude_m_AGL']:.0f} m | quét {rs['scanned_mean_pct']}% (xấu nhất {rs['scanned_worst_pct']}%)",
                 fontsize=8)
    ax.legend(handles=[Patch(color=c, label=n_) for c, n_ in zip(["#2ca02c", "#d62728", "#ffcc00", "#1f77b4", "#888888"],
                                                                 STATE_NAMES)], loc="upper center",
              bbox_to_anchor=(0.5, -0.01), ncol=5, fontsize=7, frameon=False)
    s = cond["sun"][1]
    ax.text(0.01, 0.01, f"mặt trời az {s['azimuth']:.0f}° el {s['elevation']:.0f}° | mây {wxx['cloud_pct']:.0f}% | "
            f"gió {wxx['wind_mps']:.1f} m/s từ {wxx['wind_from_deg']:.0f}°", transform=ax.transAxes, fontsize=7,
            color="yellow", va="bottom", bbox=dict(facecolor="black", alpha=0.5, pad=2))
    plt.tight_layout()
    plt.savefig(path, dpi=80)
    plt.close()


# ---------------- plan cũ làm mốc ----------------
def baseline_geometry():
    path = os.path.join(HERE, FLIGHT_CSV)
    if not os.path.exists(path):
        return None
    df = pd.read_csv(path)
    P = np.column_stack([px_to_enu(df.Pixel_X.values, df.Pixel_Y.values), df.Altitude.values])
    seg = np.diff(P[:, :2], axis=0)
    hd = np.array([bearing(s) if np.hypot(*s) > 0.3 else np.nan for s in seg])
    hd = pd.Series(np.append(hd, hd[-1] if len(hd) else 0)).ffill().bfill().values
    return prep_geometry(P, hd, np.full(len(P), -90.0))


# ---------------- chạy ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", default=str(START_DATE))
    ap.add_argument("--end", default=str(END_DATE))
    ap.add_argument("--fetch-weather", action="store_true")
    ap.add_argument("--cloud", type=float, default=None, help="ép % mây (0 = trời quang)")
    ap.add_argument("--only", default=None, help='chỉ 1 khung giờ, vd "2026-11-09 08:00"')
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--out", default=OUT_DIR)
    a = ap.parse_args()
    d0, d1 = date.fromisoformat(a.start), date.fromisoformat(a.end)
    root = os.path.join(HERE, a.out)
    os.makedirs(root, exist_ok=True)
    print(f"Bản đồ {GEO['width_m']:.1f} x {GEO['height_m']:.1f} m | ảnh: phải={GEO['right_bearing']:.0f}°, "
          f"xuống={GEO['down_bearing']:.0f}° | gốc TL {LAT0:.7f}, {LON0:.7f}")
    wx = weather_table(d0, d1, a.fetch_weather)
    base = baseline_geometry()
    slots = []
    for day in pd.date_range(d0, d1):
        for h in range(FIRST_HOUR, LAST_HOUR + 1):
            t0 = day + pd.Timedelta(hours=h)
            if a.only and t0 != pd.Timestamp(a.only):
                continue
            if sun_at([t0 + pd.Timedelta(minutes=30)])[1][0] >= MIN_SUN_EL:
                slots.append(t0)
    index = []
    for k, t0 in enumerate(slots, 1):
        sl = slot_weather(wx, t0, a.cloud)
        res = plan_slot(t0, sl, base)
        folder, cond, warn = write_slot(root, t0, sl, res, plot=not a.no_plots)
        r, p, w = cond["result"], cond["plan"], cond["weather"]
        index.append(dict(
            folder=os.path.basename(folder), slot_start=str(t0), sun_el_mid=cond["sun"][1]["elevation"],
            sun_az_mid=cond["sun"][1]["azimuth"], cloud_pct=w["cloud_pct"], wind_mps=w["wind_mps"],
            wind_from_deg=w["wind_from_deg"], gust_mps=w["gust_mps"], weather_source=w["source"],
            mode=p["mode"], gimbal_pitch_deg=p["gimbal_pitch_deg"], altitude_m=p["altitude_m_AGL"],
            strip_bearing_deg=p["strip_bearing_deg"], waypoints=p["waypoints"], duration_min=p["duration_min"],
            shutter=p["shutter"], feasible=p["feasible"], scanned_mean_pct=r["scanned_mean_pct"],
            scanned_worst_pct=r["scanned_worst_pct"], occluded_pct=r["occluded_pct"], glare_pct=r["glare_pct"],
            dark_pct=r["dark_pct"], outside_pct=r["outside_pct"],
            old_plan_scanned_pct=r["baseline_old_plan_scanned_pct"], warnings="; ".join(warn)))
        print(f"[{k}/{len(slots)}] {t0:%d/%m %H}h  el {cond['sun'][1]['elevation']:5.1f}°  "
              f"{p['mode']:14s} {p['gimbal_pitch_deg']:>4}° {p['altitude_m_AGL']:>4.0f} m  "
              f"quét {r['scanned_mean_pct']:5.1f}% (cũ {r['baseline_old_plan_scanned_pct']}%)  "
              f"{'OK' if p['feasible'] else 'KHÔNG ĐẠT'}")
    if not index:
        print("Không có khung giờ nào.")
        return
    idx = pd.DataFrame(index)
    idx.to_csv(os.path.join(root, "flights_index.csv"), index=False)
    num = idx[["sun_el_mid", "sun_az_mid", "cloud_pct", "wind_mps", "gust_mps", "duration_min", "scanned_mean_pct",
               "glare_pct", "dark_pct", "occluded_pct"]].astype(float)
    num.corr().round(3).to_csv(os.path.join(root, "correlation.csv"))
    idx["day"] = pd.to_datetime(idx.slot_start).dt.strftime("%d/%m")
    idx["hour"] = pd.to_datetime(idx.slot_start).dt.hour
    pv = idx.pivot(index="day", columns="hour", values="scanned_mean_pct")
    fig, ax = plt.subplots(figsize=(10, 4.5))
    im = ax.imshow(pv.values.astype(float), aspect="auto", cmap="RdYlGn", vmin=0, vmax=100)
    ax.set_xticks(range(len(pv.columns)))
    ax.set_xticklabels([f"{h}-{h + 1}h" for h in pv.columns], fontsize=8)
    ax.set_yticks(range(len(pv.index)))
    ax.set_yticklabels(pv.index, fontsize=8)
    for i in range(pv.shape[0]):
        for j in range(pv.shape[1]):
            v = pv.values[i, j]
            if v == v:
                ax.text(j, i, f"{v:.0f}", ha="center", va="center", fontsize=7)
    plt.colorbar(im, label="% đã quét (trung bình trong giờ)")
    ax.set_title("% bề mặt quét tốt theo ngày/giờ (UTC+7) - đường bay tối ưu từng giờ")
    plt.tight_layout()
    plt.savefig(os.path.join(root, "heatmap_scanned.png"), dpi=110)
    plt.close()
    print(f"\nXong {len(idx)} chuyến bay -> {root}")
    print(f"% quét TB: tối ưu {idx.scanned_mean_pct.mean():.1f}%"
          f" | plan cũ {idx.old_plan_scanned_pct.astype(float).mean():.1f}%"
          f" | chuyến không đạt: {(~idx.feasible).sum()}")


if __name__ == "__main__":
    main()
