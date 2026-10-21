import cv2
import numpy as np
import pandas as pd

# ================= CẤU HÌNH =================
IMAGE_PATH = 'map.jpg'  # Tên ảnh bản đồ của bạn

# 4 Tọa độ GPS bạn vừa cung cấp (Kinh độ, Vĩ độ)
# Lưu ý: cv2.perspectiveTransform cần (Lon, Lat)
gps_corners = np.float32([
    [109.21927800, 13.80447200],  # P1: NW (Tây Bắc)
    [109.21947200, 13.80394400],  # P2: SW (Tây Nam)
    [109.21991700, 13.80408300],  # P3: SE (Đông Nam)
    [109.21972200, 13.80461100]   # P4: NE (Đông Bắc)
])

# ================= XỬ LÝ =================
img = cv2.imread(IMAGE_PATH)
h, w, _ = img.shape

# 4 Góc của bức ảnh (Pixel)
# Thứ tự tương ứng: Trên-Trái (NW), Dưới-Trái (SW), Dưới-Phải (SE), Trên-Phải (NE)
pts_src = np.float32([
    [0, 0],         # NW
    [0, h],         # SW
    [w, h],         # SE
    [w, 0]          # NE
])

# 1. Tạo ma trận chuyển đổi từ Pixel sang GPS
matrix_px_to_gps = cv2.getPerspectiveTransform(pts_src, gps_corners)

# 2. Tạo ma trận chuyển đổi ngược từ GPS sang Pixel (để vẽ map nếu cần)
matrix_gps_to_px = cv2.getPerspectiveTransform(gps_corners, pts_src)

# 3. Tính toán khoảng cách thực tế (mét) của khu vực
# 1 độ vĩ ~ 111320m, 1 độ kinh ~ 111320 * cos(vĩ độ)
lat_center = (13.80447200 + 13.80394400) / 2
m_per_deg_lat = 111320
m_per_deg_lon = 111320 * np.cos(np.radians(lat_center))

# Khoảng cách từ NW đến NE (Chiều rộng)
width_m = (gps_corners[3][0] - gps_corners[0][0]) * m_per_deg_lon
# Khoảng cách từ NW đến SW (Chiều dài)
height_m = (gps_corners[0][1] - gps_corners[1][1]) * m_per_deg_lat

print(f"Kích thước khu vực: {width_m:.1f}m (Ngang) x {height_m:.1f}m (Dọc)")

# 4. Tạo lưới quét (Grid) - Ví dụ cứ 10 mét tạo 1 điểm
step_m = 10 
num_x = int(width_m / step_m)
num_y = int(height_m / step_m)

coords_list = []

# Duyệt qua từng pixel trên lưới
for iy in range(num_y):
    for ix in range(num_x):
        # Tính pixel tương ứng
        px = int((ix / num_x) * w)
        py = int((iy / num_y) * h)
        
        # Chuyển Pixel sang GPS
        pt_in = np.array([[[px, py]]], dtype=np.float32)
        pt_out = cv2.perspectiveTransform(pt_in, matrix_px_to_gps)
        lon, lat = pt_out[0][0]
        
        # Tính tọa độ Bắc/Đông (NED) so với góc NW (Tây Bắc)
        north_m = (lat - gps_corners[0][1]) * m_per_deg_lat * -1 # Hướng Bắc âm vì vĩ độ giảm dần
        east_m = (lon - gps_corners[0][0]) * m_per_deg_lon
        
        # Kiểm tra màu sắc tại điểm đó (Lọc tòa nhà)
        # Chuyển ảnh sang HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pixel_hsv = hsv[py, px]
        
        # Màu cam (Tòa nhà) trong HSV
        lower_orange = np.array([0, 100, 100])
        upper_orange = np.array([25, 255, 255])
        
        is_building = (lower_orange[0] <= pixel_hsv[0] <= upper_orange[0] and 
                       lower_orange[1] <= pixel_hsv[1] <= upper_orange[1])
        
        # Nếu không phải tòa nhà thì thêm vào danh sách an toàn
        if not is_building:
            coords_list.append({
                "pixel_x": px, "pixel_y": py,
                "lat": lat, "lon": lon,
                "north_m": north_m, "east_m": east_m
            })

# Xuất ra CSV
df = pd.DataFrame(coords_list)
df.to_csv("safe_flight_grid.csv", index=False)
print(f"✅ Đã xuất {len(coords_list)} điểm bay an toàn vào file safe_flight_grid.csv")

# Tạo ảnh trực quan hóa các điểm bay (chấm xanh)
vis_img = img.copy()
for item in coords_list:
    cv2.circle(vis_img, (item["pixel_x"], item["pixel_y"]), 2, (0, 255, 0), -1)
cv2.imwrite("grid_visualization.jpg", vis_img)
print("✅ Đã lưu ảnh kiểm tra: grid_visualization.jpg")