import open3d as o3d
import numpy as np
import cv2

# 1. Đọc ảnh
print("Đang tải ảnh...")
color_path = r"C:\Users\dinht\Downloads\261011-kit\mapthidau.png"
depth_path = "depth_map.png"

color_raw = o3d.io.read_image(color_path)
depth_raw = o3d.io.read_image(depth_path)

if not color_raw or not depth_raw:
    print("❌ Lỗi: Không đọc được ảnh. Kiểm tra lại đường dẫn!")
    exit()

# Lấy kích thước thực tế của ảnh
width = color_raw.get_width() if hasattr(color_raw, 'get_width') else np.asarray(color_raw).shape[1]
height = color_raw.get_height() if hasattr(color_raw, 'get_height') else np.asarray(color_raw).shape[0]
print(f"Kích thước ảnh: {width} x {height}")

# 2. Tạo RGBD Image
# Sử dụng thông số Camera tự động dựa trên kích thước ảnh
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color_raw, depth_raw, depth_scale=1000.0, convert_rgb_to_intensity=False)

# 3. Tạo Point Cloud
print("Đang tạo mô hình 3D...")
# Tự tạo thông số Camera dựa trên độ phân giải ảnh của bạn
intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width, height,  # Chiều rộng, chiều cao
    width,          # focal_length_x
    width,          # focal_length_y
    width / 2.0,    # cx (tâm ảnh)
    height / 2.0    # cy (tâm ảnh)
)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

# 4. Kiểm tra xem Point Cloud có dữ liệu không
if len(pcd.points) == 0:
    print("❌ Lỗi: Point Cloud bị rỗng! File depth_map.png có thể bị lỗi (toàn màu đen/trắng).")
    exit()
else:
    print(f"✅ Đã tạo mô hình 3D với {len(pcd.points)} điểm.")

# 5. Xoay và căn chỉnh hệ trục
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

# 6. Vẽ khung trần bay 10m
min_bound = pcd.get_min_bound()
max_bound = pcd.get_max_bound()

# Khung trần bay
points = [
    [min_bound[0], min_bound[1], min_bound[2] + 10.0],
    [max_bound[0], min_bound[1], min_bound[2] + 10.0],
    [max_bound[0], max_bound[1], min_bound[2] + 10.0],
    [min_bound[0], max_bound[1], min_bound[2] + 10.0]
]
lines = [[0, 1], [1, 2], [2, 3], [3, 0]]

line_set = o3d.geometry.LineSet()
line_set.points = o3d.utility.Vector3dVector(points)
line_set.lines = o3d.utility.Vector2iVector(lines)
line_set.colors = o3d.utility.Vector3dVector([[1, 0, 0] for _ in range(len(lines))])

# 7. Hiển thị
print("Đang mở cửa sổ 3D...")
o3d.visualization.draw_geometries([pcd, line_set], 
                                  window_name="Mô phỏng 3D (Khung đỏ = Độ cao 10m)",
                                  width=1024, height=768)