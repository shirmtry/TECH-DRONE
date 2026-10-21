#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# Đường dẫn tới file CSV (Bạn có thể sửa lại nếu muốn xem file khác)
CSV_PATH = "runs/20261018_154514/telemetry_debug.csv"

def plot_trajectory(file_path):
    if not os.path.exists(file_path):
        print(f"Không tìm thấy file: {file_path}")
        return

    print(f"Đang đọc dữ liệu từ: {file_path}")
    # Đọc file CSV
    df = pd.read_csv(file_path)
    
    # Lấy dữ liệu
    x = df['x']
    y = df['y']
    z = df['z_alt']
    
    # Tạo figure với 2 biểu đồ con
    fig = plt.figure(figsize=(14, 6))

    # ==========================================
    # BIỂU ĐỒ 1: NHÌN TỪ TRÊN XUỐNG (2D - X vs Y)
    # ==========================================
    ax1 = fig.add_subplot(1, 2, 1)
    ax1.plot(x, y, marker='.', linestyle='-', color='blue', label='Đường bay', markersize=2)
    
    # Đánh dấu điểm bắt đầu và kết thúc
    ax1.plot(x.iloc[0], y.iloc[0], marker='s', color='green', markersize=10, label='Bắt đầu')
    ax1.plot(x.iloc[-1], y.iloc[-1], marker='X', color='red', markersize=10, label='Kết thúc')
    
    ax1.set_title('Đường bay nhìn từ trên cao (Mặt phẳng XY)')
    ax1.set_xlabel('Trục X (mét)')
    ax1.set_ylabel('Trục Y (mét)')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.axis('equal') # Đảm bảo tỷ lệ X và Y bằng nhau
    ax1.legend()

    # ==========================================
    # BIỂU ĐỒ 2: KHÔNG GIAN 3D (X, Y, Z)
    # ==========================================
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.plot(x, y, z, color='purple', linewidth=2, label='Quỹ đạo 3D')
    
    # Đánh dấu điểm bắt đầu và kết thúc
    ax2.scatter(x.iloc[0], y.iloc[0], z.iloc[0], color='green', s=100, label='Bắt đầu')
    ax2.scatter(x.iloc[-1], y.iloc[-1], z.iloc[-1], color='red', marker='X', s=100, label='Kết thúc')
    
    ax2.set_title('Quỹ đạo bay không gian 3D (X, Y, Z)')
    ax2.set_xlabel('Trục X (mét)')
    ax2.set_ylabel('Trục Y (mét)')
    ax2.set_zlabel('Độ cao Z (mét)')
    ax2.legend()

    # Hiển thị và lưu ảnh
    plt.tight_layout()
    
    # Lưu biểu đồ ra file ảnh để bạn dễ dàng copy sang Windows
    output_img = "runs/20261018_154514/trajectory_plot.png"
    plt.savefig(output_img, dpi=300)
    print(f"Đã lưu biểu đồ tại: {output_img}")
    
    # Hiển thị lên màn hình (nếu bạn có giao diện đồ họa)
    try:
        plt.show()
    except:
        pass

if __name__ == "__main__":
    plot_trajectory(CSV_PATH)