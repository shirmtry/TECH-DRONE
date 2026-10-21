import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import math

def generate_adaptive_path(step=5.0, max_val=25.0):
    """Tạo đường bay Zic-zắc với độ cao thay đổi để né cây"""
    wps = []
    x = 0.0
    forward = True
    
    # Giả lập vùng có cây (X từ 10 đến 15, Y từ 10 đến 20)
    tree_x_min, tree_x_max = 10.0, 15.0
    tree_y_min, tree_y_max = 10.0, 20.0
    
    alt_normal = 4.0
    alt_over_tree = 7.0  # Bay cao 7m để né cây 3m
    
    while x <= max_val:
        y_range = range(0, int(max_val)+1, 2) # Chia nhỏ mỗi 2m
        if not forward:
            y_range = reversed(y_range)
            
        for y in y_range:
            if tree_x_min <= x <= tree_x_max and tree_y_min <= y <= tree_y_max:
                z = alt_over_tree
            else:
                z = alt_normal
            wps.append((x, float(y), z))
            
        x += step
        forward = not forward
        
    return wps

def animate_drone():
    step = 5.0
    wps = generate_adaptive_path(step=step)
    
    x_vals = [p[0] for p in wps]
    y_vals = [p[1] for p in wps]
    z_vals = [p[2] for p in wps]
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 1. Vẽ sẵn đường bay mờ (để thấy trước lộ trình)
    ax.plot(x_vals, y_vals, z_vals, color='gray', linewidth=1, linestyle='--', alpha=0.5, label='Lộ trình dự kiến')
    
    # 2. Vẽ khối cây
    ax.bar3d(10, 10, 0, 5, 10, 3, color='green', alpha=0.3, label='Vùng cây (Cao 3m)')
    
    # 3. Khởi tạo điểm Drone (chấm đỏ) và vệt bay (nét xanh)
    drone_marker, = ax.plot([x_vals[0]], [y_vals[0]], [z_vals[0]], marker='o', color='red', markersize=10, label='Drone')
    trail, = ax.plot([], [], [], color='blue', linewidth=2, label='Vệt đã bay')
    
    # Trang trí biểu đồ
    ax.set_title('Mô phỏng Drone bay tránh cây (Animation)')
    ax.set_xlabel('Trục X (mét)')
    ax.set_ylabel('Trục Y (mét)')
    ax.set_zlabel('Độ cao Z (mét)')
    ax.set_xlim(0, 25)
    ax.set_ylim(0, 25)
    ax.set_zlim(0, 10)
    ax.legend(loc='upper left')

    # 4. Hàm cập nhật từng khung hình
    def update(frame):
        # Cập nhật vị trí Drone
        drone_marker.set_data_3d([x_vals[frame]], [y_vals[frame]], [z_vals[frame]])
        
        # Cập nhật vệt màu xanh (từ điểm 0 đến điểm hiện tại)
        trail.set_data_3d(x_vals[:frame+1], y_vals[:frame+1], z_vals[:frame+1])
        
        return drone_marker, trail

    # 5. Chạy animation (interval=50ms tương đương 20 FPS)
    # repeat=False để dừng lại khi bay xong
    anim = animation.FuncAnimation(
        fig, update, frames=len(x_vals), interval=50, blit=False, repeat=False
    )
    
    plt.show()

if __name__ == '__main__':
    print("Đang chạy animation... (Vui lòng xem cửa sổ đồ thị)")
    animate_drone()