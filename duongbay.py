import matplotlib.pyplot as plt
import math

def generate_lawnmower(step, max_val=25.0):
    """Tạo danh sách các waypoint cho đường bay Zic-zắc"""
    wps = []
    x = 0.0
    forward = True
    while x <= max_val:
        if forward:
            wps.append((x, 0.0))     
            wps.append((x, max_val)) 
        else:
            wps.append((x, max_val)) 
            wps.append((x, 0.0))     
        x += step
        forward = not forward
    return wps

def calculate_stats(wps, speed=5.0):
    """Tính quãng đường và thời gian"""
    total_dist = 0.0
    for i in range(len(wps)-1):
        x1, y1 = wps[i]
        x2, y2 = wps[i+1]
        total_dist += math.hypot(x2-x1, y2-y1)
    
    flight_time = total_dist / speed
    # Cộng thêm 15s cho cất/hạ cánh và bo góc
    total_time = flight_time + 15 
    return total_dist, total_time

def plot_paths():
    # Tạo 2 biểu đồ con cạnh nhau
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 7))
    
    # --- BIỂU ĐỒ 1: STEP = 5.0m ---
    step5 = 5.0
    wps5 = generate_lawnmower(step=step5)
    dist5, time5 = calculate_stats(wps5)
    
    x5 = [p[0] for p in wps5]
    y5 = [p[1] for p in wps5]
    
    ax1.plot(x5, y5, marker='o', linestyle='-', color='blue', markersize=4, label='Đường bay')
    ax1.plot(x5[0], y5[0], marker='s', color='green', markersize=10, label='Bắt đầu')
    ax1.plot(x5[-1], y5[-1], marker='X', color='red', markersize=10, label='Kết thúc')
    
    ax1.set_title(f'Bước nhảy = {step5}m\nQuãng đường: {dist5:.1f}m | Thời gian: ~{time5:.1f}s', fontsize=12)
    ax1.set_xlabel('Trục X (mét)')
    ax1.set_ylabel('Trục Y (mét)')
    ax1.grid(True, linestyle='--', alpha=0.7)
    ax1.axis('equal')
    ax1.legend()

    # --- BIỂU ĐỒ 2: STEP = 6.0m ---
    step6 = 6.0
    wps6 = generate_lawnmower(step=step6)
    dist6, time6 = calculate_stats(wps6)
    
    x6 = [p[0] for p in wps6]
    y6 = [p[1] for p in wps6]
    
    ax2.plot(x6, y6, marker='o', linestyle='-', color='purple', markersize=4, label='Đường bay')
    ax2.plot(x6[0], y6[0], marker='s', color='green', markersize=10, label='Bắt đầu')
    ax2.plot(x6[-1], y6[-1], marker='X', color='red', markersize=10, label='Kết thúc')
    
    ax2.set_title(f'Bước nhảy = {step6}m (Tối ưu nhất)\nQuãng đường: {dist6:.1f}m | Thời gian: ~{time6:.1f}s', fontsize=12)
    ax2.set_xlabel('Trục X (mét)')
    ax2.set_ylabel('Trục Y (mét)')
    ax2.grid(True, linestyle='--', alpha=0.7)
    ax2.axis('equal')
    ax2.legend()

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    print("Đang mô phỏng đường bay...")
    plot_paths()