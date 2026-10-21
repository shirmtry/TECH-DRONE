import math

def calculate_flight_time(step, max_val, speed, takeoff_time=5, land_time=5, turn_penalty=1.0):
    """
    step: Khoảng cách giữa các làn (mét)
    max_val: Cạnh của hình vuông (mét)
    speed: Vận tốc bay (m/s)
    """
    wps = []
    x = 0.0
    forward = True
    
    # Tạo waypoint
    while x <= max_val:
        if forward:
            wps.append((x, 0.0))     
            wps.append((x, max_val)) 
        else:
            wps.append((x, max_val)) 
            wps.append((x, 0.0))     
        x += step
        forward = not forward
        
    # Tính quãng đường
    total_dist = 0.0
    turns = len(wps) - 1
    for i in range(len(wps)-1):
        x1, y1 = wps[i]
        x2, y2 = wps[i+1]
        dist = math.hypot(x2-x1, y2-y1)
        total_dist += dist
        
    # Tính thời gian
    flight_time = total_dist / speed
    total_time = takeoff_time + flight_time + (turns * turn_penalty) + land_time
    
    return len(wps), total_dist, flight_time, total_time

if __name__ == "__main__":
    # Tham số đầu vào
    AREA_SIZE = 25.0
    SPEED = 5.0 # m/s (Bạn có thể thay đổi theo thực tế)
    
    print(f"--- TÍNH TOÁN THỜI GIAN BAY KHU VỰC {AREA_SIZE}x{AREA_SIZE}m ---")
    print(f"Vận tốc giả định: {SPEED} m/s\n")
    
    # Thử nghiệm với các khoảng cách step khác nhau
    for step in [4.0, 5.0, 6.0, 8.0]:
        wps, dist, flight_t, total_t = calculate_flight_time(step, AREA_SIZE, SPEED)
        print(f"Khoảng cách làn (step) = {step}m:")
        print(f"  + Số điểm dừng: {wps}")
        print(f"  + Tổng quãng đường: {dist:.1f} m")
        print(f"  + Thời gian bay liên tục: {flight_t:.1f} s")
        print(f"  + TỔNG THỜI GIAN (có cất/hạ cánh & bo góc): {total_t:.1f} s")
        print("-" * 40)