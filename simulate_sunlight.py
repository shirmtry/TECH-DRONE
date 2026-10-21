import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cv2
from datetime import datetime, timedelta
import pytz
from pysolar.solar import get_altitude, get_azimuth

# ================= 1. CẤU HÌNH KHU VỰC THI =================
# Tọa độ tâm khu vực (Tính trung bình từ 4 góc của bạn)
CENTER_LAT = 13.80427
CENTER_LON = 109.21959

# Thời gian bắt đầu và kết thúc
START_DATE = datetime(2024, 5, 1) # Giả sử bắt đầu từ hôm nay
END_DATE = datetime(2026, 11, 7)  # Ngày thi chung kết
VN_TZ = pytz.timezone('Asia/Ho_Chi_Minh')

# ================= 2. TÍNH TOÁN DỮ LIỆU MẶT TRỜI =================
print(f"Đang mô phỏng dữ liệu từ {START_DATE.date()} đến {END_DATE.date()}...")

data = []
current_date = START_DATE

while current_date <= END_DATE:
    # Quét mỗi giờ từ 6h sáng đến 6h chiều
    for hour in range(6, 19):
        dt = VN_TZ.localize(datetime(current_date.year, current_date.month, current_date.day, hour, 0, 0))
        
        alt = get_altitude(CENTER_LAT, CENTER_LON, dt)
        az = get_azimuth(CENTER_LAT, CENTER_LON, dt)
        
        if alt > 0: # Chỉ tính khi mặt trời đã mọc
            # Tính chiều dài bóng đổ (Shadow Length) của một vật cao 1 mét
            shadow_ratio = 1.0 / np.tan(np.radians(alt)) if alt > 0 else 999
            
            data.append({
                "date": current_date,
                "hour": hour,
                "altitude": alt,
                "azimuth": az,
                "shadow_ratio": shadow_ratio
            })
            
    current_date += timedelta(days=1)

df = pd.DataFrame(data)
print(f"✅ Đã tính xong {len(df)} khung giờ.")

# ================= 3. XUẤT BIỂU ĐỒ NHIỆT (HEATMAP) =================
# Chúng ta muốn xem vào tháng 11 (tháng thi), bóng đổ dài hay ngắn
nov_data = df[(df['date'].dt.month == 11) & (df['date'].dt.day <= 7)]

# Vẽ biểu đồ nhiệt về độ dài bóng đổ
plt.figure(figsize=(12, 6))
# Tạo pivot table: Hàng là ngày, Cột là giờ
pivot_shadow = nov_data.pivot_table(index=nov_data['date'].dt.strftime('%m-%d'), 
                                    columns='hour', 
                                    values='shadow_ratio')

plt.title("Tỷ lệ Bóng đổ (Shadow Ratio) trong tuần thi (1/11 - 7/11/2026)")
plt.xlabel("Giờ trong ngày (6h sáng - 6h chiều)")
plt.ylabel("Ngày thi")
plt.imshow(pivot_shadow, cmap='hot_r', aspect='auto')
plt.colorbar(label='Chiều dài bóng đổ (x lần chiều cao vật)')
plt.xticks(range(len(pivot_shadow.columns)), pivot_shadow.columns)
plt.yticks(range(len(pivot_shadow.index)), pivot_shadow.index)
plt.tight_layout()
plt.savefig("shadow_heatmap_nov.png")
print("✅ Đã lưu biểu đồ nhiệt: shadow_heatmap_nov.png")

# ================= 4. TẠO VIDEO MÔ PHỎNG HƯỚNG NẮNG =================
# Tạo một video mô phỏng hướng nắng vào các ngày thi
print("Đang render video mô phỏng hướng nắng...")
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
video = cv2.VideoWriter('sun_path_simulation.mp4', fourcc, 10.0, (800, 800))

# 4 góc khu vực (để vẽ hình chữ nhật mô phỏng)
pts = np.array([
    [300, 200], [500, 200], [500, 600], [300, 600]
], np.int32)

for index, row in nov_data.iterrows():
    # Tạo khung hình nền đen
    frame = np.zeros((800, 800, 3), dtype=np.uint8)
    
    # Vẽ khu vực thi (hình chữ nhật xám)
    cv2.polylines(frame, [pts], True, (100, 100, 100), 2)
    
    # Vẽ hướng nắng (Azimuth) và độ cao (Altitude)
    az_rad = np.radians(row['azimuth'])
    
    # Vị trí mặt trời (vẽ trên vòng tròn)
    sun_x = int(400 + 300 * np.sin(az_rad))
    sun_y = int(400 - 300 * np.cos(az_rad))
    
    # Màu sắc thay đổi theo độ cao (Càng cao càng vàng)
    color = (0, int(255 * (row['altitude']/90)), 255)
    cv2.circle(frame, (sun_x, sun_y), 15, color, -1)
    
    # Vẽ tia nắng chiếu vào tâm khu vực
    cv2.line(frame, (sun_x, sun_y), (400, 400), (0, 255, 255), 1)
    
    # Ghi thông số lên video
    text_date = f"Date: {row['date'].strftime('%m/%d/%Y')} Time: {row['hour']}:00"
    text_alt = f"Alt: {row['altitude']:.1f} deg | Shadow Ratio: {row['shadow_ratio']:.1f}x"
    
    cv2.putText(frame, text_date, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame, text_alt, (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    video.write(frame)

video.release()
cv2.destroyAllWindows()
print("✅ Đã xuất video: sun_path_simulation.mp4")