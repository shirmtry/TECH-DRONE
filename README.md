AI Drone Challenge FA26 — DJI Matrice 4E - TEAM TECH DRONE
### Tài liệu phát triển, mô phỏng và chuẩn bị triển khai thực tế

> **Trạng thái:** Tài liệu làm việc cho đội thi. Cập nhật theo thông tin đã xác nhận đến ngày 21/10/2026.  
> **Nguyên tắc quan trọng:** Mô phỏng quỹ đạo hoặc chạy được code Python không đồng nghĩa với việc đã điều khiển được DJI Matrice 4E thật. Chỉ tích hợp điều khiển thật sau khi ban tổ chức xác nhận SDK/API, quyền điều khiển, giao thức, phần cứng và quy trình an toàn.

## 1. Mục tiêu dự án

Repository phục vụ quá trình nghiên cứu và chuẩn bị cho **AI Drone Challenge FA26 tại FPT University Quy Nhơn**, với drone thi đấu được xác nhận là **DJI Matrice 4E**.

Các mục tiêu phát triển:

- Thiết kế và phân tích quỹ đạo bay bằng Python.
- Tạo flight plan dạng CSV để kiểm tra waypoint, độ cao, khoảng cách và trình tự nhiệm vụ.
- Trực quan hóa vùng khảo sát, quỹ đạo và kết quả mô phỏng.
- Xây dựng pipeline xử lý ảnh/phát hiện mục tiêu khi đề thi và giao diện camera được công bố.
- Tách biệt logic nhiệm vụ dùng chung với lớp tích hợp simulator hoặc drone thật.
- Kiểm tra an toàn, khả năng phục hồi lỗi và thời gian hoàn thành nhiệm vụ trước khi thử nghiệm thực địa.

## 2. Lịch trình cuộc thi

| Mốc | Thời gian | Ghi chú |
|---|---|---|
| Vòng mô phỏng sơ loại | 17/10/2026 | Mốc đã được thông báo trước đó |
| Workshop 2 — DJI Matrice 4E | 31/10/2026 | Xác minh API, quyền điều khiển và quy tắc kỹ thuật |
| Chung kết thực tế | 07/11/2026 | FPT University Quy Nhơn |

Lịch có thể được ban tổ chức cập nhật. Luôn ưu tiên thông báo chính thức mới nhất.

## 3. Phạm vi hiện có trong repository

Dựa trên các đầu ra đã tạo, dự án hiện có các nhóm artefact:

- `test/drone_planner.py` — mã lập kế hoạch/kiểm thử quỹ đạo.
- `test/flight_plan.csv` — kế hoạch bay dạng bảng.
- `test/flights_index.csv`, `test/weather_hourly.csv` — dữ liệu chỉ mục và dữ liệu thời tiết.
- `test/coverage_map.png`, `test/heatmap_scanned.png` — ảnh trực quan hóa.
- `test/flights_all.zip` — gói dữ liệu chuyến bay.
- `flight_mission_*/` — các kết quả thử nghiệm theo chiến lược, ví dụ:
  - `flight_mission_spiral_expansion_*`
  - `flight_mission_full_coverage_*`
  - `flight_mission_multitier_*`
  - `flight_mission_safe_fixed_*`
  - `flight_mission_smooth_*`
  - `flight_mission_spot_focus_*`

Các thư mục kết quả có thể chứa CSV quỹ đạo, bản đồ được chú thích, video mô phỏng và script được sinh tự động. Tên file có thể thay đổi theo timestamp; hãy kiểm tra thư mục thực tế trước khi chạy.

**Lưu ý:** Các file CSV/map/video và tên thư mục như trên chỉ chứng minh đã tạo ra artefact lập kế hoạch/trực quan hóa. Không nên mô tả chúng là bằng chứng PX4/Gazebo SITL hoặc DJI Matrice 4E thật đã được điều khiển thành công nếu chưa có log tương ứng.

## 4. Kiến trúc phần mềm khuyến nghị

Tách code thành các lớp độc lập để tránh ghép nhầm simulator với drone thật:

```text
project/
├── README.md
├── .gitignore
├── requirements.txt             # Nếu dự án dùng pip
├── config/
│   ├── sim.yaml                 # Cấu hình mô phỏng
│   └── mission.example.yaml     # Tham số nhiệm vụ mẫu, không phải cấu hình bay thật
├── src/
│   ├── mission/                 # Logic nhiệm vụ và trạng thái
│   ├── planning/                # Waypoint, quỹ đạo, vùng khảo sát
│   ├── perception/              # Camera, tiền xử lý, phát hiện mục tiêu
│   ├── safety/                  # Giới hạn, timeout, kiểm tra trạng thái
│   └── adapters/
│       ├── simulator.py         # Adapter simulator
│       └── dji_real.py          # Chỉ triển khai sau khi được phép và xác minh SDK
├── test/
│   ├── drone_planner.py
│   └── ...
└── flight_mission_*/
```

Đây là cấu trúc mục tiêu để tổ chức code; không có nghĩa tất cả thư mục trên đã tồn tại trong repository.

### Nguyên tắc thiết kế

1. **Mission logic không phụ thuộc thiết bị:** mô tả mục tiêu nhiệm vụ, điểm khảo sát và điều kiện kết thúc.
2. **Adapter riêng cho từng môi trường:** simulator và DJI thật có API, tọa độ, trạng thái và cơ chế an toàn khác nhau.
3. **Cấu hình tách khỏi code:** không hard-code tọa độ, tốc độ hoặc thông tin kết nối trong nhiều file.
4. **Có trạng thái rõ ràng:** `INIT → PRECHECK → READY → RUNNING → COMPLETE/ABORT`.
5. **Lỗi phải đưa hệ thống về trạng thái an toàn:** mất telemetry, dữ liệu camera lỗi, timeout hoặc vị trí không hợp lệ phải kích hoạt quy trình dừng/abort đã được phê duyệt.
6. **Ghi log:** timestamp, trạng thái nhiệm vụ, vị trí/độ cao nếu có, kết quả phát hiện, cảnh báo và nguyên nhân kết thúc.

## 5. Môi trường simulation

### 5.1 Xác định simulator thực tế

Không tự giả định cuộc thi dùng một simulator cụ thể. Cần xác nhận từ ban tổ chức:

- Simulator và phiên bản được chấm.
- Hệ điều hành, Python/runtime và thư viện được cho phép.
- Cách nhận ảnh/camera frame và dữ liệu telemetry.
- Cách gửi lệnh điều khiển hoặc nộp waypoint.
- Hệ tọa độ, đơn vị, giới hạn vùng bay và thời gian chạy.
- Cách tính điểm, tiêu chí an toàn và định dạng output.

Nếu vòng thi sử dụng **PX4 SITL + Gazebo + ROS 2 Humble**, phải chạy và kiểm tra đúng môi trường được ban tổ chức quy định. Không xem script sinh CSV hoặc video minh họa là thay thế cho bài test SITL.

### 5.2 Thiết lập Python trên Windows

Từ thư mục repository:

```bat
cd C:\Users\dinht\Downloads\261011-kit
py --version
git status
```

Nếu repository có `requirements.txt`, tạo môi trường ảo và cài đúng dependency đã khai báo:

```bat
py -m venv .venv
.venv\Scripts\activate
py -m pip install --upgrade pip
py -m pip install -r requirements.txt
```

Nếu không có `requirements.txt`, **không cài một danh sách thư viện đoán mò**. Kiểm tra import trong script, tài liệu dự án và phiên bản Python tương thích trước.

### 5.3 Chạy thử planner hiện có

```bat
py test\drone_planner.py
```

Chỉ dùng lệnh trên nếu file là script chạy độc lập; nếu nó yêu cầu tham số hoặc được gọi bởi script khác, xem phần `if __name__ == "__main__":` và hướng dẫn trong code.

Chạy script tạo các phương án quỹ đạo nếu đây là entry point của repository:

```bat
py testbay.py
```

Sau khi chạy, kiểm tra các file đầu ra: CSV, ảnh bản đồ, log và video. Không tự động coi video được render là mô phỏng vật lý có telemetry thật.

### 5.4 Quy trình kiểm thử mô phỏng

- [ ] Kiểm tra dữ liệu đầu vào và hệ tọa độ.
- [ ] Xác minh mọi waypoint nằm trong vùng cho phép.
- [ ] Kiểm tra khoảng cách giữa các waypoint và độ cao hợp lệ.
- [ ] Kiểm tra quỹ đạo không có bước nhảy hoặc đổi hướng bất thường.
- [ ] Đo thời gian thực thi bằng timestamp/log, không chỉ ước lượng từ hình ảnh.
- [ ] Kiểm thử mất frame, detection rỗng, dữ liệu lỗi và timeout.
- [ ] Chạy nhiều seed/tình huống nếu simulator hỗ trợ.
- [ ] Lưu cấu hình, commit hash và kết quả để có thể tái lập.

## 6. Lập kế hoạch quỹ đạo và khảo sát

Repository đã thử nghiệm nhiều kiểu kế hoạch: xoắn ốc mở rộng, phủ vùng theo lưới, nhiều tầng, quỹ đạo mượt, phương án ưu tiên điểm và phương án cố định an toàn.

Khi so sánh, không chỉ dựa vào hình ảnh đẹp. Hãy đo bằng các chỉ số:

| Chỉ số | Ý nghĩa |
|---|---|
| Mission completion | Nhiệm vụ hoàn thành đúng điều kiện hay không |
| Coverage | Tỷ lệ vùng/đối tượng được quan sát theo định nghĩa đề thi |
| Detection quality | Precision/recall hoặc metric ban tổ chức chỉ định |
| Mission duration | Thời gian từ bắt đầu đến kết thúc |
| Path length | Tổng chiều dài quỹ đạo |
| Constraint violations | Số lần vi phạm vùng, độ cao, tốc độ hoặc giới hạn khác |
| Safety events | Timeout, mất telemetry, abort hoặc sự kiện nguy hiểm |

Các tham số thử nghiệm như độ cao, tốc độ, bán kính quỹ đạo, khoảng cách waypoint và ngưỡng confidence phải nằm trong file cấu hình. Không sao chép tham số từ mô phỏng sang drone thật khi chưa được kiểm định.

## 7. Computer vision / AI perception

Pipeline tổng quát:

```text
Camera frame
    ↓
Kiểm tra frame hợp lệ + timestamp
    ↓
Tiền xử lý ảnh
    ↓
Mô hình phát hiện/phân loại mục tiêu
    ↓
Lọc confidence + loại bỏ phát hiện trùng
    ↓
Kết quả có cấu trúc + thời điểm + độ tin cậy
    ↓
Mission logic quyết định bước tiếp theo
```

Khuyến nghị:

- Tách model inference khỏi điều khiển bay.
- Đo latency trung bình và latency ở phân vị cao (ví dụ P95).
- Kiểm tra frame rỗng, độ phân giải khác, ánh sáng thay đổi và detection không ổn định.
- Lưu phiên bản model, nhãn, ngưỡng confidence và cấu hình tiền xử lý.
- Không biến một phát hiện đơn lẻ thành lệnh bay nguy hiểm; áp dụng kiểm tra hợp lệ và điều kiện nhiệm vụ đã được phê duyệt.
- Dùng đúng nguồn camera/API mà ban tổ chức cung cấp; không tự giả định topic ROS hoặc luồng video DJI.

## 8. Chuẩn bị tích hợp DJI Matrice 4E thật

**Đây là checklist tích hợp, không phải hướng dẫn tự ý cất cánh.** Chỉ thử nghiệm trong khu vực được phép, dưới sự giám sát của người có thẩm quyền và theo quy trình an toàn của ban tổ chức.

### 8.1 Những điều cần xác nhận tại Workshop 2

- [ ] SDK/API được phép sử dụng và phiên bản cụ thể.
- [ ] Có cho phép điều khiển tự động hay chỉ xử lý dữ liệu/nộp kết quả.
- [ ] Máy tính điều khiển và kết nối vật lý/network được phép.
- [ ] Quyền truy cập camera, telemetry, gimbal và dữ liệu định vị.
- [ ] Cơ chế upload nhiệm vụ/waypoint và định dạng được yêu cầu.
- [ ] Hệ tọa độ, đơn vị, mốc gốc và quy ước trục.
- [ ] Giới hạn tốc độ, độ cao, vùng hoạt động và khoảng cách an toàn.
- [ ] Quy trình takeoff, landing, return-to-home, abort và takeover thủ công.
- [ ] Cách xử lý mất kết nối, pin yếu, GNSS yếu và obstacle warnings.
- [ ] Yêu cầu kiểm tra thiết bị, firmware và quy định pháp lý/địa điểm.

Ghi lại câu trả lời chính thức trong tài liệu đội thi trước khi viết adapter thực tế.

### 8.2 Tách biệt simulator và thiết bị thật

Không dùng chung một hàm gửi lệnh nếu hai hệ thống có ngữ nghĩa khác nhau. Adapter thật chỉ được viết dựa trên SDK/API chính thức được phép, và phải kiểm tra trạng thái/điều kiện trước mỗi thao tác.

Ví dụ giao diện ở mức kiến trúc (pseudo-code, không phải DJI SDK thực thi):

```python
class DroneAdapter:
    def connect(self):
        raise NotImplementedError

    def get_telemetry(self):
        raise NotImplementedError

    def execute_approved_mission(self, mission):
        raise NotImplementedError

    def request_safe_abort(self, reason):
        raise NotImplementedError
```

Không tự điền lệnh SDK, địa chỉ kết nối, port hoặc tên topic cho DJI Matrice 4E nếu chưa có tài liệu xác nhận. Pseudo-code này không trực tiếp điều khiển drone.

### 8.3 Các giai đoạn xác minh trước khi bay

1. **Static review:** kiểm tra code, cấu hình, tọa độ, giới hạn và xử lý lỗi.
2. **Offline test:** chạy planner/perception trên dữ liệu ghi sẵn.
3. **Simulator test:** xác minh logic và các trường hợp lỗi trong môi trường được chấp thuận.
4. **Bench test:** chỉ kiểm tra kết nối/trạng thái khi thiết bị được cố định và theo quy trình nhà sản xuất/ban tổ chức.
5. **Supervised field test:** chỉ thực hiện khi đã được cho phép, vùng bay an toàn, có người giám sát và quy trình takeover/abort.
6. **Competition rehearsal:** chạy đúng phiên bản đã đóng băng, kiểm tra thời gian, log và quy trình bắt đầu/kết thúc.

Không thực hiện thử nghiệm cánh quạt quay trên bàn hoặc gần người; tuân thủ quy trình an toàn chính thức.

## 9. An toàn và điều kiện abort

Mission controller cần có các điều kiện dừng đã được đội thi và ban tổ chức thống nhất, ví dụ:

- Telemetry mất hoặc quá cũ.
- Dữ liệu vị trí/độ cao không hợp lệ.
- Camera hoặc pipeline perception không hoạt động.
- Vượt giới hạn vùng bay hoặc giới hạn nhiệm vụ.
- Timeout, pin thấp hoặc cảnh báo thiết bị.
- Có người/vật cản hoặc điều kiện môi trường không an toàn.
- Người giám sát yêu cầu dừng hoặc takeover.

Khi có sự cố, ưu tiên cơ chế dừng/abort được DJI và ban tổ chức phê duyệt. Không tự viết hành vi return-to-home, landing hay lệnh điều khiển khẩn cấp theo phỏng đoán.

## 10. Git và quản lý artefact

### Bỏ qua video MP4 ở mọi thư mục

Thêm vào `.gitignore`:

```gitignore
*.mp4
```

Quy tắc này áp dụng cho các file MP4 ở thư mục gốc và thư mục con. Nếu MP4 đã được Git theo dõi, cần gỡ khỏi index trước khi commit; thao tác này không xóa file video trên máy tính:

```bat
git rm --cached -r -- "*.mp4"
git add .gitignore
git commit -m "Ignore generated MP4 videos"
```

Nếu pathspec không khớp trên môi trường Git đang dùng, liệt kê file đã được theo dõi bằng `git ls-files` rồi gỡ các đường dẫn MP4 cụ thể khỏi index.

### Quy trình commit đề xuất

```bat
git status
git add .
git diff --cached --stat
git commit -m "Describe the change"
git push origin main
```

Trước khi commit, kiểm tra để không đưa lên repository các file lớn không cần thiết, dữ liệu nhạy cảm, token, thông tin kết nối hoặc artefact sinh tự động không liên quan. `.gitignore` không xóa file đã có trong lịch sử Git.

## 11. Nhật ký thí nghiệm

Với mỗi lần chạy, ghi tối thiểu:

```text
Experiment ID:
Date/time:
Git commit:
Environment (OS / Python / simulator):
Mission config:
Model/version:
Input dataset or scenario:
Expected result:
Actual result:
Duration:
Coverage / detection metrics:
Safety events:
Output folder:
Conclusion / next change:
```

Không chỉ lưu video; giữ cả cấu hình, CSV, log và phiên bản code để so sánh kết quả công bằng.

## 12. Definition of Done trước ngày thi

- [ ] Biết chính xác simulator và môi trường chấm.
- [ ] Có lệnh chạy lại được từ README.
- [ ] Kết quả có thể tái lập từ commit và cấu hình đã lưu.
- [ ] Kiểm tra waypoint, hệ tọa độ và đơn vị.
- [ ] Có test cho lỗi perception, timeout và dữ liệu không hợp lệ.
- [ ] Đã xác nhận chính thức API/quyền điều khiển DJI Matrice 4E.
- [ ] Adapter thật không chứa lệnh hoặc tham số phỏng đoán.
- [ ] Quy trình preflight, abort và takeover được duyệt.
- [ ] Đã diễn tập với phiên bản code được đóng băng.
- [ ] Đã kiểm tra Git không chứa video hoặc secret không mong muốn.

## 13. Nguồn tài liệu cần ưu tiên

1. Tài liệu, thông báo và quy định kỹ thuật chính thức từ ban tổ chức AI Drone Challenge FA26.
2. Tài liệu DJI chính thức tương ứng với Matrice 4E, firmware, SDK và quyền sử dụng thực tế.
3. Tài liệu chính thức của simulator/middleware đúng phiên bản mà cuộc thi yêu cầu.
4. Tài liệu thư viện và model thực sự được dùng trong repository.

Không coi ví dụ trên mạng, API của dòng drone khác hoặc cấu hình simulator khác là bằng chứng tương thích với DJI Matrice 4E.

---

**Maintainer:** Đội thi TECH DRONE
**Last updated:** 2026-10-21  
**Status:** Simulation/planning documentation; real-drone integration awaits official interface and competition-rule confirmation.