Để triển khai dự án này một cách mượt mà và hiểu đúng bản chất khi code và bảo vệ kiến trúc, bạn và nhóm **không cần phải đọc toàn bộ hàng nghìn trang spec của AUTOSAR**, mà chỉ cần tập trung chính xác vào **5 trụ cột kiến thức cốt lõi** dưới đây (đặc biệt là nhánh **AUTOSAR Adaptive Platform - AP** vì hệ thống chạy trên Linux/Raspberry Pi 5):

---

### TRỤ CỘT 1: Phân biệt AUTOSAR Classic (CP) vs AUTOSAR Adaptive (AP)
* **Khái niệm cần nắm**:
  - **AUTOSAR Classic (CP)**: Chạy trên vi điều khiển (MCU - STM32, Infineon Aurix), OSEK/VDX RTOS, thời gian thực cứng (Hard Real-time vi giây), dùng CAN/LIN. Đây thường là ECU điều khiển cơ cấu chấp hành (đạp phanh, xoay vô lăng).
  - **AUTOSAR Adaptive (AP)**: Chạy trên vi xử lý hiệu năng cao (MPU - Raspberry Pi 5, Nvidia Jetson, NXP S32G), chạy trên **POSIX OS (Linux / QNX)**, phục vụ cho AI, ADAS, Autonomous Driving, tính toán phức tạp.
* **Mối liên hệ trong bài toán của bạn**:
  - Raspberry Pi 5 đóng vai trò là một **Adaptive Application (AA)** tính toán quỹ đạo và giải MPC.
  - Sau đó nó giao tiếp với bên ngoài (CARLA hoặc ECU chấp hành) qua mạng Ethernet bằng chuẩn giao tiếp hướng dịch vụ (SOA).

---

### TRỤ CỘT 2: Kiến trúc Hướng Dịch vụ (SOA) & Giao thức SOME/IP (Trọng tâm nhất)
Đây là phần quan trọng nhất cần nắm kỹ để làm việc với các file cấu hình và code truyền thông:

1. **Bản chất của SOA trong ô tô**:
   - Khác với CAN bus truyền thống (gửi nhận theo ID bản tin cố định và đóng khung tín hiệu), SOA coi mỗi tính năng là một **Dịch vụ (Service)**.
   - Bên phát là **Server / Provider**, bên nhận là **Client / Consumer**.
2. **4 thành phần giao tiếp trong một SOME/IP Service Interface**:
   - **Events (Publish/Subscribe)**: Server phát dữ liệu định kỳ hoặc khi có sự kiện (VD: Sensor phát `OnDetectedObstacles` 20Hz, MPC phát `OnMotionControlCommand` 20Hz). Client muốn nhận thì phải `Subscribe`.
   - **Methods (Request/Response hoặc Fire-and-Forget)**: Gọi hàm từ xa RPC (VD: HMI gọi hàm `SetEmergencyOverride()` để ép xe dừng).
   - **Fields**: Tương tự như getter/setter của biến trong C++, có thêm `Notifier` khi biến đổi giá trị (VD: `MpcTuningParameters` để chỉnh tham số MPC khi xe đang chạy).
   - **Eventgroups**: Nhóm các Event lại với nhau để Client có thể đăng ký nhận một nhóm cùng lúc.
3. **SOME/IP-SD (Service Discovery)**:
   - Cơ chế tự động tìm kiếm nhau qua mạng: Khi Pi 5 cắm dây LAN vào máy tính, nó tự phát gói tin multicast UDP (mặc định port `30490`) để hỏi: *"Ở đây có ai cung cấp LocalizationService không?"* và CARLA Server trả lời *"Tôi có, hãy kết nối vào IP:Port này"*. Không cần cấu hình IP cứng phức tạp.
4. **Cấu trúc gói tin SOME/IP Header**:
   - Nắm được: `Message ID` (gồm `Service ID` + `Method/Event ID`), `Length`, `Request ID` (`Client ID` + `Session ID`), `Protocol Version`, `Interface Version`, `Message Type`, `Return Code`.

---

### TRỤ CỘT 3: Thư viện thực thi SOME/IP trên Linux (`COVESA vsomeip`)
Khi triển khai trên Linux/Raspberry Pi 5, bạn sẽ dùng thư viện C++ mã nguồn mở chuẩn công nghiệp là **`vsomeip`** (do BMW khởi xướng, nay thuộc hiệp hội ô tô COVESA):

* **Cách hoạt động**:
  - Giao tiếp giữa 2 process trên cùng Pi 5: Dùng **Unix Domain Sockets** hoặc **Shared Memory** (tốc độ cực nhanh, micro-seconds).
  - Giao tiếp giữa Pi 5 và PC CARLA: Tự động chuyển qua **UDP / TCP Ethernet**.
* **Những gì cần học**:
  - Cách viết file cấu hình `vsomeip.json` (định nghĩa `service`, `instance`, `events`, `eventgroups`, `unicast IP`, `multicast`).
  - Cách dùng C++ API của vsomeip:
    - Tạo `vsomeip::application`.
    - Gọi `offer_service()`, `offer_event()`, `notify_one()`.
    - Phía nhận gọi `request_service()`, `subscribe()`, `register_message_handler()`.

---

### TRỤ CỘT 4: Quy trình Thiết kế & Tệp Cấu hình AUTOSAR (ARXML / Manifest)
Người làm AUTOSAR cần hiểu tài liệu và luồng sinh mã (Workflow):

* **Quy trình thiết kế (Design Workflow)**:
  1. Định nghĩa Data Types (Struct của Obstacle, EgoState...).
  2. Định nghĩa Service Interface (gồm Events, Methods, Fields nào).
  3. Định nghĩa Software Components (SWC Ports: `P-Port` là Provided/Server, `R-Port` là Required/Client).
  4. Network Binding: Gán Service vào IP, Port, Transport Protocol (UDP/TCP).
* **Định dạng file**:
  - Trong AUTOSAR chuẩn: Được mô tả bằng file **`.arxml`** (XML schema của AUTOSAR).
  - Trong triển khai mã nguồn mở/Adaptive: Thường dùng **Franca IDL (`.fidl`)** hoặc cấu hình **JSON (`vsomeip.json`)** để ánh xạ trực tiếp sang C++.

---

### TRỤ CỘT 5: Quản lý Vòng đời & An toàn trên Linux (Execution Management & Health Monitoring)
Trong AUTOSAR Adaptive có 2 phân hệ cốt lõi mà lập trình viên C++ hệ thống cần nắm:

1. **`ara::exec` (Execution Management)**:
   - Ứng dụng chạy dưới dạng tiến trình Linux (Process).
   - Nắm về quản lý trạng thái: `Initializing` $\rightarrow$ `Running` $\rightarrow$ `Terminating`.
   - Cơ chế bắt tín hiệu hệ thống Linux (`SIGINT`, `SIGTERM`) để giải phóng tài nguyên an toàn khi dừng xe.
2. **`ara::phm` (Platform Health Management - Watchdog)**:
   - **Deadline Supervision**: Chu kỳ thuật toán MPC là $50\,\text{ms}$ hoặc $100\,\text{ms}$. Nếu quá thời gian này mà chưa tính toán xong góc lái, hệ thống coi là vi phạm an toàn.
   - **Alive Supervision / Timeout**: Nếu mất tín hiệu cảm biến (không nhận được gói tin SOME/IP qua $150\,\text{ms}$), hệ thống phải kích hoạt chế độ **Fail-Safe** (phanh dừng an toàn khẩn cấp).

---

### 📋 Checklist tài liệu / từ khóa để bạn tìm đọc trước:

| Từ khóa / Chủ đề | Đọc ở đâu / Tài liệu khuyên dùng | Mục đích cần đạt |
| :--- | :--- | :--- |
| **SOME/IP Protocol Specification** | Trang chủ `some-ip.com` hoặc tài liệu của Vector / AUTOSAR PRS SOME/IP | Hiểu cấu trúc gói tin, sự khác nhau giữa Event, Method, Field và cơ chế SOME/IP-SD |
| **COVESA vsomeip in 10 minutes** | GitHub repo: `COVESA/vsomeip` (đọc phần `wiki` và các sample `request-response`, `subscribe-notify`) | Nắm cách viết code C++ tạo Client/Server SOME/IP |
| **AUTOSAR Adaptive Platform Overview** | Slide tổng quan *"AUTOSAR Adaptive Platform for Beginners"* (Vector Informatic hoặc Elektrobit whitepaper) | Hiểu được bức tranh tổng thể: ARA API, ara::com, ara::exec |
| **Modern C++ cho Hệ thống Nhúng (C++14/17)** | Smart Pointers (`std::unique_ptr`, `std::shared_ptr`), Concurrency (`std::thread`, `std::mutex`), RAII | Chuẩn bị tư duy viết C++ an toàn, tránh memory leak trên Raspberry Pi 5 |

Bạn có thể lướt qua wiki của **`COVESA/vsomeip`** trước tiên, vì đây là phần thực tế nhất sẽ ánh xạ thẳng vào các dòng code C++ sắp tới của nhóm!