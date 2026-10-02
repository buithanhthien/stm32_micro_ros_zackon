# STM32 bật trước mini PC

Phát hiện ban đầu tại máy ngày 2026-10-01 (trước khi sửa):

- Firmware Core/Src/lwip.c: STM32 192.168.1.50/24.
- Firmware Core/Src/freertos.c: Agent 192.168.1.150:8888.
- NetworkManager profile STM32_Link: mini PC 192.168.4.150/24.
- Service Agent kẹt ExecStartPre chờ 192.168.1.150, timeout và restart.
- UI trước sửa ping 192.168.4.50; ping thực tế không nhận trả lời.
- stm32_reset.service chờ subscriber /mcu_reset; firmware không tạo subscriber này.
- Firmware reset MCU sau 20 lần ping Agent thất bại khi link UP.

## Thay đổi

Firmware tiếp tục chờ Agent, không reset MCU vì Agent chưa bật. Luồng
WAIT_NET -> WAIT_AGENT -> CREATE_ENTITIES -> RUN và tái tạo session được giữ.
Đã build trên Ubuntu bằng GCC ARM 13.2.1 và thư viện micro-ROS Jazzy
(76 packages). Bộ công cụ đặt tại .tools, kết quả tại build-linux/app.
Xem tools/README.md để build/nạp lại, không cần CubeIDE.

Đã sửa thêm LWIP_RAM_HEAP_POINTER: địa chỉ cứng 0x20048000 chồng lên
.bss của bản build mới. Cả hai lwipopts.h hiện dùng heap do linker bố trí.

Service mới bỏ ExecStartPre phụ thuộc IP; giữ khởi động cùng máy và Restart=always.
Cài bằng terminal có quyền sudo:

```bash
bash /home/khoaiuh/stm32_micro_ros_zackon/deployment/install-agent-service.sh
```

Script sao lưu service cũ vào /var/backups/stm32-microros, tắt service reset
và khởi động Agent. Không tự sửa profile mạng.

UI đã được cập nhật tại /home/khoaiuh/zackon_build_up/robot_ui/startup_layout.py:
hiển thị ActiveState/SubState/Result, dùng STM32_IP (mặc định 192.168.4.50)
và STM32_IFACE (mặc định enx00e04c534458). Bản diff lưu cạnh tài liệu này.

## IP đã thống nhất theo yêu cầu

- Mini PC / Agent: 192.168.4.150/24, UDP 8888.
- STM32: 192.168.4.50/24, không có gateway.
- Profile STM32_Link hiện đã dùng 192.168.4.150/24.

Đã sửa đích Agent trong Core/Src/freertos.c, IP STM32 trong cả
Core/Src/lwip.c và LWIP/App/lwip.c, cùng cấu hình CubeMX .ioc để khi
sinh lại mã vẫn giữ đúng IP. UI mặc định kiểm tra 192.168.4.50.
Cần build và nạp lại firmware để các thay đổi này có hiệu lực trên STM32.
File microros_agent.service.before giữ nguyên làm bản sao cấu hình cũ.

## Kiểm thử trên thiết bị

1. Build/nạp firmware đã sửa bằng môi trường STM32 có thư viện micro-ROS.
2. Cài service, thống nhất IP, mở lại UI.
3. Ngắt nguồn cả hai; cấp nguồn STM32, đợi ít nhất một phút rồi bật mini PC.
4. Kiểm tra service active/running và UDP 8888; kiểm tra nhận dữ liệu thật:

```bash
source /opt/ros/jazzy/setup.bash
source /home/khoaiuh/zackon_build_up/install/setup.bash
ros2 topic echo /odomfromSTM32 --once
ros2 topic hz /odomfromSTM32
```

5. UI phải báo STM32 có dữ liệu mới; thử khởi động lại Agent rồi kiểm tra
odom tự trở lại. Việc chỉ thấy topic hoặc ping được chưa đủ để kết luận.

Đã kiểm tra cú pháp Python UI, bash script, systemd unit và git diff --check.
Đã nạp và xác minh flash thành công qua ST-Link. Agent đã được cài,
enabled và tự chạy sau boot; stm32_reset.service đã disabled.

Kiểm tra ngày 2026-10-02: firmware có sẵn ban đầu nhận được odom. Sau khi
nạp/reset, cả bản mới và bản cũ khôi phục đều gửi ARP nhưng không nhận
được phản hồi (mini PC có gửi ARP reply đúng MAC). Bản mới nhất đã được
nạp lại. Chưa xác nhận odom của bản mới hoặc bài thử mất nguồn hoàn toàn;
cần kiểm tra khởi động nguội trên thiết bị trước khi kết luận hoàn tất.
