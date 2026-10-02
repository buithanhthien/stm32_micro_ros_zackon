# Build và nạp STM32F756 trên Ubuntu 24.04

Đã chuẩn bị và chạy trực tiếp trên máy này, không cần STM32CubeIDE.

## Build firmware

Tại thư mục gốc dự án:

```bash
make -f Makefile.linux -j4
```

Kết quả trong `build-linux/app/`:

- `MotorControlwithMicroROS.elf`: debug/symbols.
- `MotorControlwithMicroROS.bin`: nạp từ 0x08000000.
- `MotorControlwithMicroROS.hex`: Intel HEX.
- `firmware.map`: bố trí flash/RAM.

Makefile.linux dùng danh sách nguồn CubeIDE trong tools/sources.mk,
CPU Cortex-M7, FPU fpv5-sp-d16, hard-float, linker STM32F756ZGTX_FLASH.ld.
Khi thêm file vào dự án, cập nhật tools/sources.mk.

## Nạp firmware

Kết nối USB ST-Link và cấp nguồn board:

```bash
bash tools/flash-linux.sh
```

Script build trước, sao lưu toàn bộ flash vào build-linux/backups, ghi
chương trình và verify. Không mass-erase. Từ chối binary vượt quá 0xC0000
bytes để bảo vệ PID ở sector 7 (0x080C0000–0x080FFFFF).

Nếu thiếu quyền USB, chạy một lần:

```bash
bash tools/install-stlink-rule.sh
```

Rule ST-Link đã được cài trên máy này. Có thể cần rút/cắm USB sau cài rule.

## Công cụ đã cài trong dự án

- GCC ARM 13.2.1, binutils, newlib: .tools/root/usr.
- OpenOCD 0.12.0 và thư viện phụ thuộc: cùng thư mục.
- Thư viện micro-ROS Jazzy:
  micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros.

Để tải lại compiler/OpenOCD từ kho Ubuntu Noble:

```bash
bash tools/setup-linux-toolchain.sh
```

Để build lại micro-ROS từ workspace nguồn đã tải tại build-linux/uros:

```bash
bash tools/build-microros.sh
```

Script này cần dev_ws/install và mcu_ws đã chuẩn bị trong phiên làm việc;
không phải bootstrap từ một checkout trống. Nó tách thư viện ROS x86 ở
/opt/ros khỏi quá trình cross-compile. Thư viện đóng gói bằng pack-microros.py.

## Kiểm tra odom

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/check-odom.py
```

Chờ bản tin tối đa 20 giây, sau đó đo 10 giây và báo tần số/khoảng ngắt.

## Trạng thái kiểm chứng

- Build thành công; binary bản sửa heap: 271256 bytes.
- Nạp qua ST-Link và verify thành công.
- Heap LwIP đặt trong .bss thay vì địa chỉ cứng chồng lên dữ liệu ứng dụng.
- Chưa xác nhận odom sau nạp: cả firmware mới và firmware cũ khôi phục
  đều không nhận được Ethernet sau reset trong phép thử này.
- Mini PC nhìn thấy STM32 gửi ARP và đã trả lời đúng MAC.
- Cần thử ngắt hoàn toàn nguồn STM32, gồm USB ST-Link, rồi cấp lại.

Bản sao firmware đã nhận được odom trước khi nạp:
`build-linux/backups/stm32-20261002-111044.bin` (1 MB, gồm PID).
Không dùng file này để mass-erase/ghi đè toàn bộ flash nếu chỉ cần phục hồi code.
