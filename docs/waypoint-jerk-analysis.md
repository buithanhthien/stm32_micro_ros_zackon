# Phân tích robot khựng khi qua waypoint

## Cập nhật kiểm tra source và bag — 2026-10-03

**Chưa xác nhận robot đã hết khựng.** Các phần bên dưới mục cập nhật này là
lịch sử điều tra, gồm bản sửa đã có trong working tree khi bắt đầu lượt này.
Không nạp firmware, không phát lệnh chạy và không thay cấu hình ROS trong lượt này.

### Luồng ROS tìm thấy trên máy

- `/home/khoaiuh/zackon_build_up/robot_ui/waypoints_mode_layout.py`:
  `run_sequence()` gọi `navigate_to_waypoint()` cho điểm đầu; hàm này gửi
  `NavigateToPose` tới `/navigate_to_pose`. `_goal_result_callback()` chỉ gửi
  điểm tiếp theo khi nhận `STATUS_SUCCEEDED`. Đây là **chuỗi đích riêng biệt**,
  không phải một đường xuyên nhiều điểm. Đã xác minh từ source; chưa xác minh
  UI này đang chạy đúng lúc xảy ra triệu chứng được báo.
- `src/view_robot/launch/NAV2_BRINGUP.launch.py` trong cùng workspace include
  `zackon_navigation.launch.py`, truyền `nav2_params.yaml`. Launch navigation
  tạo controller/planner/behavior/BT/waypoint follower, chỉ remap TF; không
  remap cmd_vel qua smoother hoặc collision monitor. Hai node này có YAML
  nhưng không được tạo trong launch navigation đã đọc. Cần kiểm tra graph
  thực tế trước khi kết luận node nào cuối cùng xuất `/cmd_vel`.
- YAML: controller 20 Hz, RotationShim bọc MPPI; SimpleGoalChecker có
  xy/yaw tolerance 0,25, stateful=true, rotate_to_goal_heading=true;
  planner SmacPlanner2D giữ hướng cuối (`use_final_approach_orientation=false`).
  Vì UI gửi từng goal, mỗi điểm chịu điều kiện hoàn thành goal và căn hướng
  cuối. Đây là cơ chế ROS có thể gây giảm tốc/dừng tại từng waypoint.
- `WaitAtWaypoint` có pause=200 ms nhưng UI trên **không gọi FollowWaypoints**;
  không dùng thông số này để quy kết thời gian dừng của UI.
- Sau `/cmd_vel`: micro-ROS subscriber BEST_EFFORT, KEEP_LAST depth=1 →
  callback → deadband/minimum boost → Teleop/watchdog → mailbox → heading/kick
  → ramp → PID → PWM. Encoder cung cấp phản hồi PID và odom theo luồng đã
  mô tả bên dưới. Firmware không nhận chỉ số waypoint hoặc cờ waypoint cuối.

### Đối chiếu dữ liệu có sẵn

Đọc lại trực tiếp rosbag `build-linux/experiments/straight-20261002-174444/bag`
bằng `tools/analyze-motion-bag.py`, không replay vào ROS graph. Tóm tắt bền vững:
`docs/diagnostics/straight-20261002-audit.json`. CSV chung timestamp nhận nằm ở
`build-linux/experiments/straight-20261002-174444/audit-timeline.csv`.

| Quan sát | Kết quả |
| --- | --- |
| cmd_vel | 27349 mẫu; 27245 mẫu zero chính xác; gap lớn nhất 349,74 ms |
| Chuyển nonzero → zero | Một lần, 5,5211 s sau odom đầu tiên trong bag |
| Sau chuyển zero | Hàng nghìn bản tin/giây (cao nhất 5526 mẫu trong một bin 1 s) |
| odomfromSTM32 | 728 mẫu; gap lớn nhất 4,0991 s |
| v odom | −0,01234 đến +0,08191 m/s |
| w odom | −0,39962 đến +0,40730 rad/s |

`report.json` cùng thư mục ghi trial FollowPath đường thẳng kết thúc vì
`measured speed exceeded guard`, sau đó cancel và **không xác nhận được dừng**.
Thời điểm chuyển zero phù hợp giai đoạn cancel. Bag không xác định publisher
của từng Twist; không quy kết lượng zero lớn cho controller hay firmware.
Script trial hiện tại có giới hạn phát zero 20 Hz, nên không được coi source
hiện tại là bằng chứng chính xác phiên bản script đã tạo bag trước đó.

Đây không phải bag đi qua nhiều waypoint và không có trace setpoint/PWM/raw
encoder đồng thời. Tốc độ bánh tính ngược từ twist odom chỉ là đại lượng suy
ra, không thay thế phép đo encoder độc lập. Gap nhận bag không chứng minh
callback MCU timeout. Không có cặp before/after cùng đường chạy để đánh giá
hiệu quả sửa; cũng không suy ra PID/firmware là nguyên nhân robot lạng.

### Thay đổi nhỏ trong lượt này và đề xuất ROS

Host test mới tái hiện lỗi giới hạn: Teleop clamp ±1 m/s nhưng kick cộng
0,03 m/s sau clamp, heading cũng có thể đẩy một bánh vượt giới hạn. Sửa duy
nhất trong code điều khiển lượt này là clamp lại ±MAXSPEED **sau heading/kick,
trước ramp**, ở `Core/Src/freertos.c`. Giới hạn giữ nguyên 1 m/s và 2,5 m/s².
Đây là lỗi setpoint được xác minh bằng test, không phải kết luận nguyên nhân
khựng waypoint. Không đổi heading gain, boost, encoder, mapping, watchdog,
PID hay MotionTask. Các thay đổi mailbox/dt/heading trong working tree là
bản sửa từ trước, vẫn cần A/B vật lý do đã có báo cáo hồi quy.

Đề xuất ROS nhỏ nhất cho chế độ **đi xuyên chuỗi điểm**: `run_sequence()` gửi
một `NavigateThroughPoses.Goal` với toàn bộ poses có thứ tự thay vì nối nhiều
`NavigateToPose` sau SUCCESS. Giữ điều hướng một điểm như cũ. Dùng một goal
handle cho cả chuỗi, giữ cơ chế generation chống callback cũ, cancel toàn
chuỗi khi người dùng dừng; abort/reject không tự gửi lại phần còn lại. Chỉ
báo hoàn tất khi cả action SUCCESS; áp dụng tolerance/hướng đích cho điểm
cuối, không cập nhật yaw tolerance giữa các điểm qua subprocess.

Nav2 Jazzy đã cài có BT
`/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml`:
ComputePathThroughPoses nối đường → FollowPath, RemovePassedGoals radius=0,7 m.
Cần kiểm tra bán kính này với khoảng cách waypoint và hành lang thực tế;
đi xuyên điểm không có nghĩa bắt buộc đạt chính xác tọa độ/hướng tại mỗi điểm.
Controller/recovery vẫn được phép giảm tốc hoặc dừng. Không nới goal checker
cuối và không lọc bỏ Twist zero để giả lập chuyển động liên tục.
Đề xuất này chưa áp dụng vào workspace ROS; chưa có test action end-to-end.

### Kiểm thử và bước còn thiếu

- `python3 tests/test_motor_control.py`: **13/13 kịch bản đạt**. Sáu ca cũ và
  bảy ca mới: callback/Teleop nhận lệnh giữ lặp lại, kick tại giới hạn dương/
  âm, heading tại giới hạn, zero sau kick đã kích hoạt, watchdog xuyên đến
  motor, đảo chiều xuyên callback/Teleop. Mỗi vòng kiểm tra giới hạn setpoint
  và bước ramp ≤0,0125 m/s. Zero giảm về 0 theo ramp hiện có, không bị giữ
  lệnh cũ. Watchdog biên 500/501 ms và wraparound vẫn đạt.
- `make -f Makefile.linux -j4`: thành công ELF/BIN/HEX; text=262752,
  data=9484, bss=240316 byte. Còn warning return value micro-ROS và biến/hàm
  không dùng. Host test mock PID/HAL, không xác nhận tốc độ/gia tốc cơ khí,
  ổn định vòng kín hoặc thời gian dừng vật lý. Ca zero mô phỏng lệnh cuối từ
  controller, không kiểm tra việc Nav2 thực sự phát zero ở đích cuối.
- Cần bag nhiều waypoint có `/cmd_vel`, `/odomfromSTM32`, action status và
  trace MCU cùng lúc, trước/sau sửa ROS với cùng firmware, đường và tham số.
  Thu graph publisher, runtime param dump để xác định launch thực tế; gắn
  phiên bản firmware/source vào mỗi bộ dữ liệu. Giới hạn công cụ phát zero
  ở tần số hữu hạn để không tái tạo lượng traffic bất thường của bag cũ.
- Trước A/B, kê bánh kiểm tra chiều và dừng; thử timeout với L1 thả và riêng
  tình huống L1 giữ. Chạy 0,05–0,10 m/s ở khu vực trống; chỉ thử chuỗi điểm
  sau khi xác nhận zero/cancel/timeout dừng được. Không tăng tốc tối đa khi
  chưa xác nhận các ca này. Lượt hiện tại không có kiểm chứng mới trên robot.

Tái tạo phân tích offline cho bag trước/sau:

```bash
source /opt/ros/jazzy/setup.bash
python3 tools/analyze-motion-bag.py PATH_TO_BAG build-linux/diagnostics/motion-audit
```

## Kết luận và phạm vi bằng chứng

Có lỗi luồng setpoint trong source: TeleopTask (khoảng 10 ms) và
MotorControlTask (5 ms) cùng ghi DesiredSpeed. Vòng motor xen giữa đọc lại
đầu ra ramp của chính nó làm đầu vào. Ví dụ lệnh giữ 0,5 m/s, MAXACC=2,5 m/s²:
đáng lẽ tăng 0,0125 m/s mỗi 5 ms, nhưng trước sửa nhiều vòng chỉ giữ nguyên,
đợi TeleopTask ghi lại lệnh. Kick/heading cũng tác động lên giá trị đã sửa.
Đây là lỗi firmware xác định được từ code, chưa chứng minh là toàn bộ nguyên
nhân khựng tại waypoint trên robot.

Mẫu ROS thụ động ngày 2026-10-02, khoảng 12 giây (firmware đang có trên board,
chưa nạp bản sửa): `/cmd_vel` có 0 publisher, 1 subscriber `stm32_odom_node`,
BEST_EFFORT; không nhận bản tin Twist nên không đo được Hz/gap của lệnh.
`/odomfromSTM32`: 1130 bản tin, khoảng 98,41 Hz, gap nhận lớn nhất 112,23 ms,
vx và wz đều bằng 0. Không thấy `/tf` hoặc `/odom` trong ROS graph lúc kiểm tra.
Không suy ra mất cmd_vel hay lỗi waypoint từ gap odom này.
Bag: `/tmp/robot-motion-diagnostic`; log: `/tmp/cmdvel-{hz,echo,info}.log`.
Các file /tmp là dữ liệu tạm, cần lưu nơi bền vững nếu dùng để đối chiếu về sau.

## Luồng đang chạy

1. `StartMicroRosRxTask`: spin executor tối đa 1 ms, mục tiêu khoảng 10 ms,
   lấy `g_uros_mutex` tối đa 5 tick. Odom publisher dùng cùng mutex; quản lý kết
   nối cũng có thể làm gián đoạn RX. Chu kỳ task không phải tần số publisher.
2. `cmdvel_cb`: lưu raw v/w và gap nhận; deadband v=0,0005 m/s, w=0,005 rad/s;
   nâng lệnh khác zero nhỏ lên 0,05 m/s và 0,25 rad/s. Tính
   vR=v+w*0,292/2, vL=v-w*0,292/2; Spd1_cmdfromROS chứa **trái**, Spd2 chứa **phải**.
3. `StartTeleopTask`: kiểm tra age >500 ms; nếu còn ROS thì clamp mỗi bánh
   ±1 m/s, map motor1=phải, motor2=trái. Nếu timeout chuyển về manual; không
   giữ L1 thì lệnh bằng 0. Manual hiện có mapping ngược với autonomous;
   bản sửa giữ nguyên và yêu cầu thử vật lý trước khi thay đổi.
4. Trước sửa, Teleop ghi DesiredSpeed; sau sửa ghi mailbox motor_target1/2
   nguyên tử theo cặp bằng critical section. MotorControl là nơi duy nhất
   ghi setpoint tốc độ PID trong các task đang chạy.
5. MotorControl: snapshot raw → heading hold khi |vR-vL|<0,03 và |v_avg|>0,05
   → kick → một lớp ramp → DesiredSpeed1/2 → SpeedPID → Motor_SetSpeed.
   Heading PD tính mỗi 4 vòng, giờ lưu correction giữa các vòng và dùng thời
   gian tích lũy thực tế cho D. Không dùng DesiredSpeed đã ramp để xét heading.
6. PID trong main đặt sample time **5 ms**, P_ON_E; PID_Compute vẫn kiểm tra
   HAL tick và chỉ tính khi đủ 5 ms, Ki/Kd scale theo sample time cố định.
   Giá trị mặc định Kp=2500, Ki=350, Kd=0; phải xem gain thực sau khi load flash/UI.
   Có anti-windup và reset integral khi setpoint gần 0, nhưng PWM không nhất
   thiết bằng 0 ngay khi setpoint 0 vì vẫn có phản hồi tốc độ.
7. Motor_SetSpeed: dấu motor1=-1, motor2=+1, clamp ±1200, xuất DIR và CCR TIM9
   CH1/2 theo ARR thực tế. Trace PWM là lệnh có dấu trước đổi dấu driver,
   không phải số đếm CCR. Một số comment TIM1 trong motor.c đã cũ.
8. EncoderTask: encoder1 TIM3 dấu -1, encoder2 TIM4 dấu +1; count mở rộng từ
   delta int16, 28000 ticks/vòng, đường kính 0,10 m; count delta → m/s → CurSpeed.
   Độ phân giải danh định ~0,00224 m/s mỗi tick tại 5 ms. Sau sửa dùng elapsed
   thực của EncoderTask, bỏ sample dt=0. Tích phân g_theta dùng cùng dt.
9. OdometryTask độc lập mỗi 5 ms: encoder2=trái, encoder1=phải; dùng dt thực,
   tích phân pose; publisher gửi `/odomfromSTM32` khoảng 10 ms.

MotionTask, chỗ tạo task và các lời gọi MP_StepPosition/MP_StepVelocity trong
nó đều bị comment. Không bật lại chúng. Nhánh position PID vẫn tồn tại trong
MotorControl nhưng mode mặc định là velocity; không coi profile vị trí là
nguồn phát setpoint trong luồng ROS đang phân tích.

## Giả thuyết và dấu hiệu để phân biệt

| Quan sát cùng thời điểm | Giả thuyết và bước tiếp theo |
| --- | --- |
| /cmd_vel về 0, đảo dấu hoặc nhảy mạnh đúng waypoint | Xem node xuất lệnh, cách gửi goal nối tiếp, chờ kết thúc goal, thời gian dwell và điều kiện dừng/quay tại goal. Xem controller/goal checker thực tế trước khi chỉnh tolerance. Nếu muốn đi xuyên nhiều điểm, kiểm tra cách gửi đường liên tục thay vì từng goal có yêu cầu dừng. Không che lệnh zero bằng smoothing firmware. |
| /cmd_vel trên máy tính đều nhưng callback có gap >500 ms | Kiểm tra agent/reconnect, transport, QoS, tranh mutex RX/pub; đối chiếu dbg_rx_spin_err_cnt, dbg_manager_reconnect_cnt và counter callback. BEST_EFFORT có thể mất mẫu. Không tăng timeout để che mất lệnh. |
| /cmd_vel có nhiều publisher | Xác định mux/quyền điều khiển, loại trừ các nguồn luân phiên gửi zero và lệnh chạy. |
| Raw cmd liên tục nhưng setpoint thành bậc theo nhịp 10 ms | Lỗi dùng chung DesiredSpeed nói trên; so A/B trước/sau sửa với cùng lệnh. |
| v/w dao động quanh deadband | Bước nhảy 0↔0,05 m/s, 0↔0,25 rad/s có thể làm mất độ cong/dao động quanh goal. Đo tốc độ nhỏ nhất thật sự ổn định, sau đó mới giảm/bỏ boost hoặc chuyển bù ma sát có kiểm soát. |
| Tăng setpoint sau 200 ms đứng yên | Kick +0,03 m/s dựa tốc độ trung bình, không phát hiện từng bánh hay quay tại chỗ. KICK_MAX hiện clamp phần cộng, không phải tổng setpoint. Thử A/B kick sau khi đã đo, không cộng dồn vào raw target. |
| DesiredSpeed ổn nhưng PWM/encoder dao động | Kiểm tra PID saturation, gain flash thực, ma sát, nguồn/driver, nhiễu/đứt xung, dt motor/encoder và pid_updated. Tune một yếu tố mỗi lần. |
| Motor dt lệch hoặc pid_updated khác 3 thường xuyên | HAL tick và RTOS tick khác pha, task trễ hoặc bị chặn; PID còn dùng hệ số thời gian cố định. Đo trước khi đổi PID sang dt biến thiên. |

Encoder_GetCount hiện có trạng thái mutable dùng bởi EncoderTask và
OdometryTask cùng priority; nếu bị time-slice giữa thao tác đọc/cập nhật có
thể race. Nếu thấy bước nhảy count, xác minh lịch task và cân nhắc chỉ một
nơi đọc encoder rồi publish snapshot. Bản sửa này chưa thay cơ chế đó.

## Thay đổi và watchdog

- Tách raw target/setpoint, bảo vệ cặp double và ROS command/timestamp khi
  chuyển dữ liệu giữa callback và teleop; giữ correction heading riêng.
- Ramp dùng MAXACC [m/s²] * dt [s], dt=min(elapsed, 5 ms). Task trễ không
  tạo một bước bù lớn; vòng bù có elapsed=0 không tăng tốc thêm. Gia tốc không
  vượt giới hạn theo elapsed, nhưng có thể thấp hơn khi task trễ. API cũ
  Apply_Robot_Ramping vẫn giữ để tương thích, luồng motor dùng bản có dt.
- Encoder dùng dt thực; kick tính thời gian đứng yên theo elapsed.
- Giữ timeout **age >500 ms**, không phải >=500; giữ counter một lần mỗi
  lần chuyển autonomous→manual. Không đổi giao thức, dấu, mapping hay giới hạn.
- Timeout đưa raw về 0 khi manual không được enable, sau đó giảm theo ramp.
  Đây không phải emergency stop tức thời: thời gian lý tưởng thêm |v|/2,5 s
  (0,1 m/s: ~40 ms; 1 m/s: ~400 ms), cộng lịch task và đáp ứng cơ khí.
  Nếu L1 đang giữ và tay cầm hợp lệ, firmware cũ và mới đều có thể tiếp tục
  chạy manual sau mất ROS. Watchdog nằm ở TeleopTask nên task này bị treo
  có thể trì hoãn dừng. Cần thử các tình huống này, không tuyên bố fail-safe
  phần cứng từ kiểm thử phần mềm.

## Thu dữ liệu khi tái hiện

Chạy các terminal riêng, cùng ROS_DOMAIN_ID/RMW với hệ thống thật:

```bash
ros2 topic list
ros2 topic info /cmd_vel --verbose
ros2 topic hz /cmd_vel
ros2 topic echo /cmd_vel
# Chỉ thêm /tf và /odom nếu topic có trong hệ thống:
ros2 bag record --topics /cmd_vel /odomfromSTM32 /tf /odom
```

Twist không có header: dùng timestamp nhận của bag, không coi đó là thời
điểm publisher tạo lệnh. So raw v/w với twist odom và tính lại
vR=vx+wz*0,292/2, vL=vx-wz*0,292/2; lưu ý phản hồi cơ khí có độ trễ.
Odom publisher có thể trễ riêng so với encoder nên cần trace MCU bổ sung.

Trace `dbg_motor_trace[128]` ghi mỗi vòng motor velocity, ~640 ms lịch sử tại
5 ms, tốn 11264 byte RAM. `dbg_motor_trace_write_count` là số mẫu đã ghi,
slot kế tiếp là count%128, xem sequence để sắp thứ tự. Đặt
`dbg_motor_trace_freeze=1` qua debugger rồi xuất; muốn tiếp tục đặt về 0.
Không halt CPU khi robot đang chạy: halt làm sai watchdog/timing.
Trace không thêm topic/UART; để bắt đúng sự kiện ngắn cần live watch/logger
qua debug hoặc freeze gần lúc khựng. Chưa có trigger tự động hay logger dài hạn.

Mỗi mẫu gồm thời gian RTOS ms, dt motor/encoder, raw1/2, DesiredSpeed1/2,
CurSpeed1/2 gần nhất, PWM1/2, raw cmd v/w, số callback, age lệnh, gap gần nhất,
gap lớn nhất, số gap >500, số timeout, mode và bitmask PID đã tính
(bit0=motor1, bit1=motor2). Gap chỉ cập nhật khi có callback kế tiếp; age
và timeouts cần xem khi mất hẳn kết nối. Encoder là mẫu gần nhất, không phải
hai phép đo thực hiện ngay tại thời điểm ghi trace.

Thời gian trace tính từ boot, bag dùng clock máy tính: căn theo chuỗi thay
đổi lệnh nhận được và cmd_count; chưa có đồng bộ timestamp MCU↔bag chính xác.
Để soi timing, xem cả dbg_motor_dt_max_ms và dbg_encoder_dt_max_ms.

## Kiểm thử và chạy robot

```bash
python3 tests/test_motor_control.py
make -f Makefile.linux -j4
```

Host test trích hàm C đang dùng từ source và biên dịch với scheduler/HAL/PID
mock: lệnh giữ tăng đều mỗi 5 ms; đảo chiều/về zero; heading giữ correction;
task trễ không gây bước ramp lớn; watchdog 500/501 ms và wraparound.
Không kiểm thử đặc tính PID, transport hoặc thời gian thực trên MCU bằng mock.

Đã qua 6 kịch bản; firmware link ELF/BIN/HEX thành công. Build còn warning
sẵn có về return value micro-ROS và hàm/biến không dùng. Chưa nạp bản sửa.

Thử theo thứ tự: kê bánh kiểm tra chiều/map, kiểm tra dừng; trên mặt phẳng
trống chạy 0,05–0,10 m/s với người sẵn sàng ngắt động lực; lệnh giữ → zero →
đảo chiều chậm → đường cong → nhiều waypoint. Ghi bag trước lúc phát lệnh và
sau lúc khựng. Kiểm tra mất publisher/agent/kết nối ở tốc độ thấp, L1 thả;
xác nhận timeout tăng và setpoint/encoder trở về zero. Thử riêng tình huống
chuyển về manual với L1. Không tăng tốc bình thường trước khi các ca này đạt.

Rủi ro bản sửa: ramp giờ chạy đủ 200 Hz nên tăng/giảm tốc nhanh hơn trước,
heading giữ liên tục có thể làm gain hiện tại mạnh hơn; dt encoder thật có
thể làm lộ jitter trước đây bị che; trace thêm RAM và thời gian ghi. Kiểm tra
stack high-water của các task 1 KB và dt trên board trước khi chạy dài.
Nếu cần jerk limit sau khi đo, tích hợp vào chính ramp hiện tại với đơn vị
m/s³; không xếp thêm nhiều lớp filter và không bật profile vị trí.

## Kiểm tra lại sau báo cáo hồi quy (2026-10-02)

Người dùng đã nạp qua tools/flash-linux.sh, báo robot lạng và RViz chậm khi
Fixed Frame=map. Khi đo lại, STM32 bật nhưng RViz đã tắt: **không coi mẫu này
là tái hiện lỗi đang chạy**. Chưa xác định được nguyên nhân gốc của hồi quy;
không khuyến nghị giữ bản sửa chỉ vì build/host test đạt.

Đo thụ động 15 giây:

| Topic | Hz | Gap nhận lớn nhất | Tuổi header trung vị |
| --- | ---: | ---: | ---: |
| /front_lidar/scan | 9,92 | 202 ms | 99,5 ms |
| /rear_lidar/scan | 10,00 | 103 ms | 99,2 ms |

Ranges thay đổi ở mọi cặp scan liên tiếp; không thấy timestamp lặp. Không
nhận odom STM32. Đo TF thêm 12 giây thấy các cạnh bánh xe và lidar calibration,
không nhận map→odom hoặc odom→base_link; /odomfromSTM32 không có publisher.
EKF trong workspace ROS dùng odom0=/odomfromSTM32 và publish_tf=true.
Do đó đường dữ liệu odom/TF đang thiếu trong mẫu hiện tại; scan vẫn chạy không
đảm bảo RViz vẽ được trong frame map. Chưa chứng minh trạng thái này tồn tại
đúng lúc người dùng thấy chậm.

Đọc ST-Link không halt/reset/nạp, đối chiếu 272188 byte đầu flash khớp binary
build-linux hiện tại. Snapshot RAM khi uptime khoảng 23 giây:

- g_uros_connected=0, g_request_reconnect=0, dbg_pub_odom_cnt=0;
- dbg_cmd_cb_count=0, num_timeout_ROScmd=0;
- dbg_motor_dt_max_ms=5, dbg_encoder_dt_max_ms=5;
- trace mới nhất đọc được: raw/setpoint/encoder/PWM đều zero, pid_updated=3;
- FreeRTOS heap free/min khoảng 46248 byte;
- gnetif.ip_addr=192.168.4.50, flags=0x0b, không có NETIF_FLAG_LINK_UP (0x04).
  Offset flags=45 đã đối chiếu DWARF của đúng ELF.

RAM đọc khi CPU chạy không phải snapshot nguyên tử; thời gian/counter có thể
lệch giữa các vùng đọc. Link DOWN không tự chứng minh đứt dây: cần xác nhận
cáp/đèn PHY và trạng thái driver. Các số liệu này không chứng minh CPU bị quá
 tải, cũng không loại trừ tải/jitter khi thực sự có ROS traffic và chuyển động.

Điểm cần rút lại trong đánh giá bản sửa: tách raw target khiến ramp tăng ở
mỗi vòng 5 ms thay vì thường chờ lệnh ghi lại khoảng 10 ms; giữ heading correction
làm thay đổi tác động vòng lái. Đây là thay đổi động học điều khiển, không phải
chỉ thêm chẩn đoán. Host test mock PID không kiểm tra ổn định hệ robot.
Joystick đi thẳng còn ROS lạng là manh mối: đường ROS có deadband/minimum boost
v/w và controller ngoài, nên chưa đủ để kết luận lỗi cơ khí/PID hoặc đảo bánh.

Hướng xử lý khi chưa có mẫu lỗi: dùng bản trước làm baseline; không thêm tiếp
filter/gain hoặc nạp bản sửa mới theo phỏng đoán. Bản backup trước lần flash
16:42 là build-linux/backups/stm32-20261002-164201.bin; chưa kiểm chứng nó là
bản chạy tốt. File đó chứa cả sector PID, không ghi đè toàn bộ 1 MB để rollback
code. Đã lưu diff của thay đổi tại
build-linux/diagnostics/waypoint-regression/previous-change.patch; chưa áp dụng
rollback, chưa thay firmware hoặc cấu hình ROS trong lần kiểm tra lại này.

Cập nhật ngay trong lần kiểm tra: người dùng xác nhận dây Ethernet vừa bị tuột
và đã cắm lại. **Không dùng quan sát link DOWN/thiếu odom nói trên làm bằng
chứng lỗi bản sửa.** Sau khi cắm lại và board khởi động lại, RAM báo flags=0x0f,
g_uros_connected=1, dbg_pub_odom_cnt=1315, motor/encoder dt_max vẫn 5 ms.
Lần đo ROS cuối nhận 1496 odom trong khoảng 15 giây: 100,003 Hz, gap lớn nhất
11,29 ms, tuổi header trung vị 0,625 ms, lớn nhất 1,490 ms, timestamp không lặp.
Robot đứng yên; mẫu cuối chỉ nhận odom, không có scan/TF, nên không đối chiếu
được toàn chuỗi đồng thời. Chưa đo lại trong tình huống di chuyển/RViz có lỗi.

Kết luận kiểm tra lại: không chứng minh được firmware mới làm chậm cập nhật,
cũng chưa xác định nguyên nhân chính xác robot lạng. Không có căn cứ yêu cầu
người dùng giữ bản sửa. Khuyến nghị khôi phục bản trước làm baseline, rồi chỉ
thay một yếu tố mỗi lần nếu tiếp tục A/B. Trong lượt này chỉ cập nhật tài liệu
và lưu công cụ/diff chẩn đoán; không sửa thêm code điều khiển hoặc nạp board.
