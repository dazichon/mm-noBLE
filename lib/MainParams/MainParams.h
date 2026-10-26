#pragma once

// ###########################################################################################################
// #  1. PID ĐI THẲNG  (Encoder + Gyro + Wall). Cả ba chỉ có Kp, tổng correction = enc + yaw + wall           #
// ###########################################################################################################

// ----- 1a. ENCODER PID: cân 2 bánh -----
#define ENC_STRAIGHT_KP 0.13f // PWM hiệu chỉnh = KP * (|ENC_L| - |ENC_R|) [XUNG, tích lũy]. Đơn vị PWM/xung: 0.05 -> lệch 20 xung = 1 PWM. KHÔNG dùng số lớn (1.8 = 1.8 PWM cho mỗi xung lệch -> bão hòa)
#define ENC_MAX_CORR    10.0f // Hiệu chỉnh tối đa (PWM) cho mỗi bánh, để encoder chỉ cân tốc độ 2 bánh

// ----- 1b. GYRO PID: giữ hướng -----
#define HOLD_YAW_KI 3.0f       // Khâu I giữ hướng: xe vẫn trôi dần 1 bên khi chạy lâu thì tăng, lắc qua lại thì giảm. 0: tắt
#define HOLD_YAW_KP 2.0f       // Giữ thẳng hướng bằng gyro. Tăng: xe sửa lệch hướng mạnh hơn, quá cao thì ngoằn ngoèo. 0: tắt
                              // (giới hạn correction yawMaxCorr = 20 PWM nằm trong RobotNav.h)

// ----- 1c. WALL PID: sửa lệch ngang theo VL53 trái/phải -----
#define WALL_ENABLE    1       // 1: cộng wall correction nhỏ vào correction tổng (Encoder PID + yaw vẫn chạy). 0: tắt
#define WALL_KP        0.5f    // PWM cho mỗi mm lệch so với baseline. 10mm lệch -> 3 PWM
#define WALL_MAX_CORR  8.0f    // Giới hạn |wall correction| (PWM) - nhỏ hơn yaw (20) và encoder (10)
#define WALL_YAW_GAIN  0.15f   // ĐỘ dịch hướng gyro cho mỗi mm lệch ngang (10mm -> 1.5 độ). Xe về giữa chậm thì tăng, lắc qua lại thì giảm. 0: dùng WALL_KP cộng PWM kiểu cũ
#define WALL_YAW_MAX   5.0f    // Độ dịch hướng tối đa
#define WALL_BASE_L    173.0f  // VL53 trái khi xe ở giữa (mm)
#define WALL_BASE_R    183.0f  // VL53 phải khi xe ở giữa (mm)
#define WALL_VALID_MAX 195.0f  // Đọc xa hơn mức này (mm) = hở tường / nhìn sang ô kế tiếp -> bỏ qua bên đó

// ----- 1d. VL53 trước -----
#define HOLD_USE_FRONT    0       // 0: không dùng VL53 trước (đang trả -3), không phanh theo tường trước
#define HOLD_TARGET_FRONT 150.0f  // Tăng: xe dừng xa tường trước hơn. Giảm: xe dừng sát tường trước hơn
#define VL53_FRONT_SIGNAL_LIMIT 0.5f // MCPS: VL53 trước bỏ phản xạ yếu hơn mức này (mặc định chip 0.25). Phía trước trống mà vẫn báo ~85mm thì tăng (0.75, 1.0); tường thật gần không đọc được thì giảm. 0: giữ mặc định

// ###########################################################################################################
// #  2. CÂN BẰNG 2 ĐỘNG CƠ (áp dụng CHUNG cho mọi tốc độ & mọi chế độ)                                      #
// ###########################################################################################################
// Đơn vị %: công suất bánh = PWM tính ra * (1 + giá trị/100), nên bù cùng tỉ lệ ở mọi tốc độ.
// Xe lệch sang PHẢI (bánh trái mạnh hơn): giảm MOTOR_TRIM_LEFT (số âm) hoặc tăng MOTOR_TRIM_RIGHT.
// Xe lệch sang TRÁI (bánh phải mạnh hơn): ngược lại. Ví dụ -5 = bánh đó yếu đi 5%.
#define MOTOR_TRIM_LEFT   -6.0f   // % bù bánh trái. Tăng: bánh trái mạnh hơn. Giảm (âm): bánh trái yếu đi
#define MOTOR_TRIM_RIGHT  0.0f   // % bù bánh phải. Tăng: bánh phải mạnh hơn. Giảm (âm): bánh phải yếu đi
#define FORWARD_PWM       180    // PWM chạy thẳng (dò đường). 0: dùng giá trị lưu trong xe (lệnh SET_FSPD, đang là 150). Tăng: nhanh hơn, khó giữ thẳng hơn
#define ACCEL_START_PWM   60.0f  // PWM lúc xuất phát từ đứng yên (đủ để xe lăn, không bốc đầu)
#define ACCEL_RAMP_MS     400    // ms tăng dần từ ACCEL_START_PWM lên tốc độ chạy thẳng. Vẫn bốc đầu thì tăng. 0: tắt

// ###########################################################################################################
// #  3. CHIỀU DÀI Ô & SỐ Ô CHẠY TEST                                                                        #
// ###########################################################################################################
#define CELL_PULSES      4400   // Số xung encoder của 1 ô (đổi số này khi đo lại quãng đường 1 ô)
#define TEST_CELLS       1      // Số ô chạy trong bài test (tổng xung = CELL_PULSES * TEST_CELLS)
#define SEQ_CELL_PULSES  4400   // Xung/ô trong sequence - giá trị TẠM để test, chưa calib
#define CALIB_REV_PWM        80     // Calib ô (chế độ 2): PWM khi lùi sát vách cụt. Tăng: chạm vách mạnh hơn
#define CALIB_REV_MS         600    // Calib ô: thời gian lùi cố định (ms), rồi dừng, calib gyro về 0 độ
#define CALIB_FWD_PULSES     1500   // Calib ô: xung tiến từ vách tới tâm ô (không tính vào ô kế tiếp)
#define NAV_SIDE_WALL_MM  220   // Chế độ 6: VL53 trái/phải đọc nhỏ hơn mức này (mm) = có tường. Có tường ~170-185, thông >= ~250
#define NAV_FRONT_WALL_MM 150   // Chế độ 6: VL53 trước đọc nhỏ hơn mức này (mm, xe đứng giữa ô) = tường chắn NGAY cuối ô hiện tại (đo ~113-153). Tường cách 1 ô đọc ~200-260 -> phải nằm giữa 2 dải
#define NAV_MAX_STEPS     30    // Chế độ 6: số ô đi tối đa rồi dừng
#define NAV_START_SIDES_WALL 1  // Chế độ 6: ô xuất phát chưa có lần đọc nhìn-trước. 1: coi như có tường trái/phải (quy ước ô xuất phát). 0: không biết (nếu trước bị chặn thì dừng, không quay)
#define STOP_LEAD_REF_PWM 150  // PWM lúc đo STOP_LEAD_PULSES; chạy nhanh hơn tự phanh sớm hơn theo (PWM/150)^2
#define STOP_LEAD_PULSES 280   // Chỉ chế độ 1 (cell-to-cell): phanh sớm chừng này xung trước đích để bù quãng xe trôi sau phanh (đo ~280). 0: tắt

// ----- Flood-fill (chế độ 4, đi tới đích) -----
#define FF_CONTINUOUS 1   // 1: đi LIÊN TỤC (gộp các ô thẳng đã biết vách thành 1 lần chạy, không dừng từng ô). 0: dừng ở từng ô
#define FF_MAX_RUN    16  // Số ô tối đa gộp trong 1 lần chạy liên tục

// ###########################################################################################################
// #  4. CHỌN CHẾ ĐỘ TEST (bấm nút BUTTON, chờ TURN_DELAY_MS rồi chạy)                                       #
// ###########################################################################################################
#define TURN_DELAY_MS    2000   // Chờ sau khi bấm nút
#define SEQ_MODE          11   // 11: DO TOI DICH -> DO VE XUAT PHAT -> SPEEDRUN DUONG NGAN NHAT. 10: NHU 8 NHUNG CHAY LIEN TUC qua o thang, chi dung de quay. 8: DO DUONG + VE BAN DO TUONG (xem muc 9 cuoi file). 7: chạy kichBan() trong main.cpp (viết bằng lib/Motion/Motion.h). 6: tự đi từng ô (trước thông thì thẳng; trước có tường thì trái nếu thông, phải nếu thông; cụt thì quay 180 + calib ô). 5: chỉ đọc 3 VL53L0X, ghi 100 mẫu (~30Hz) vào RAM rồi in ra khi cắm USB, KHÔNG chạy motor. 2: ngõ cụt (1 ô -> dừng -> quay 180 -> lùi sát vách -> tiến CALIB_FWD_PULSES tới tâm ô). 1: cell-to-cell (1 ô -> dừng -> trái 90 -> 1 ô -> dừng -> phải 90 -> 1 ô -> dừng). 4: test CORNER state machine (1 ô -> cua trái -> 1 ô -> STOP). 3: calibration encoder (chạy thẳng 1 ô, in ENC_L/ENC_R/AVG_TICKS/TICKS_PER_CELL). 0: chỉ quay tại chỗ TURN_ANGLE

// ###########################################################################################################
// #  5. CUA KHI ĐANG CHẠY (Corner Controller)                                                               #
// ###########################################################################################################
#define CORNER_SPEED       60.0f // PWM bánh ngoài khi cua (cua chậm)
#define CORNER_INNER_RATIO 0.45f // PWM bánh trong / bánh ngoài ở độ cong tối đa. Nhỏ hơn = cua gắt hơn
#define CORNER_DECEL_ZONE  30.0f // độ cuối: độ cong giảm dần về 0
#define CORNER_TOLERANCE   2.0f  // coi là tới target khi sai số yaw <= độ này
#define CORNER_LEAD        0.05f // s: giảm cong sớm theo tốc độ quay, chống overshoot

// ###########################################################################################################
// #  6. QUAY TẠI CHỖ                                                                                        #
// ###########################################################################################################
#define TURN_ANGLE       90.0f  // Góc quay (độ): 90 hoặc 180
#define TURN_RIGHT       1      // 1: bấm nút quay PHẢI. 0: quay TRÁI
#define TURN_MAX_PWM     80.0f  // PWM lớn nhất khi quay
#define TURN_MIN_PWM     50.0f  // PWM nhỏ nhất khi gần target (đủ thắng ma sát)
#define TURN_DECEL_ZONE  35.0f  // Giảm tốc tuyến tính trong ngần này độ cuối
#define TURN_TOLERANCE   1.5f   // Dừng khi sai số yaw <= độ này
#define TURN_BRAKE_LEAD  0.05f  // s: phanh sớm = tốc độ quay (độ/s) * giá trị này. Tăng nếu vẫn quay quá, giảm nếu quay thiếu
#define TURN_LEFT_YAW_SIGN -1.0f // Dấu thay đổi yaw khi quay TRÁI. Log đi thẳng: lệch phải => yaw tăng => quay trái làm yaw giảm (-1). Nếu quay sai chiều, đổi thành +1

// ###########################################################################################################
// #  7. KHỞI ĐỘNG                                                                                           #
// ###########################################################################################################
#define HOLD_AUTO_START     0     // 1: tự chạy giữ thẳng sau HOLD_START_DELAY_MS khi bật nguồn; 0: chờ lệnh "HOLD" qua Serial ("STOP" để dừng)
#define HOLD_START_DELAY_MS 3000  // Thời gian chờ sau khi bật nguồn trước khi PID chạy (ms). Tăng: có nhiều thời gian đặt xe hơn

// ###########################################################################################################
// #  8. TEST LỆCH 2 ĐỘNG CƠ (open-loop, tắt PID)                                                            #
// ###########################################################################################################
// 1: TẮT PID, cấp cùng 1 PWM cho 2 bánh chạy thẳng TEST_PULSES xung rồi dừng, in chênh lệch encoder. 0: chạy bình thường
#define TEST_MOTOR_DRIVE 0
#define TEST_PWM    100    // PWM cấp cho cả 2 bánh khi test (đã nhân MOTOR_TRIM). Đổi số này để test ở tốc độ khác
#define TEST_PULSES 9000   // Dừng khi trung bình xung 2 bánh đạt giá trị này

// ###########################################################################################################
// #  9. DÒ ĐƯỜNG + VẼ BẢN ĐỒ (SEQ_MODE 8, lib/Mapping)                                                       #
// ###########################################################################################################
// Đặt xe ở ô (0,0) góc dưới trái, quay mặt lên (Bắc). Ngưỡng tường dùng NAV_SIDE_WALL_MM / NAV_FRONT_WALL_MM.
#define MAP_COLS       16   // số cột mê cung thật (<= 16)
#define MAP_ROWS       16  // số hàng mê cung thật (<= 16)
#define MAP_GOAL_X     7    // góc dưới trái vùng đích
#define MAP_GOAL_Y     7
#define MAP_GOAL_SIZE  2    // 1: đích 1 ô, 2: đích 2x2
#define MAP_SIDE_FROM  0.35f // đọc vách hông khi xe đã đi được từ 35% ...
#define MAP_SIDE_TO    0.75f // ... tới 75% quãng đường của ô. Tường bị nhận nhầm/bỏ sót thì dịch cửa sổ này
#define MAP_CALIB_BACK 1     // 1: hễ biết chắc sau lưng có tường thì lùi kịch tường calib (CALIB_REV_*, CALIB_FWD_PULSES). 0: tắt
#define MAP_ALIGN_FRONT 1    // 1: trước mặt có tường thì tiến/lùi căn đúng khoảng cách FRONT_ALIGN_MM rồi mới quay. 0: tắt
#define FRONT_ALIGN_MM  95.0f  // VL53 trước đọc bao nhiêu (mm) khi xe ĐÚNG TÂM Ô có tường trước -> đặt xe giữa ô, đọc rồi ghi vào đây
#define FRONT_ALIGN_SKIP 18.0f // mm: lệch ít hơn mức này thì KHÔNG căn (căn tường trước chỉ là phụ). Xe vẫn căn nhiều quá thì tăng
#define FRONT_ALIGN_TOL 10.0f  // mm: khi đã căn, vào được trong khoảng này là dừng
#define FRONT_ALIGN_MIN_PWM 45.0f // PWM khởi đầu khi căn; đứng im thì tự tăng dần tới 70
#define FRONT_ALIGN_KP  0.3f   // PWM / mm. Tăng: căn nhanh hơn, quá cao thì tiến lùi qua lại
#define MAP_SPEEDRUN_PWM 210  // SEQ_MODE 11: PWM chạy thẳng lúc speedrun (vd 180). 0: giữ tốc độ thường
#define MAP_MAX_CELLS  400  // số ô đi tối đa rồi dừng (an toàn)
