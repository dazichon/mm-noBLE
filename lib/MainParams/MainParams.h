#pragma once

// THÔNG SỐ XE - chỉ sửa giá trị ở cột giữa, giữ nguyên tên #define.
// Tìm nhanh bằng Ctrl+F: tên thông số hoặc số mục 01..14.
// Đơn vị: PWM = mức điều khiển motor (0-255); ms = mili giây; mm = milimét; xung = xung encoder.
//
//   01. Chế độ chạy & khởi động        08. Tường trước: dừng & căn
//   02. Động cơ & tốc độ               09. Tường sau: lùi calib
//   03. Quãng đường 1 ô & phanh        10. Quay tại chỗ
//   04. Giữ thẳng - encoder            11. Cua khi đang chạy
//   05. Giữ thẳng - gyro               12. Bản đồ & đích
//   06. Bám tường hông                 13. Chế độ 6 & flood-fill
//   07. Cảm biến & ngưỡng nhận tường   14. Test riêng 2 động cơ
//                                      15. In ra terminal

// ============================================================================
// 01. CHẾ ĐỘ CHẠY & KHỞI ĐỘNG
// ============================================================================
// SEQ_MODE: 0  = chỉ quay tại chỗ TURN_ANGLE (mục 10)
//           1  = cell-to-cell: 1 ô -> dừng -> trái 90 -> 1 ô -> dừng -> phải 90 -> 1 ô -> dừng
//           2  = ngõ cụt: 1 ô -> dừng -> quay 180 -> lùi sát vách -> tiến CALIB_FWD_PULSES tới tâm ô
//           3  = calib encoder: chạy thẳng, in ENC_L/ENC_R/AVG_TICKS/TICKS_PER_CELL
//           4  = test cua khi chạy: 1 ô -> cua trái -> 1 ô -> dừng
//           5  = chỉ đọc 3 VL53L0X, ghi 100 mẫu (~30Hz) rồi in khi cắm USB, KHÔNG chạy motor
//           6  = tự đi từng ô (trước thông thì thẳng; có tường thì rẽ bên thông; cụt thì quay 180 + calib)
//           7  = chạy kichBan() trong src/main.cpp (viết bằng lib/Motion/Motion.h)
//           8  = dò đường + vẽ bản đồ, dừng ở từng ô
//           10 = như 8 nhưng chạy liên tục qua ô thẳng, chỉ dừng để quay
//           11 = dò tới đích -> dò về xuất phát -> speedrun đường ngắn nhất

#define SEQ_MODE                11        // Chọn chế độ theo bảng trên
#define TURN_DELAY_MS           2000      // ms: chờ sau khi bấm nút rồi mới chạy
#define HOLD_AUTO_START         0         // 1: tự chạy giữ thẳng sau HOLD_START_DELAY_MS khi bật nguồn; 0: chờ lệnh "HOLD" qua Serial ("STOP" để dừng)
#define HOLD_START_DELAY_MS     2000      // ms: chờ sau khi bật nguồn trước khi PID chạy. Tăng: có nhiều thời gian đặt xe hơn

// ============================================================================
// 02. ĐỘNG CƠ & TỐC ĐỘ
// ============================================================================
// Bù động cơ (%, áp dụng chung mọi tốc độ & mọi chế độ): PWM thực = PWM tính ra * (1 + TRIM/100).
// Xe lệch PHẢI (bánh trái mạnh hơn): giảm MOTOR_TRIM_LEFT (số âm) hoặc tăng MOTOR_TRIM_RIGHT. Lệch TRÁI: ngược lại.

#define MOTOR_TRIM_LEFT         -6.0f     // % bù bánh trái. Tăng: bánh trái mạnh hơn. Âm: yếu đi
#define MOTOR_TRIM_RIGHT        0.0f      // % bù bánh phải. Tăng: bánh phải mạnh hơn. Âm: yếu đi
#define FORWARD_PWM             180       // PWM chạy thẳng khi dò đường. 0: dùng giá trị lưu trong xe (lệnh SET_FSPD). Tăng: nhanh hơn, khó giữ thẳng hơn
#define MAP_SPEED_TURN_COST     1.0f      // Speedrun (SEQ_MODE 11): mỗi lần quay 90 độ tốn ngang chừng này ô (quay đầu x2). Tăng: ưu tiên đường ít rẽ. 0: chỉ tính số ô
#define MAP_SPEEDRUN_PWM        210       // PWM chạy thẳng lúc speedrun (SEQ_MODE 11). 0: giữ FORWARD_PWM
#define ACCEL_START_PWM         60.0f     // PWM lúc xuất phát từ đứng yên (đủ để xe lăn, không bốc đầu)
#define ACCEL_RAMP_MS           400       // ms: tăng dần từ ACCEL_START_PWM lên tốc độ chạy thẳng. Vẫn bốc đầu thì tăng. 0: tắt

// ============================================================================
// 03. QUÃNG ĐƯỜNG 1 Ô & PHANH
// ============================================================================
#define CELL_PULSES             4100      // xung encoder của 1 ô (180mm). Đổi khi đo lại quãng đường 1 ô
#define SEQ_CELL_PULSES         4100      // xung/ô cho các bài test sequence (chế độ 6)
#define TEST_CELLS              1         // số ô chạy trong bài test (tổng xung = CELL_PULSES * TEST_CELLS)
#define STOP_LEAD_PULSES        280       // xung: phanh sớm trước đích để bù quãng trôi sau phanh. Dùng ở chế độ 1, 2, 6, 8, 10, 11. 0: tắt
#define STOP_LEAD_REF_PWM       150       // PWM lúc đo STOP_LEAD_PULSES; chạy nhanh hơn thì tự phanh sớm hơn theo (PWM/150)^2

// ============================================================================
// 04. GIỮ THẲNG - ENCODER (cân tốc độ 2 bánh)
// ============================================================================
// Tổng hiệu chỉnh khi đi thẳng = encoder (mục 04) + gyro (mục 05) + tường hông (mục 06).

#define ENC_STRAIGHT_KP         0.13f     // PWM hiệu chỉnh = KP * (|ENC_L| - |ENC_R|) [xung, tích lũy]. 0.05 -> lệch 20 xung = 1 PWM. KHÔNG dùng số lớn (1.8 -> bão hòa)
#define ENC_MAX_CORR            10.0f     // PWM: hiệu chỉnh tối đa mỗi bánh, để encoder chỉ cân 2 bánh, không bẻ lái

// ============================================================================
// 05. GIỮ THẲNG - GYRO (giữ hướng)
// ============================================================================
// Giới hạn hiệu chỉnh yawMaxCorr = 20 PWM nằm trong lib/Navigation/RobotNav.h.

#define HOLD_YAW_KP             2.0f      // Tăng: sửa lệch hướng mạnh hơn, quá cao thì ngoằn ngoèo. 0: tắt
#define HOLD_YAW_KI             3.0f      // Khâu I: xe vẫn trôi dần 1 bên khi chạy lâu thì tăng, lắc qua lại thì giảm. 0: tắt

// ============================================================================
// 06. BÁM TƯỜNG HÔNG (sửa lệch ngang theo VL53 trái/phải)
// ============================================================================
// WALL_YAW_GAIN khác 0: dịch hướng đích của gyro; bằng 0: dùng WALL_KP cộng thẳng vào PWM.

#define WALL_ENABLE             1         // 1: bật bám tường hông. 0: tắt
#define WALL_BASE_L             173.0f    // mm: VL53 trái đọc khi xe ở giữa ô
#define WALL_BASE_R             183.0f    // mm: VL53 phải đọc khi xe ở giữa ô
#define WALL_VALID_MAX          195.0f    // mm: đọc xa hơn mức này = hở tường / nhìn sang ô kế tiếp -> bỏ qua bên đó
#define WALL_YAW_GAIN           0.15f     // độ dịch hướng gyro cho mỗi mm lệch (10mm -> 1.5 độ). Về giữa chậm thì tăng, lắc thì giảm. 0: dùng WALL_KP
#define WALL_YAW_MAX            5.0f      // độ: dịch hướng tối đa
#define WALL_KP                 0.5f      // PWM/mm khi dùng cách cộng PWM (WALL_YAW_GAIN = 0)
#define WALL_MAX_CORR           8.0f      // PWM: giới hạn hiệu chỉnh tường khi cộng PWM (nhỏ hơn gyro 20 và encoder 10)

// ============================================================================
// 07. CẢM BIẾN & NGƯỠNG NHẬN TƯỜNG
// ============================================================================
// Ngưỡng nhận tường (có/không) khác với khoảng cách căn tâm ô (mục 08).

#define NAV_SIDE_WALL_MM        220       // mm: VL53 trái/phải đọc nhỏ hơn mức này = có tường. Có tường ~170-185, thông >= ~250
#define NAV_FRONT_WALL_MM       150       // mm: VL53 trước (xe đứng giữa ô) nhỏ hơn mức này = tường ngay cuối ô hiện tại (đo ~113-153). Tường cách 1 ô đọc ~200-260
#define VL53_FRONT_SIGNAL_LIMIT 0.5f      // MCPS: VL53 trước bỏ phản xạ yếu hơn mức này (mặc định chip 0.25). Trống mà vẫn báo ~85mm thì tăng (0.75, 1.0); tường gần không đọc được thì giảm. 0: mặc định
#define HOLD_USE_FRONT          0         // Chế độ HOLD: 1 = phanh theo tường trước; 0 = không dùng VL53 trước (không ảnh hưởng dò đường)
#define HOLD_TARGET_FRONT       150.0f    // mm: chế độ HOLD dừng cách tường trước chừng này. Tăng: dừng xa hơn

// ============================================================================
// 08. TƯỜNG TRƯỚC: DỪNG & CĂN TÂM Ô
// ============================================================================
// FRONT_ALIGN_MM dùng chung cho cả 2 cách: đặt xe ĐÚNG TÂM Ô có tường trước, đọc VL53 trước rồi ghi vào đây.
// a) Dừng theo tường khi đang chạy (không chạm): bỏ sai số xung cộng dồn khi đi thẳng dài.
// b) Căn lại khi đã đứng yên: tiến/lùi tới FRONT_ALIGN_MM rồi mới quay.

#define FRONT_ALIGN_MM          110.0f     // mm: VL53 trước đọc khi xe ĐÚNG TÂM Ô có tường trước

// a) Dừng theo tường trước khi đang chạy
#define FRONT_STOP_ENABLE       1         // 1: tới ô có tường trước thì dừng khi VL53 trước = FRONT_ALIGN_MM, không chỉ đếm xung. 0: tắt
#define FRONT_STOP_MAX_MM       300.0f    // mm: chỉ dùng VL53 trước khi đọc <= mức này (đọc xa dao động nhiều)
#define FRONT_STOP_MAX_EXTEND   0.2f      // điểm dừng theo tường được XA hơn encoder tối đa phần này của ô. Xa hơn = tường của ô kế tiếp -> bỏ (chống chạy lố 1 ô)
// Bộ lọc chống tường giả (VL53 trước thỉnh thoảng báo ~150-220mm khi phía trước trống):
#define FRONT_STOP_SAMPLES      5         // số mẫu gần LIÊN TIẾP cần có (3..8). Mẫu -3/lỗi xen giữa thì đếm lại. Tăng: khó nhận nhầm hơn, bắt tường muộn hơn
#define FRONT_STOP_CONSIST_MM   20.0f     // mm: tường thật thì (số đo + quãng xe đã đi) gần như không đổi; lệch quá mức này = mẫu giả. Hay bỏ sót tường thật thì tăng
#define FRONT_STOP_MIN_MOVE_MM  15.0f     // mm: xe phải đi ít nhất chừng này trong các mẫu đó mới kiểm tra được

// b) Căn lại khi đứng yên
#define MAP_ALIGN_FRONT         1         // 1: có tường trước thì căn tới FRONT_ALIGN_MM rồi mới quay. 0: tắt
#define FRONT_ALIGN_SKIP        18.0f     // mm: lệch ít hơn mức này thì KHÔNG căn. Xe vẫn căn nhiều quá thì tăng
#define FRONT_ALIGN_TOL         10.0f     // mm: khi đang căn, vào được trong khoảng này là dừng
#define FRONT_ALIGN_MIN_PWM     40.0f     // PWM khởi đầu khi căn; đứng im thì tự tăng dần tới 55 (gán trong App.cpp)
#define FRONT_ALIGN_KP          0.2f      // PWM/mm. Tăng: căn nhanh hơn, quá cao thì tiến lùi qua lại

// ============================================================================
// 09. TƯỜNG SAU: LÙI KỊCH TƯỜNG, CALIB/RESET GYRO, TIẾN VỀ TÂM Ô
// ============================================================================
// Gyro chỉ calib ở lần lùi đầu tiên sau khi bật nguồn; các lần sau chỉ đặt yaw = 0.

#define MAP_CALIB_BACK          1         // 1: biết chắc sau lưng có tường thì lùi kịch tường calib. 0: tắt
#define CALIB_REV_PWM           70        // PWM khi lùi sát vách. Tăng: chạm vách mạnh hơn
#define CALIB_REV_MS            400       // ms: thời gian lùi tối đa (dừng sớm khi encoder đứng yên = đã chạm vách)
#define CALIB_FWD_PULSES        1100      // xung: tiến từ vách tới tâm ô (không tính vào ô kế tiếp)

// ============================================================================
// 10. QUAY TẠI CHỖ (gyro MPU6050)
// ============================================================================
#define TURN_MAX_PWM            80.0f     // PWM lớn nhất khi quay
#define TURN_MIN_PWM            50.0f     // PWM nhỏ nhất khi gần target (đủ thắng ma sát)
#define TURN_DECEL_ZONE         35.0f     // độ: giảm tốc tuyến tính trong ngần này độ cuối
#define TURN_TOLERANCE          1.5f      // độ: dừng khi sai số yaw <= giá trị này
#define TURN_BRAKE_LEAD         0.05f     // s: phanh sớm = tốc độ quay (độ/s) * giá trị này. Vẫn quay quá thì tăng, quay thiếu thì giảm
#define TURN_LEFT_YAW_SIGN      -1.0f     // Dấu thay đổi yaw khi quay TRÁI (lệch phải => yaw tăng => quay trái làm yaw giảm: -1). Quay sai chiều thì đổi +1
#define TURN_ANGLE              90.0f     // Chế độ 0: góc quay (độ), 90 hoặc 180
#define TURN_RIGHT              1         // Chế độ 0: 1 = quay PHẢI, 0 = quay TRÁI

// ============================================================================
// 11. CUA KHI ĐANG CHẠY (Corner)
// ============================================================================
#define CORNER_SPEED            60.0f     // PWM bánh ngoài khi cua (cua chậm)
#define CORNER_INNER_RATIO      0.45f     // PWM bánh trong / bánh ngoài ở độ cong tối đa. Nhỏ hơn = cua gắt hơn
#define CORNER_DECEL_ZONE       30.0f     // độ cuối: độ cong giảm dần về 0
#define CORNER_TOLERANCE        2.0f      // độ: coi là tới target khi sai số yaw <= giá trị này
#define CORNER_LEAD             0.05f     // s: giảm cong sớm theo tốc độ quay, chống overshoot

// ============================================================================
// 12. BẢN ĐỒ & ĐÍCH (chế độ 8, 10, 11)
// ============================================================================
// Xuất phát ô (0,0) góc dưới trái, mặt hướng Bắc. MAP_GOAL_X/Y là góc dưới trái vùng đích.

#define MAP_COLS                16        // số cột mê cung thật (<= 16)
#define MAP_ROWS                16        // số hàng mê cung thật (<= 16)
#define MAP_GOAL_X              7         // góc dưới trái vùng đích
#define MAP_GOAL_Y              7
#define MAP_GOAL_SIZE           2         // 1: đích 1 ô, 2: đích 2x2
#define MAP_SIDE_FROM           0.0f      // đọc vách hông của ô đang đi vào khi xe đi được từ phần này ...
#define MAP_SIDE_TO             0.30f     // ... tới phần này quãng đường ô. VL53 hông nhìn trước ~1 ô: đầu quãng tia chiếu vào vách ô đang vào, sau ~35% đã chiếu sang cột/tường trước (cũ 0.35-0.75 hay đọc sai)
// Khi tường trước đã gần, tia 45° hông chiếu vào tường trước (số đo ~ 1.3 x VL53 trước) -> đọc nhầm là có vách hông. Bỏ các mẫu đó
#define SIDE_ECHO_ENABLE        1         // 1: bỏ mẫu vách hông trùng với tường trước. 0: tắt
#define SIDE_ECHO_FRONT_MAX     260.0f    // mm: chỉ xét khi VL53 trước đọc <= mức này
#define SIDE_ECHO_RATIO         1.3f      // số đo hông khi chiếu vào tường trước ~ hệ số này x VL53 trước (đo từ log)
#define SIDE_ECHO_TOL           25.0f     // mm: lệch so với (hệ số x VL53 trước) trong mức này thì coi là chiếu vào tường trước
#define MAP_TIE_TOWARD_CENTER   1         // 1: hai hướng bằng điểm flood-fill thì chọn ô gần tâm đích hơn (mô phỏng bản đồ thật: 104 ô thay vì 220). 0: ưu tiên đi thẳng
#define MAP_PRESET              1         // 1: nạp sẵn bản đồ lib/Mapping/PresetMaze.h -> lượt dò đầu đi đường ngắn nhất ngay. 0: không biết trước bản đồ
#define MAP_PRESET_TRUST        1         // 1: số đọc vách KHI CHẠY không ghi đè bản đồ nạp sẵn (đứng yên thấy tường trước vẫn ghi). 0: số đọc luôn ghi đè
#define MAP_MAX_CELLS           400       // số ô đi tối đa rồi dừng (an toàn)

// ============================================================================
// 13. CHẾ ĐỘ 6 & FLOOD-FILL
// ============================================================================
#define NAV_MAX_STEPS           30        // Chế độ 6: số ô đi tối đa rồi dừng
#define NAV_START_SIDES_WALL    1         // Ô xuất phát: 1 = coi như có tường trái/phải; 0 = không biết
#define FF_CONTINUOUS           1         // Flood-fill: 1 = gộp các ô thẳng đã biết vách thành 1 lần chạy; 0 = dừng từng ô
#define FF_MAX_RUN              16        // Flood-fill: số ô tối đa gộp trong 1 lần chạy liên tục

// ============================================================================
// 14. TEST RIÊNG 2 ĐỘNG CƠ (open-loop, tắt PID)
// ============================================================================
// TEST_MOTOR_DRIVE = 1: cấp cùng 1 PWM cho 2 bánh, chạy TEST_PULSES xung rồi dừng, in chênh lệch encoder.

#define TEST_MOTOR_DRIVE        0         // 1: chạy test này thay cho chế độ bình thường. 0: chạy bình thường
#define TEST_PWM                100       // PWM cấp cho cả 2 bánh (đã nhân MOTOR_TRIM)
#define TEST_PULSES             9000      // dừng khi trung bình xung 2 bánh đạt giá trị này

// ============================================================================
// 15. IN RA TERMINAL
// ============================================================================
// Nhật ký dò đường ([MAP]/[RUN]/[NAV]...) luôn in. Các mục dưới tắt thì vẫn xem lại được bằng lệnh qua Serial:
// MAP = bản đồ + nhật ký, LOG = log chạy thẳng, TURNLOG = log quay.

#define PRINT_MAP               0         // 1: tự in bản đồ ở mỗi ô và khi kết thúc
#define PRINT_RUN_LOG           1         // 1: tự in log chạy thẳng (bảng ENC/VL53/YAW...) sau khi xe dừng. Dài, nhưng cần khi gửi log để phân tích
#define PRINT_TURN_LOG          0         // 1: tự in log quay (TURN/CORNER) sau khi xe dừng
#define PRINT_TELEMETRY         0         // 1: khi đứng yên in ENC/SPEED/PWM mỗi 100ms
