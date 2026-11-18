#pragma once
// THÔNG SỐ CUA SLALOM 90° (MODE 4): quỹ đạo theo QUÃNG ĐƯỜNG encoder, yaw/gyro MPU phản hồi
#define SLALOM_TURN_LEFT        1         // 1: cua trái, 0: cua phải
#define SLALOM_SPEED_MM_S       190.0f    // mm/s: tốc độ tiến khi cua
#define SLALOM_RADIUS_MM        52.0f     // mm: bán kính cung giữa (nhỏ hơn 60 để đoạn thẳng giữa 2 cua dài ~44mm thay vì 22mm, đủ chỗ căn lại bằng tường)
#define SLALOM_DELTA_MM         31.0f     // mm: độ dài pha tăng/giảm độ cong (S1 = S3)
#define SLALOM_ANGLE_DEG        90.0f     // độ: góc cua
#define SLALOM_WHEEL_TRACK_MM   98.0f     // mm: khoảng cách tâm 2 bánh (tạm, chưa đo)
#define SLALOM_ENC_SIGN         -1        // chiều encoder khi xe TIẾN: -1 = xung đếm ÂM (đúng với xe này), +1 = đếm dương. Sai dấu thì distance/meas_v trong log âm và xe chạy thẳng
#define SLALOM_PRE_STRAIGHT_MS  0         // ms: chạy thẳng (giữ yaw) trước cua; 0 = bỏ, để test riêng thuật toán cua
#define SLALOM_ACCEL_MM_S2      1200.0f   // mm/s²: tăng/giảm tốc của biên dạng tốc độ (đoạn thẳng nối cua rất ngắn)
#define SLALOM_LOOP_US          5000      // chu kỳ vòng điều khiển (us)
#define SLALOM_LOG_MS           20        // ms: chu kỳ ghi log
// --- Điều khiển yaw: omegaCmd = omegaFF + KP*yawErr + KD*(omegaFF - omegaĐo)  (rad, rad/s)
#define SLALOM_KP_YAW           10.0f     // (rad/s)/rad. Tăng: bám yaw chặt hơn, quá lớn thì dao động
#define SLALOM_KD_YAW           0.25f     // (rad/s)/(rad/s). Cản theo sai lệch tốc độ góc
#define SLALOM_OMEGA_MAX        4.6f      // rad/s: giới hạn omegaCmd (v=190, R=52: omega FF đỉnh = 3.65 rad/s)
// --- Tốc độ từng bánh: PWM = DEAD + PER_MMS*v (feedforward) + PI theo encoder
#define SLALOM_PWM_DEAD         30.0f     // PWM tối thiểu để bánh bắt đầu quay
#define SLALOM_PWM_PER_MMS      0.12f     // PWM trên mm/s (feedforward). Đo từ log chạy thẳng: PWM ~60 cho ~220 mm/s. PI bù phần còn lại
#define SLALOM_WHEEL_KP         0.40f     // PWM trên mm/s sai số
#define SLALOM_WHEEL_KI         4.0f      // PWM trên (mm/s·s)
#define SLALOM_PWM_MAX          230       // giới hạn PWM
// --- Kết thúc
#define SLALOM_YAW_TOL_DEG      1.5f      // độ: sai số yaw chấp nhận khi hết quãng đường
#define SLALOM_GYRO_TOL_DPS     40.0f     // độ/s: tốc độ góc tối đa khi coi là xong
#define SLALOM_EXTRA_MM         15.0f     // mm: được đi thêm tối đa sau S_TOTAL để dồn nốt yaw; quá thì báo lỗi DISTANCE_LIMIT
#define SLALOM_TIMEOUT_MS       3000      // ms
#define SLALOM_ABORT_YAW_DEG    35.0f     // độ: lệch yaw so với mục tiêu quá mức này thì dừng (xe mất kiểm soát)
// --- Speedrun mode 2 (cua Slalom khi đang chạy liên tục)
#define SLALOM_RUN_PWM          70        // PWM chạy thẳng giữa các cua ở mode 2 (đo từ log: PWM ~70 cho ~130-150 mm/s, PWM 100 cho ~250 mm/s). Tăng: nhanh hơn, vào cua nhanh hơn tốc độ cua nên bánh trong phải giảm mạnh
#define SLALOM_RUN_GYRO_TOL_DPS 130.0f    // độ/s: khi chạy liên tục, tốc độ góc còn lại tối đa lúc coi là cua xong (đoạn thẳng sau sẽ tự cân hướng)
#define SLALOM_RUN_VMAX_MM_S    190.0f    // mm/s: tốc độ cua tối đa khi chạy liên tục (mode 2); nhanh hơn thì hãm về mức này
#define SLALOM_RUN_YAW_TOL_DEG  6.0f      // độ: khi chạy liên tục (mode 2), sai số yaw tối đa lúc coi là cua xong. Đoạn thẳng sau cua tự kéo hướng về hướng chuẩn bằng gyro nên không cần chặt như mode 4
#define SLALOM_STRAIGHT_SPEED_MM_S 240.0f // mm/s: tốc độ đỉnh đoạn thẳng nối cua (log: 325 -> vào cua 1 ở ~300 mm/s thay vì 170 vì hãm không kịp)
#define SLALOM_RUN_STOP_SPEED_MM_S 100.0f // mm/s: tốc độ cuối đoạn thẳng cuối (trước khi phanh dừng ở đích)
#define SLALOM_POST_TURN_WALL_MUL 1.5f    // hệ số nhân độ lợi + giới hạn bám tường ở đoạn thẳng NGAY SAU cua (2.0 làm xe lắc)
#define SLALOM_STOP_LEAD_MM     12.0f     // mm: đoạn thẳng cuối (có tường trước, phanh dừng) kết thúc khi VL53 trước đọc <= FRONT_ALIGN_MM + giá trị này (bù quãng trôi sau phanh)
#define SLALOM_PWM_REV_MAX      140       // PWM đảo chiều tối đa để HÃM bánh trong. Log: bánh trong chạm -110 ngay đầu cua 1 khi vào nhanh
#define SLALOM_LAG_SLOW_DEG     25.0f     // độ: yaw trễ quỹ đạo chừng này thì tốc độ tiến giảm xuống SLALOM_LAG_MIN_SCALE (để quay kịp)
#define SLALOM_LAG_MIN_SCALE    0.45f     // tốc độ tiến tối thiểu (tỉ lệ) khi yaw trễ nhiều
#define SLALOM_RUN_EXTRA_MM     45.0f     // mm: khi chạy liên tục (mode 2) cua được đi thêm tối đa chừng này sau S_total để quay cho đủ góc (không hủy cua giữa chừng)
#define SLALOM_LOOKAHEAD_MM     5.0f      // mm: feedforward độ cong nhìn trước chừng này để bù độ trễ quay (quá lớn thì xe cua sớm). 0: tắt
#define SLALOM_FRONT_TRIG_BIAS_MM -3.0f  // mm: hiệu chỉnh ngưỡng VL53 trước khi vào cua. Ngưỡng = FRONT_ALIGN_MM + quãng cua sớm (~68) + giá trị này (~170 mm). Âm = tiến sát tường hơn mới cua (muộn hơn), dương = cua sớm hơn
#define SLALOM_DECEL_MM_S2      900.0f    // mm/s²: giảm tốc của biên dạng đoạn thẳng trước cua (thấp hơn tăng tốc vì bánh chỉ hãm được chừng này)
#define SLALOM_FRONT_EXTEND_MM  55.0f     // mm: encoder báo đủ mà VL53 trước còn thấy tường xa thì đi tiếp tối đa chừng này (log: đoạn 44 mm thường cần thêm 25-36 mm)
#define SLALOM_ANGLE_TRIM_LEFT_DEG  -10.0f  // độ: cộng vào góc đích cua TRÁI (log v=190: vượt ~1.8 độ với trim +0.3)
#define SLALOM_ANGLE_TRIM_RIGHT_DEG -11.0f  // độ: cộng vào góc đích cua PHẢI (log v=190: vượt ~1.65 độ với trim -1.0)
#define SLALOM_WALL_YAW_CAP_DEG      4.0f   // độ: dịch hướng tối đa do bám tường ở đoạn thẳng nối cua
#define SLALOM_WALL_FADE_MM          20.0f  // mm: ở đoạn vào cua, phần lái lệch của bám tường giảm dần về 0 trong chừng này trước điểm cua
#define SLALOM_FRONT_COVER_MM   20        // mm: VL53 trước đọc 0..chừng này liên tiếp SLALOM_FRONT_COVER_N mẫu (lúc frontGuard bật) = bị che kín bằng tay -> dừng khẩn cấp
#define SLALOM_FRONT_COVER_N    4         // số mẫu liên tiếp (~35 ms/mẫu)
#define ROAM_CALIB_EXTRA_PULSES 1000      // xung (mode 6): cộng THÊM vào CALIB_FWD_PULSES khi lùi kịch tường ở ngõ cụt. Sau calib xe còn đứng SÂU hơn tâm ô (log: đoạn ngay sau calib dài hơn dự kiến ~78 mm, hoặc cua sớm). Xe cua SỚM -> tăng; cua MUỘN -> giảm. 1 xung = 0.0439 mm
#define ROAM_FRONT_EXTEND_MM   120.0f     // mm (mode 6): hết quãng encoder mà VL53 trước còn thấy tường xa (<= FRONT_STOP_MAX_MM = 300, tức vẫn là tường của CHÍNH ô này, tường ô kế >= ~340) thì xe chưa đi đủ ô -> đi tiếp tới khi VL53 = ngưỡng cua, tối đa chừng này
#define SLALOM_EXPLORE_PWM      100       // PWM chạy thẳng lúc DÒ của mode 5 (lượt 1 + 2). Chậm hơn FORWARD_PWM để VL53 hông lấy đủ mẫu. Speedrun vẫn dùng MAP_SPEEDRUN_PWM. 0: giữ FORWARD_PWM
#define SLALOM_EXPLORE_SIDE_FROM_MM 40.0f // mm (mode 5): đoạn dò chỉ lấy mẫu vách hông khi xe ở vị trí từ chừng này (tính từ tâm ô trước)...
#define SLALOM_EXPLORE_SIDE_TO_MM   112.0f // ... tới chừng này. Cửa sổ cũ 0-54 làm tia chéo còn chiếu vào vách ô vừa rời (log: ô (4,1) đọc đúng 2 vách của ô xuất phát). Chỉnh theo dòng "vet p:L/R" trong log
#define SLALOM_EXPLORE_OPEN_MM_L 205        // mode 5: mắt TRÁI đọc >= chừng này (hoặc -3) mới coi là THÔNG. Log: tường trái đọc tới ~205, thông thật 210-227
#define SLALOM_EXPLORE_OPEN_MM_R 200        // mode 5: mắt PHẢI, tường phải chưa thấy đọc quá 190, thông thường là -3
#define SLALOM_EXPLORE_L_NOM_MM  185       // mode 5: VL53 TRÁI đọc tường khi xe chạy giữa hành lang (log: 179-192). Mắt trái đọc < chừng này -> xe lệch trái -> nâng ngưỡng THÔNG bên phải
#define SLALOM_EXPLORE_R_NOM_MM  155       // mode 5: VL53 PHẢI đọc tường khi xe chạy giữa (log: 148-163). Log ô (4,3): R=135-143 (lệch phải) -> L tường đọc 206-210 vượt 205 -> báo THÔNG giả
#define SLALOM_EXPLORE_OPEN_MM_OLD 190          // mode 5: VL53 hông đọc >= chừng này (hoặc -3) coi là THÔNG (log: tường ~179, thông ~195)
#define SLALOM_FRONT_FAR_MM     800.0f    // mm (map nạp sẵn): đoạn có tường cuối đoạn, encoder đủ mà VL53 trước còn đọc xa (<= mức này) -> đi tiếp tới khi tới điểm cua
#define SLALOM_YAW_RESET_MM     180.0f    // mm: sau cua, đoạn thẳng đi được chừng này (~1 ô) thì reset yaw MPU = 0, lấy hướng hiện tại làm hướng chuẩn. Số lớn (vd 99999) = tắt
#define SLALOM_YAW_RESET_MAX_DPS 20.0f    // độ/s: chỉ reset khi xe không đang xoay (gyro nhỏ hơn mức này), chờ tới khi đủ điều kiện
#define SLALOM_YAW_RESET_MAX_WALL_MM 8.0f // mm: chỉ reset yaw khi sai lệch tường hông <= chừng này (xe đã ở giữa hành lang); không có tường hông thì reset luôn
#define SLALOM_YAW_RESET_MAX_DRIFT_MM 3.0f // mm: chỉ reset yaw khi sai lệch tường thay đổi <= chừng này trong ~70 mm đi (xe song song tường, lệch < ~2.5 độ)
#define SLALOM_STRAIGHT_BRAKE_KP 2.0f     // PWM trên mm/s: đoạn thẳng trước cua nhanh hơn biên dạng thì hãm mạnh cỡ này (log: 0.4 hãm không nổi, vào cua ~320 mm/s thay vì 190 -> cua rộng, ra sát tường ngoài)
#define SLALOM_EDGE_TURN_MM     100.0f     // mm (map nạp sẵn, mode 2/7/9): mắt hông bên sắp rẽ thấy tường -> TRỐNG thì đi thêm chừng này rồi cua. Cua sớm/đâm góc: tăng; cua muộn: giảm. Log: dòng [MEP]
#define SLALOM_EDGE_OPEN_N      2         // mép tường hông: cần chừng này mẫu TRỐNG liên tiếp mới chốt (log: xe lệch phải, mắt trái đọc 221 lẻ 1 mẫu -> cua sớm 30 mm)
#define SLALOM_EDGE_OPEN_STRONG_MM 260    // mm: 1 mẫu đọc >= chừng này (hoặc -3) thì chốt ngay, không cần chờ mẫu thứ 2
#define SLALOM_EXPLORE_OPEN_MIN_SAMPLES_R 1 // mode 5: riêng mắt PHẢI chỉ cần 1 mẫu thông là thông
#define SLALOM_EXPLORE_OPEN_MIN_SAMPLES 1   // mode 5: chỉ cần 1 mẫu VL53 hông thấy THÔNG (ở bất kỳ lúc nào trong đoạn) là ghi thông và khóa, không dùng trung vị/đa số. Tăng nếu bị báo thông giả
#define SLALOM_EXPLORE_R_SPREAD_MM    55    // mode 5: mắt PHẢI thấy (max - min) trong đoạn dò >= chừng này thì coi là THÔNG (tường chạy ổn định, log <= ~43 mm; thông nhảy >= ~65 mm)
#define SLALOM_EXPLORE_R_SPREAD_MIN_N 5     // mode 5: cần ít nhất chừng này mẫu phải hợp lệ mới xét độ phân tán
