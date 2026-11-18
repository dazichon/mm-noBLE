# Ghi chú toàn bộ PID / bộ điều khiển căn tường & cân xe

Nguồn: `lib/MainParams/MainParams.h` (giá trị chỉnh), `src/App.cpp:38-90` (gán vào `robotNav`),
`lib/Navigation/NavStraight.cpp` (`updatePIDLoop`), `lib/Navigation/NavTurn.cpp` (`alignFrontWall`, `calibCellFromBackWall`).

---

## 0. Tổng quan

| # | Bộ điều khiển | File / hàm | Dùng khi |
|---|---|---|---|
| 1 | Gyro PID (yaw P+I+D) | `NavStraight.cpp:226-245` | Đi thẳng |
| 2 | Wall -> dịch hướng gyro (cascade) | `NavStraight.cpp:194-218` | Đi thẳng, có tường hông |
| 3 | Wall P (PWM trực tiếp, fallback) | `NavStraight.cpp:219-222` | Chỉ khi KHÔNG có gyro |
| 4 | Encoder P (cân 2 bánh) | `NavStraight.cpp:182-188` | Chỉ khi KHÔNG có gyro (hiện Kp=0) |
| 5 | **Căn tường TRƯỚC** (P + boost + giữ hướng gyro) | `NavTurn.cpp:312-370` `alignFrontWall()` | Trước khi quay, ở tường chắn (MAP_ALIGN_FRONT) |
| 6 | Giữ hướng khi lùi calib vách sau (P gyro) | `NavTurn.cpp:237-304` | Ngõ cụt |
| 7 | Motor trim (cân 2 động cơ, open-loop) | `MOTOR_TRIM_*` | Mọi chế độ |

Công thức cuối mỗi vòng lặp (đi thẳng):
```
pidOut = encCorr + (wallCorr nếu không gyro) + yawCorr
PWM_L = (base - pidOut) * motorScaleL     PWM_R = (base + pidOut) * motorScaleR
clamp [35, 255]
```
Quy ước dấu: `pidOut > 0` => giảm bánh trái, tăng bánh phải => xe quay TRÁI.

---

## 1. Gyro PID – giữ hướng (đi thẳng)

```
e       = wrap180(targetYaw + turnLeftYawSign*wallYawOff - yaw)
yawI   += e*dt          (kẹp ±yawIMax/Ki, chỉ khi 0<dt<0.1s)
yawD    = Kd * gz       (gz = vận tốc góc Z, deg/s)
yawCorr = clamp( -(Kp*e + Ki*yawI) + yawD , ±yawMaxCorr )
```

| Tham số | MainParams | Giá trị | Ghi chú |
|---|---|---|---|
| Kp | `HOLD_YAW_KP` | 5.5 | (default trong RobotNav.h: 2.0) |
| Kd | `HOLD_YAW_KD` | 0.15 | dập lắc đuôi cá |
| Ki | `HOLD_YAW_KI` | 0.08 | khử cong cánh cung; reset khi Ki<=0 |
| yawIMax | RobotNav.h:26 | 12.0 PWM | giới hạn phần I |
| yawMaxCorr | RobotNav.h:150 | 30 PWM | giới hạn tổng yaw |

Chỉnh runtime qua Serial: `SET_YAW_KP=`/`YAW_KP=`, `SET_YAW_KD=`/`YAW_KD=` (không có lệnh cho Ki).

---

## 2. Wall – căn giữa theo tường hông (cascade vào gyro)

Khi có gyro: tường **không cộng PWM**, mà **dịch hướng đích** của gyro -> xe đi chéo nhẹ về tâm rồi thẳng lại.

```
eL = L_filt - WALL_BASE_L ;  eR = R_filt - WALL_BASE_R
cả 2 hợp lệ: wallErr = 0.5*(eL - eR)
chỉ L:       wallErr =  eL
chỉ R:       wallErr = -eR
wallErr = clamp(±wallErrClamp=30mm)
targetYawOff = clamp(WALL_YAW_GAIN * wallErr, ±WALL_YAW_MAX)
slew limiter bất đối xứng: tăng (bù) 0.8°/vòng, giảm (xả) 0.15°/vòng
```
"Hợp lệ" = `20 < giá trị lọc < WALL_VALID_MAX`.

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| `WALL_ENABLE` | 1 | bật/tắt |
| `WALL_YAW_GAIN` | 0.10 °/mm | 10mm lệch -> 1° |
| `WALL_YAW_MAX` | 3.5° | trần dịch hướng |
| `WALL_BASE_L` / `WALL_BASE_R` | 173 / 183 mm | VL53 khi xe ở giữa ô – **phải đo lại nếu đổi xe/cảm biến** |
| `WALL_VALID_MAX` | 220 mm | xa hơn = không có tường |
| `WALL_KP` | 0.30 PWM/mm | **chỉ dùng khi không có gyro** |
| `WALL_MAX_CORR` | 6.0 PWM | trần của Wall P fallback |

Serial: `SET_WALL_KP=`/`WALL_KP=` (chỉ ảnh hưởng nhánh fallback, không ảnh hưởng nhánh gyro; gain/max không có lệnh chỉnh).

---

## 3. Encoder P – cân 2 bánh

`encCorr = clamp(encKp * (travL - travR), ±ENC_MAX_CORR)` – **chỉ chạy khi không có gyro** và `encoderSyncActive`.

| Tham số | Giá trị |
|---|---|
| `ENC_STRAIGHT_KP` | 0.0 (tắt: tránh đánh nhau với gyro) |
| `ENC_MAX_CORR` | 10 PWM |

(Lưu ý: `RobotNav.cpp:40` có default 0.35 nhưng `App.cpp` ghi đè bằng 0.0.)

---

## 4. CĂN TƯỜNG TRƯỚC (`alignFrontWall()`) – trọng tâm để cân xe

Gọi khi `MAP_ALIGN_FRONT = 1` và trước mặt có tường, **trước khi quay**. Là vòng chặn (blocking), tối đa `frontAlignMaxMs`.

### 4.1 Thuật toán
1. `stopPID()`, lấy `yawT` = hướng đã chốt (`_turnHeading`) hoặc yaw hiện tại.
2. Đọc VL53 trước, **median-3** (3 mẫu mới, chỉ nhận `0 < raw < 400`). Chưa đủ 3 mẫu thì chờ.
3. `e = d - FRONT_ALIGN_MM` (>0: còn xa tường -> tiến; <0: quá gần -> lùi).
4. **Lần đọc đầu**: nếu `|e| <= FRONT_ALIGN_SKIP` -> bỏ qua, không căn (OK ngay).
5. Nếu `|e| <= FRONT_ALIGN_TOL`: dừng motor; đứng yên trong dung sai **>100ms** -> OK, thoát.
6. Ngược lại tính PWM:
   ```
   boost = (now - progT) / 30.0          // progT reset khi d đổi > 2mm
   pwm   = clamp( |e|*FRONT_ALIGN_KP + boost , frontAlignMinPwm , frontAlignMaxPwm )
   ```
   `boost` tăng 1 PWM mỗi 30ms khi xe đứng im (thắng ma sát tĩnh).
7. **Giữ hướng bằng gyro (P-only)**:
   ```
   corr = clamp( -yawKp * wrap180(yawT - yaw) , ±10 )     // dùng chung HOLD_YAW_KP = 5.5
   if (e < 0) corr = -corr                                // lùi thì đảo dấu
   PWM_L = (pwm - corr)*scaleL ;  PWM_R = (pwm + corr)*scaleR
   ```
   e>0: tiến (IN1); e<0: lùi (IN2).
8. Hết `frontAlignMaxMs` -> thoát, in "HET GIO".

### 4.2 Tham số

| Tham số | Nơi đặt | Giá trị | Tác dụng |
|---|---|---|---|
| `FRONT_ALIGN_MM` | MainParams | 95 mm | VL53 trước đọc được khi xe **đúng tâm ô** có tường trước (đo thực tế rồi ghi vào) |
| `FRONT_ALIGN_SKIP` | MainParams | 18 mm | lệch ít hơn thì không căn. Xe căn quá nhiều -> tăng |
| `FRONT_ALIGN_TOL` | MainParams | 10 mm | vào dải này là dừng |
| `FRONT_ALIGN_MIN_PWM` | MainParams | 45 | PWM sàn |
| `FRONT_ALIGN_KP` | MainParams | 0.3 PWM/mm | tăng: căn nhanh, quá cao thì tiến-lùi qua lại |
| `frontAlignMaxPwm` | **App.cpp:85** | 55 | trần PWM (RobotNav.h default 70 bị ghi đè) |
| `frontAlignMaxMs` | **App.cpp:86** | 800 ms | timeout (RobotNav.h default 1500 bị ghi đè) |
| `yawKp` (giữ hướng) | dùng chung `HOLD_YAW_KP` | 5.5 | trần corr ±10 |

### 4.3 Điểm cần để ý (cân xe)
- Với Kp=0.3: lệch 18mm -> 5.4 PWM, thấp hơn MinPwm 45 -> thực tế **luôn chạy ở 45** cho tới khi `boost` đẩy lên; Kp gần như vô nghĩa trừ khi |e| > ~150mm. Tức là phần "P" gần như chỉ là bang-bang 45-55 PWM, dễ **vọt qua dung sai ±10mm** rồi lùi lại (dao động tiến-lùi). Nếu thấy xe giật tiến lùi: giảm `FRONT_ALIGN_MIN_PWM`, hoặc nới `TOL`, hoặc tăng `SKIP`.
- Comment trong MainParams ("tự tăng dần tới 70") **sai** so với thực tế: `App.cpp` ép trần = 55.
- `800ms` timeout có thể không đủ nếu xe đứng im phải chờ boost (45 -> 55 mất 300ms).
- Tường trước đọc VL53 lệch (xe nghiêng) -> căn khoảng cách đúng nhưng không căn **góc**; góc chỉ do gyro giữ theo `yawT`, không dùng tường để nắn thẳng xe. Muốn cân xe thật sự song song tường: cần dùng thêm 2 VL53 hông/hoặc lùi sát vách (mục 5).
- `FRONT_ALIGN_MM` phải đo lại khi đổi vị trí cảm biến. `NAV_FRONT_WALL_MM=150` (ngưỡng nhận biết tường) phải lớn hơn `FRONT_ALIGN_MM` + `FRONT_ALIGN_TOL`.
- Giá trị `HOLD_USE_FRONT=0`: **không** phanh theo tường trước trong lúc chạy thẳng; tường trước chỉ dùng cho nhận biết ô và `alignFrontWall`.

---

## 5. Cân xe bằng vách SAU (`calibCellFromBackWall`) – ngõ cụt

Không phải PID đầy đủ; giữ hướng bằng **P gyro** khi lùi:
```
corr = clamp(yawKp * (target - yaw), ±10)          // lùi: dấu ngược khi tiến
revPwm = min(CALIB_REV_PWM, 40 + (CALIB_REV_PWM-40)*t/150ms)   // ramp 150ms
PWM_L = (revPwm - corr)*scaleL ; PWM_R = (revPwm + corr)*scaleR
```
- Dừng khi: lùi >= `calibRevMinPulses`(1300 xung) **và** encoder đứng yên >100ms; hoặc hết `CALIB_REV_MS`(600ms).
- Ép thêm `calibSquareMs`(120ms) với PWM `CALIB_REV_PWM`(72) cho lưng xe áp phẳng (không sửa hướng – vách tự nắn thẳng).
- Sau đó: đứng yên 150ms -> `mpu6050.calibrate(120)` -> `resetYaw()` -> yaw=0 -> tiến `CALIB_FWD_PULSES`(1500) bằng PID đi thẳng tới tâm ô.
- `MAP_CALIB_BACK=1`, cứ `MAP_TURNS_BEFORE_CALIB=5` lần quay thì lùi căn 1 lần. **Đây là cách duy nhất reset drift gyro + căn góc xe thật sự.**

---

## 6. Motor trim – cân 2 động cơ (open-loop)

```
motorScaleL = 1 + MOTOR_TRIM_LEFT/100   =  0.94   (TRIM_LEFT = -6%)
motorScaleR = 1 + MOTOR_TRIM_RIGHT/100  =  1.00
```
Áp dụng ở mọi chế độ (đi thẳng, căn trước, lùi calib). Serial đặt lại: lệnh tại `CommandHandler.cpp:136-140`.
Xe lệch PHẢI -> giảm trim trái (âm) hoặc tăng trim phải; lệch TRÁI ngược lại.
`TEST_MOTOR_DRIVE=1` chạy open-loop tắt hết PID để đo chênh encoder và tinh chỉnh trim.

---

## 7. Các tham số liên quan khác

| Tham số | Giá trị | Ghi chú |
|---|---|---|
| `BASE_FORWARD_SPEED` | 80 | PWM chạy thẳng |
| `CRAWL_SPEED` / `CRAWL_ZONE_RATIO` | 50 / 0.85 | bò 15% cuối ô |
| `ACCEL_START_PWM` / `ACCEL_RAMP_MS` | 55 / 180 | ramp xuất phát |
| `CELL_PULSES` | 4400 | xung/ô |
| `STOP_LEAD_PULSES` | 280 | phanh sớm bù trôi |
| Cua (`CORNER_*`) | speed 62, inner 0.40, decel 25°, tol 1.5°, lead 0.04s | corner controller riêng |
| Quay tại chỗ (`TURN_*`) | max 75, min 40, decel 25°, tol 1°, brake lead 0.035s | profile giảm tốc tuyến tính theo góc |
| `NAV_SIDE_WALL_MM` / `NAV_FRONT_WALL_MM` | 220 / 150 | ngưỡng nhận biết tường |

## 8. Lưu ý về giá trị "hai nơi"
Default trong `RobotNav.h` bị `App.cpp` ghi đè bằng MainParams: `yawKp` (2.0->5.5), `yawKd` (0.08->0.15), `yawKi` (0->0.08), `encKp` (0.35->0), `frontAlignKp` (1.0->0.3), `frontAlignMaxPwm` (70->55), `frontAlignMaxMs` (1500->800). Muốn đổi thì sửa **MainParams.h** (hoặc dòng 85-86 trong `App.cpp` cho 2 tham số cứng).

## 9. Mode 5 – dò map bằng cua Slalom (ghi 2026-11-06)
- Luồng: dò tới đích -> dò về xuất phát (cua Slalom, chạy liên tục) -> speedrun bằng cua. Hàm: `MazeMapper::fullRunSlalom/exploreToSlalom` (Mapping.cpp).
- Quy tắc vách hông: **thông là khóa** (`openLock`), mẫu "có tường" đến sau không được sửa; không dùng trung vị.
- Lỗi đã gặp: ô (4,1) báo tường 2 bên dù trái thông. Log đo tay: VL53 trái khi **có tường ~179 mm**, khi **thông vẫn ~195 mm** (không ra xa), vì tia chéo chiếu vào vật khác. Ngưỡng cũ `navSideWallMm=220` cao hơn cả hai nên không bao giờ thấy "thông".
- Sửa: `SLALOM_EXPLORE_OPEN_MM 190` (đọc >=190 hoặc -3 là thông) + `SLALOM_EXPLORE_OPEN_MIN_SAMPLES 3` (3 mẫu liên tiếp) trong SlalomParams.h; biến `_openRunL/_openRunR` trong RobotNav.h. Biên chỉ ~16 mm nên nếu báo thông giả thì tăng MIN_SAMPLES, nếu bỏ sót thì hạ OPEN_MM.
- Ý tưởng chưa làm: hạ timing budget VL53 (~20 ms) để lấy mẫu dày hơn (hiện ~35 ms, ~7-9 mm/mẫu ở 200-250 mm/s); hoặc chỉnh góc/vị trí lắp cảm biến hông để lúc thông đọc xa hẳn.
- Thông số cua đã tune: xem memory `slalom-tuned-params.md`.

## 10. Thông số RIÊNG của mode 2 / 4 / 5 (snapshot 2026-11-06, trước khi đồng bộ mode 1)
Toàn bộ nằm ở `lib/MainParams/SlalomParams.h` (mode 2, 4, 5 dùng chung), cộng vài dòng ở `src/App.cpp`.

**Cua + tốc độ (SlalomParams.h)**
| Thông số | Giá trị |
|---|---|
| `SLALOM_SPEED_MM_S` / `RUN_VMAX_MM_S` | 190 / 190 |
| `SLALOM_STRAIGHT_SPEED_MM_S` / `RUN_STOP_SPEED_MM_S` | 240 / 100 |
| `RADIUS_MM` / `DELTA_MM` / `ANGLE_DEG` / `WHEEL_TRACK_MM` | 52 / 31 / 90 / 98 |
| `ACCEL_MM_S2` / `DECEL_MM_S2` | 1200 / 900 |
| `KP_YAW` / `KD_YAW` / `OMEGA_MAX` | 10 / 0.25 / 4.6 |
| `PWM_DEAD` / `PWM_PER_MMS` / `WHEEL_KP` / `WHEEL_KI` / `PWM_MAX` / `PWM_REV_MAX` | 30 / 0.12 / 0.40 / 4.0 / 230 / 140 |
| `RUN_PWM` | 70 |
| `RUN_YAW_TOL_DEG` / `RUN_GYRO_TOL_DPS` / `RUN_EXTRA_MM` | 6 / 130 / 45 |
| `LAG_SLOW_DEG` / `LAG_MIN_SCALE` / `LOOKAHEAD_MM` | 25 / 0.45 / 5 |
| `ANGLE_TRIM_LEFT/RIGHT_DEG` | -1.5 / -2.6 |
| `POST_TURN_WALL_MUL` / `WALL_YAW_CAP_DEG` / `WALL_FADE_MM` | 1.5 / 4 / 20 |
| `FRONT_TRIG_BIAS_MM` / `STOP_LEAD_MM` / `FRONT_EXTEND_MM` | -8 / 12 / 55 |

**Riêng mode 5 (dò)**: `EXPLORE_SIDE_FROM/TO_MM` 40/112, `EXPLORE_OPEN_MM` 190, `OPEN_MIN_SAMPLES` 2 (trái) / `_R` 1 (phải). `exploreToSlalom`, `roamSlalom(30)`, `openLock`, `visits`.

**`src/App.cpp` (applyParams)** - dòng chỉ áp dụng cho mode 2/5:
```
mapper.speedrunSpeed = (SEQ_MODE == 2 || SEQ_MODE == 5) ? SLALOM_RUN_PWM : MAP_SPEEDRUN_PWM;
mapper.slalomTurns   = (SEQ_MODE == 2 || SEQ_MODE == 5);
mapper.calibBack     = MAP_CALIB_BACK && SEQ_MODE != 2 && SEQ_MODE != 5;
mapper.usePreset     = (MAP_PRESET || SEQ_MODE == 2) && SEQ_MODE != 5;
```
**Căn tường trước (chung mọi mode, giữ nguyên)**: `FRONT_ALIGN_MM` 100, `TOL` 6, `SKIP` 10, `KP` 0.2, `MIN_PWM` 40, `FRONT_EMERGENCY_MM` 75.

**Mode 1 (trước khi đồng bộ)**: dò bằng PWM (`FORWARD_PWM` 200, `ACCEL_START_PWM` 70, `ACCEL_RAMP_MS` 400), speedrun `MAP_SPEEDRUN_PWM` 210, PID thẳng `HOLD_YAW_KP` 2.5 / `HOLD_YAW_KI` 1.5 / `ENC_STRAIGHT_KP` 0.08 / `WALL_KP` 0.5 / `WALL_YAW_GAIN` 0.15, quay tại chỗ `TURN_*` (80/50/35/1.5/0.05), `STOP_LEAD_PULSES` 280, `calibBack` bật.

## 11. Đồng bộ mode 1 với mode 2/4/5 (thử) + CÁCH QUAY LẠI
Đã đổi trong `src/App.cpp` (chỉ dòng dưới, `SlalomParams.h`/`MainParams.h` không đổi):
- `speedrunSpeed`, `slalomTurns` áp dụng cho `SEQ_MODE == 1` luôn -> speedrun mode 1 chạy bằng `slalomStraight` + cua Slalom, cùng tốc độ/PID như mode 2.
- `fullRun()` (Mapping.cpp): bật `frontGuard` + `distUseMax` quanh lượt speedrun khi `slalomTurns`, giống `runKnownMap`.
- Lượt DÒ của mode 1 vẫn dò bằng PWM + quay tại chỗ (không đổi được tương ứng sang mm/s).

**Quay lại**: trong `App.cpp` sửa 2 dòng đầu ở khung trên về `(SEQ_MODE == 2 || SEQ_MODE == 5)` như đã ghi; trong `fullRun()` xoá các dòng `frontGuard/distUseMax` quanh `speedRun()`.

## 12. Đã QUAY LẠI bản trước khi đồng bộ mode 1 (mục 11 không còn áp dụng)
Mode 1 chạy như cũ (speedrun PWM, quay tại chỗ). Giữ lại 2 sửa lỗi: `fullRun()` lượt về dùng `startX/startY` (trước là (0,0) cứng), và `MAP_PRESET 0` khi map không phải bản ảnh.

## 13. Mode 1 dùng lại thông số của bản git (commit 8da590a)
Khối `#if SEQ_MODE == 1` cuối `applyParams()` trong `src/App.cpp` ghi đè: motorScaleL (trim -6), baseForwardSpeed 180, accelStartPwm 60, encKp 0.13, yawKp 2.0, yawKi 3.0, frontAlign 110 / skip 18 / tol 10.
Mode 2/4/5 giữ MainParams.h hiện tại (trim -20, FORWARD_PWM 200, ACCEL 70, encKp 0.08, yawKp 2.5, yawKi 1.5, align 100/10/6). Muốn bỏ: xoá khối #if đó.

## 14. Mode 1 chỉnh lại GIỐNG các mode khác (bỏ khối ghi đè mục 13)
Đã xoá khối `#if SEQ_MODE == 1` trong `applyParams()`. Mode 1 dùng đúng MainParams.h như mode 2/4/5 (trim -20, FORWARD_PWM 200, ACCEL 70, encKp 0.08, yawKp 2.5, yawKi 1.5, align 100/10/6). Muốn dùng lại bản git: thêm lại khối ở mục 13.

## 15. Đã QUAY LẠI mốc 14:39 (sau khi làm xong mode 5 chạy tự do 30 ô)
Khôi phục: `MAP_START` (4,0), `MAP_GOAL` (1,4), `MAP_PRESET` 1 trong MainParams.h; `fullRun()` lượt về lại là `exploreTo(0, 0, ...)` như lúc đó. Mode 1 không còn khối ghi đè thông số/đồng bộ. (Sửa lỗi lượt về dùng `startX/startY` đã bị bỏ theo mốc này.)
