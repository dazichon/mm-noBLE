# TỔNG HỢP: QUAY / CUA 90° + NGÕ CỤT (LÙI → CALIB LẠI)

Nguồn: `lib/Navigation/RobotNav.{h,cpp}`, `lib/MainParams/MainParams.h`, `src/main.cpp`, `lib/MPU6050/MPU6050.{h,cpp}`.
Code bên dưới copy NGUYÊN VĂN từ project này (số dòng ở project gốc ghi kèm để đối chiếu).

---

## 0. CHECKLIST GHÉP (đọc trước)

Project đích cần CÓ SẴN (hoặc ghép luôn) những thứ sau, vì code dưới đây phụ thuộc:

| Phụ thuộc | Ở đâu trong project này | Dùng cho |
|---|---|---|
| `mpu6050.update()/getYaw()/calibrate()/resetYaw()/setAutoBias()` | `lib/MPU6050` | yaw feedback, calib gyro |
| `getLeftEncoder()/getRightEncoder()/resetEnc()` | RobotNav | đếm xung |
| `stopMotors()`, `stopPID()`, `startHold()`, `stepCell()`, `updatePIDLoop()` | RobotNav | chạy thẳng PID |
| `motorScaleL/R`, `currentLeftSpeed/RightSpeed`, `baseForwardSpeed` | RobotNav | PWM |
| `hold.yawKp`, `hold.cells` | struct `HoldParams` | giữ hướng khi lùi, số ô |
| `pidRunActive`, `stepCellActive`, `stepTraveledPulses`, `pulsesPerCell`, `_targetYaw`, `_startEncLeft/Right`, `_lastEncLeft/Right`, `_lastPIDLoopTime`, `stepStartL/R` | RobotNav | finishCorner() sửa các biến này |
| `M1_IN1, M1_IN2, M2_IN1, M2_IN2` | `RobotConfig.h` | chân motor |
| `navPrintf()` + `navReport` | RobotNav (mục 5) | log của calibCellFromBackWall (đổi thành `Serial.printf` nếu không có) |
| `wrap180()` (static) | RobotNav.cpp:1123 | dùng ở turn + corner |

Thứ tự ghép gợi ý: (1) hằng số MainParams → (2) khai báo RobotNav.h → (3) hàm RobotNav.cpp → (4) gán trong `setup()` main.cpp → (5) nối vào luồng điều hướng (mục 6).

**Lưu ý quan trọng khi ghép:**
- `calibCellFromBackWall()` gọi `mpu6050.calibrate()` + `resetYaw()` → sau đó **yaw = 0, `_turnHeading = 0`**. Mọi code khác lưu yaw tuyệt đối trước đó sẽ sai (project này không lưu nên ổn).
- `calibCellFromBackWall()` dùng `hold.yawKp` với dấu **ngược** khi lùi (xem comment trong code).
- Giá trị `calibRevMs`/`calibRevPwm`/`calibFwdPulses` trong **RobotNav.h là 100ms / 60 / 1500** nhưng **MainParams.h ghi đè thành 400ms / 80 / 1500** (main.cpp chỉ gán trong nhánh SEQ_MODE 2 và 6). Khi ghép nhớ gán các giá trị này, nếu không sẽ dùng mặc định của .h.
- Tolerance quay: RobotNav.h mặc định `1.5`, MainParams `TURN_TOLERANCE 0.8` (main.cpp ghi đè ở setup).

---

## 1. THAM SỐ (MainParams.h)

```cpp
// ----- Cân bằng motor (ảnh hưởng cả quay/cua vì dùng motorScaleL/R) -----
#define MOTOR_TRIM_LEFT   -6.0f
#define MOTOR_TRIM_RIGHT  0.0f

// ----- Calib ô khi vào ngõ cụt -----
#define CALIB_REV_PWM        80     // PWM khi lùi sát vách cụt
#define CALIB_REV_MS         400    // thời gian lùi cố định (ms), rồi dừng, calib gyro về 0 độ
#define CALIB_FWD_PULSES     1500   // xung tiến từ vách tới tâm ô (không tính vào ô kế tiếp)
#define SEQ_CELL_PULSES      4300   // xung/ô trong sequence (tạm)
#define STOP_LEAD_PULSES     280    // phanh sớm chừng này xung trước đích

// ----- Ngưỡng tường dùng để quyết định rẽ / ngõ cụt (SEQ_MODE 6) -----
#define NAV_SIDE_WALL_MM  220   // VL53 trái/phải < mức này = có tường
#define NAV_FRONT_WALL_MM 150   // VL53 trước < mức này = tường chắn ngay cuối ô
#define NAV_MAX_STEPS     30
#define NAV_START_SIDES_WALL 1

// ----- CUA khi đang chạy (Corner Controller) -----
#define CORNER_SPEED       60.0f // PWM bánh ngoài
#define CORNER_INNER_RATIO 0.45f // PWM bánh trong / ngoài ở độ cong tối đa (nhỏ = gắt)
#define CORNER_DECEL_ZONE  30.0f // độ cuối: độ cong giảm dần về 0
#define CORNER_TOLERANCE   2.0f  // tới target khi sai số yaw <= độ này
#define CORNER_LEAD        0.05f // s: giảm cong sớm theo tốc độ quay

// ----- QUAY TẠI CHỖ -----
#define TURN_ANGLE       90.0f
#define TURN_RIGHT       1
#define TURN_MAX_PWM     70.0f
#define TURN_MIN_PWM     42.0f
#define TURN_DECEL_ZONE  35.0f
#define TURN_TOLERANCE   0.8f
#define TURN_BRAKE_LEAD  0.05f
#define TURN_LEFT_YAW_SIGN -1.0f // quay TRÁI làm yaw GIẢM. Quay sai chiều → đổi +1
```

Gán trong `setup()` (main.cpp:16-29, 137-139):

```cpp
robotNav.motorScaleL = 1.0f + MOTOR_TRIM_LEFT / 100.0f;
robotNav.motorScaleR = 1.0f + MOTOR_TRIM_RIGHT / 100.0f;
robotNav.turnMaxPwm = TURN_MAX_PWM;
robotNav.turnMinPwm = TURN_MIN_PWM;
robotNav.turnDecelZone = TURN_DECEL_ZONE;
robotNav.turnTolerance = TURN_TOLERANCE;
robotNav.turnLeftYawSign = TURN_LEFT_YAW_SIGN;
robotNav.turnBrakeLead = TURN_BRAKE_LEAD;
robotNav.cornerSpeed = CORNER_SPEED;
robotNav.cornerInnerRatio = CORNER_INNER_RATIO;
robotNav.cornerDecelZone = CORNER_DECEL_ZONE;
robotNav.cornerTolerance = CORNER_TOLERANCE;
robotNav.cornerLead = CORNER_LEAD;
// trước khi chạy dead-end / nav:
robotNav.seqCellPulses = SEQ_CELL_PULSES;
robotNav.calibRevPwm = CALIB_REV_PWM;
robotNav.calibRevMs = CALIB_REV_MS;
robotNav.calibFwdPulses = CALIB_FWD_PULSES;
robotNav.stopLeadPulses = STOP_LEAD_PULSES;   // đặt lại 0 sau khi chạy xong
```

---

## 2. KHAI BÁO (RobotNav.h)

enum (RobotNav.h:36):
```cpp
enum CornerState : uint8_t { STRAIGHT, CORNER_LEFT, CORNER_RIGHT, EXIT_CORNER };
```

Public (RobotNav.h:176-214, 230-231):
```cpp
  // Test quay tại chỗ theo yaw MPU6050 (blocking). Log lưu RAM, in bằng printTurnLog()
  float turnMaxPwm = 70.0f;
  float turnMinPwm = 42.0f;
  float turnDecelZone = 35.0f;
  float turnTolerance = 1.5f;
  float turnLeftYawSign = -1.0f; // quay TRÁI => yaw giảm
  float turnBrakeLead = 0.05f;   // s: phanh sớm khi sai số <= dung sai + tốc độ quay * giá trị này
  float _turnHeading = 0.0f;
  bool _turnHeadingValid = false;
  bool turnLeft90Test(float angle = 90.0f);   // angle > 0: TRÁI, < 0: PHẢI, 180 = quay đầu
  void printTurnLog();
  uint16_t seqCellPulses = 4200;

  // Cua 90° khi đang chạy (không quay tại chỗ)
  float cornerSpeed = 60.0f;
  float cornerInnerRatio = 0.45f;
  float cornerDecelZone = 30.0f;
  float cornerMinCurve = 0.2f;
  float cornerTolerance = 2.0f;
  float cornerLead = 0.05f;

  // Corner Controller (state machine không chặn). RobotNav::update() tự chạy updateCorner().
  CornerState cornerState = STRAIGHT;
  uint16_t cornerEntryMs = 120;
  uint16_t cornerExitMs = 150;
  uint16_t cornerTimeoutMs = 3500;
  float cornerStartYaw = 0.0f;
  float cornerTargetYaw = 0.0f;
  bool startCornerLeft();
  bool startCornerRight();
  void updateCorner();
  bool cornerBusy() const { return cornerState != STRAIGHT; }

  // Calib lại 1 ô khi vào ngõ cụt
  float calibRevPwm = 60.0f;
  uint16_t calibRevMs = 100;
  long calibFwdPulses = 1500;
  bool calibCellFromBackWall();
  void runDeadEndTest();

  void runCellToCellTest();   // test trái/phải 90 giữa các ô
  void runCornerNavTest();    // test cua state machine
  bool stepNoStop = false;    // true: đi hết quãng KHÔNG phanh, để chạy tiếp vào corner
```

Private (RobotNav.h:249, 285-291):
```cpp
  struct TurnRec { uint32_t t; float yaw, target, err; int16_t pl, pr; uint8_t st; }; // st: 0=TURN 1=CORNER 2=STRAIGHT
  static const uint16_t TURN_LOG_SIZE = 400;
  TurnRec _turnLog[TURN_LOG_SIZE];
  uint16_t _turnLogN = 0;
  bool _turnLogPrinted = true;

  float _cornerTarget = 0.0f;
  unsigned long _cornerT0 = 0, _cornerStateT = 0, _cornerPrevT = 0;
  float _cornerPrevYaw = 0.0f, _cornerV0 = 0.0f;
  bool startCorner(CornerState dir);
  void finishCorner();
  void cornerPwm(float err, float errPred, float outer, int &pl, int &pr) const;
```
Public thêm: `bool turnLogPending() const { return _turnLogN > 0 && !_turnLogPrinted; }`

Hook trong `RobotNav::update()` (RobotNav.cpp:527-531) — **phải có** để state machine cua chạy:
```cpp
  if (cornerState != STRAIGHT) {
    if (mpuReady) mpu6050.update();
    updateCorner();
    return;
  }
```

Hook trong `updatePIDLoop()` (RobotNav.cpp:423-430) — **phải có** để `stepNoStop` hoạt động (đi hết quãng mà không phanh, vào thẳng corner):
```cpp
    if ((pulseReached || timeoutSafe) && stepNoStop) {
      stepNoStop = false;
      stepCellActive = false;
      pidRunActive = false;
      holdRun = false;
      return;
    }
```
và dòng giảm tốc cuối ô (RobotNav.cpp:502): `if (!holdRun && stepCellActive && !stepNoStop && ...)`.

---

## 3. QUAY TẠI CHỖ 90°/180° (dùng yaw MPU) — RobotNav.cpp:1121-1218

```cpp
static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

bool RobotNav::turnLeft90Test(float angle) {
  if (!mpuReady) return false;
  stopPID();
  mpu6050.setAutoBias(false);
  mpu6050.update();
  // Target cộng dồn từ target lần trước (không lấy từ yaw đo được) -> sai số dư của lần quay trước được sửa ở lần sau,
  // không cộng dồn. Nếu xe bị xoay tay/lệch > 20° so với target cũ thì lấy lại từ yaw hiện tại.
  float base = mpu6050.getYaw();
  if (_turnHeadingValid && fabsf(_turnHeading - base) < 20.0f) base = _turnHeading;
  // Target KHÔNG chuẩn hóa: yaw của MPU là góc tích lũy (không quấn vòng) nên sai số = target - yaw luôn đúng chiều,
  // kể cả quay 180° (chuẩn hóa về [-180,180] sẽ làm xe quay ngược chiều khi target rơi sát ±180)
  float target = base + turnLeftYawSign * angle; // angle > 0: quay TRÁI, angle < 0: quay PHẢI
  _turnHeading = target;
  _turnHeadingValid = true;
  float prevYaw = mpu6050.getYaw();
  unsigned long prevT = millis();
  _turnLogN = 0;
  _turnLogPrinted = false;

  unsigned long t0 = millis(), lastLog = 0;
  bool done = false;
  while (!done && millis() - t0 < 6000) {
    mpu6050.update();
    float yaw = mpu6050.getYaw();
    float err = target - yaw;  // err > 0: yaw cần tăng
    int pl = 0, pr = 0;
    // Tốc độ quay (độ/s) để dự đoán quãng trôi sau khi phanh
    unsigned long nowT = millis();
    float rate = (nowT > prevT) ? (yaw - prevYaw) * 1000.0f / (float)(nowT - prevT) : 0.0f;
    prevYaw = yaw;
    prevT = nowT;
    // Phanh sớm: sai số còn lại <= dung sai + (tốc độ * turnBrakeLead). Quãng trôi sau phanh ~ rate * lead
    float lead = fabsf(rate) * turnBrakeLead;
    bool approaching = (rate * err) > 0.0f;  // đang quay về phía target

    if (fabsf(err) <= turnTolerance || (approaching && fabsf(err) <= turnTolerance + lead)) {
      // Tới target: phanh ngắn mạch. PHẢI cập nhật gyro liên tục trong lúc phanh (không delay mù),
      // nếu không phần góc xe còn quay khi phanh không được tích phân -> yaw báo thiếu, xe quay quá
      analogWrite(M1_IN1, 255); analogWrite(M1_IN2, 255);
      analogWrite(M2_IN1, 255); analogWrite(M2_IN2, 255);
      unsigned long s = millis();
      while (millis() - s < 80) { mpu6050.update(); delay(1); }
      stopMotors();
      s = millis();
      while (millis() - s < 150) { mpu6050.update(); delay(1); }
      yaw = mpu6050.getYaw();
      err = target - yaw;
      prevYaw = yaw;
      prevT = millis();
      if (fabsf(err) <= turnTolerance) done = true;
    } else {
      // Giảm tốc tuyến tính trong turnDecelZone cuối
      float k = constrain(fabsf(err) / turnDecelZone, 0.0f, 1.0f);
      int pwm = (int)lroundf(turnMinPwm + (turnMaxPwm - turnMinPwm) * k);
      // turnLeftYawSign * err > 0 => cần quay TRÁI (bánh trái lùi, bánh phải tiến), ngược lại quay PHẢI
      bool goLeft = (turnLeftYawSign * err) > 0.0f;
      if (goLeft) { pl = -pwm; pr = pwm; } else { pl = pwm; pr = -pwm; }
      analogWrite(M1_IN1, pl > 0 ? pl : 0); analogWrite(M1_IN2, pl < 0 ? -pl : 0);
      analogWrite(M2_IN1, pr > 0 ? pr : 0); analogWrite(M2_IN2, pr < 0 ? -pr : 0);
    }

    if (millis() - lastLog >= 20 && _turnLogN < TURN_LOG_SIZE) {
      lastLog = millis();
      _turnLog[_turnLogN++] = {(uint32_t)millis(), yaw, wrap180(target), err, (int16_t)pl, (int16_t)pr};
    }
    delay(2);
  }

  stopMotors();  // motor = 0
  mpu6050.setAutoBias(true);
  currentLeftSpeed = currentRightSpeed = 0;
  if (_turnLogN < TURN_LOG_SIZE) {
    mpu6050.update();
    float yaw = mpu6050.getYaw();
    _turnLog[_turnLogN++] = {(uint32_t)millis(), yaw, wrap180(target), target - yaw, 0, 0};
  }
  return done;
}

void RobotNav::printTurnLog() {
  Serial.println("==== TURN/CORNER LOG: TIME,STATE,YAW,TARGET,ERROR,PWM_L,PWM_R (PWM am = lui) ====");
  for (uint16_t i = 0; i < _turnLogN; i++) {
    const TurnRec &e = _turnLog[i];
    static const char *ST[] = {"TURN", "CORNER", "STRAIGHT"};
    Serial.printf("%lu,%s,%.2f,%.2f,%.2f,%d,%d\n", (unsigned long)e.t, ST[e.st > 2 ? 0 : e.st], e.yaw, e.target, e.err, e.pl, e.pr);
  }
  Serial.printf("==== HET (%u mau) ====\n", _turnLogN);
  _turnLogPrinted = true;
}
```

Cách gọi: `turnLeft90Test(90.0f)` trái 90, `turnLeft90Test(-90.0f)` phải 90, `turnLeft90Test(180.0f)` quay đầu. Sau mỗi lần quay nên `settle(300)` (vòng `mpu6050.update(); delay(2);` 300ms).

> Còn một đường quay cũ (blocking, không có hồi tiếp tolerance): `RobotNav::turnLeft/turnRight` (RobotNav.cpp:205-234) → `MPU6050::rotateToAngle` (MPU6050.cpp:158-270), dùng `leftCompensation/rightCompensation/turnSpeed`. Đường này dùng cho chế độ 3/4 (bám tường, flood-fill) qua `turnLeftAndStep/turnRightAndStep/turnAroundAndStep` (RobotNav.cpp:720-751). Nếu project đích dùng `turnLeft90Test` thì KHÔNG cần các hàm cũ này.

---

## 4. CUA 90° KHI ĐANG CHẠY (state machine) — RobotNav.cpp:1220-1355

```cpp
static void driveFwd(int l, int r) {
  analogWrite(M1_IN1, constrain(l, 0, 255)); analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, constrain(r, 0, 255)); analogWrite(M2_IN2, 0);
}

// PWM 2 bánh khi cua: bánh ngoài = outer, bánh trong giảm theo độ cong (0..1, tỉ lệ sai số trong vùng giảm tốc)
void RobotNav::cornerPwm(float err, float errPred, float outer, int &pl, int &pr) const {
  float u = constrain(fabsf(errPred) / cornerDecelZone, cornerMinCurve, 1.0f);
  float inner = outer * (1.0f - (1.0f - cornerInnerRatio) * u);
  bool goLeft = (turnLeftYawSign * err) > 0.0f;  // cần làm yaw đổi theo chiều quay TRÁI
  pl = (int)lroundf((goLeft ? inner : outer) * motorScaleL);
  pr = (int)lroundf((goLeft ? outer : inner) * motorScaleR);
}

bool RobotNav::startCornerLeft()  { return startCorner(CORNER_LEFT); }
bool RobotNav::startCornerRight() { return startCorner(CORNER_RIGHT); }

bool RobotNav::startCorner(CornerState dir) {
  if (!mpuReady || cornerState != STRAIGHT) return false;
  mpu6050.setAutoBias(false);
  mpu6050.update();
  cornerStartYaw = mpu6050.getYaw();
  // Trái: yaw đổi theo turnLeftYawSign (mặc định -1 => startYaw - 90). Phải: ngược lại
  float delta = (dir == CORNER_LEFT ? 1.0f : -1.0f) * turnLeftYawSign * 90.0f;
  _cornerTarget = cornerStartYaw + delta;
  cornerTargetYaw = wrap180(_cornerTarget);
  _cornerV0 = (currentLeftSpeed + currentRightSpeed) * 0.5f; // tốc độ lúc vào cua, giảm dần về cornerSpeed
  if (_cornerV0 < cornerSpeed) _cornerV0 = cornerSpeed;
  _cornerT0 = _cornerStateT = _cornerPrevT = millis();
  _cornerPrevYaw = cornerStartYaw;
  cornerState = dir;
  Serial.printf(">> [CORNERSM] START %s | yaw=%.2f target=%.2f\n",
                dir == CORNER_LEFT ? "TRAI" : "PHAI", cornerStartYaw, cornerTargetYaw);
  return true;
}

void RobotNav::updateCorner() {
  if (cornerState == STRAIGHT) return;
  if (!mpuReady) { finishCorner(); return; }
  unsigned long nowT = millis();
  float yaw = mpu6050.getYaw();
  int pl, pr;

  if (cornerState == EXIT_CORNER) {
    // Hai bánh về lại tốc độ chạy thẳng, không còn lệch trái/phải
    float t = cornerExitMs ? constrain((nowT - _cornerStateT) / (float)cornerExitMs, 0.0f, 1.0f) : 1.0f;
    float v = cornerSpeed + ((float)baseForwardSpeed - cornerSpeed) * t;
    pl = (int)lroundf(v * motorScaleL);
    pr = (int)lroundf(v * motorScaleR);
    driveFwd(pl, pr);
    currentLeftSpeed = pl;
    currentRightSpeed = pr;
    if (t >= 1.0f) finishCorner();
    return;
  }

  float err = wrap180(_cornerTarget - yaw);
  float rate = (nowT > _cornerPrevT) ? wrap180(yaw - _cornerPrevYaw) * 1000.0f / (float)(nowT - _cornerPrevT) : 0.0f;
  _cornerPrevYaw = yaw;
  _cornerPrevT = nowT;
  float errPred = err - rate * cornerLead;
  bool timeout = nowT - _cornerT0 > cornerTimeoutMs;
  if (fabsf(err) <= cornerTolerance || errPred * err <= 0.0f || timeout) {
    Serial.printf(">> [CORNERSM] %s | yaw=%.2f target=%.2f err=%.2f -> EXIT\n",
                  timeout ? "TIMEOUT" : "REACHED", yaw, cornerTargetYaw, err);
    cornerState = EXIT_CORNER;
    _cornerStateT = nowT;
    return;
  }

  // Vào cua: bánh ngoài giảm tốc từ tốc độ lúc vào về cornerSpeed, rồi cua với độ cong theo sai số yaw
  float t = cornerEntryMs ? constrain((nowT - _cornerT0) / (float)cornerEntryMs, 0.0f, 1.0f) : 1.0f;
  float outer = _cornerV0 + (cornerSpeed - _cornerV0) * t;
  cornerPwm(err, errPred, outer, pl, pr);
  driveFwd(pl, pr);
  currentLeftSpeed = pl;
  currentRightSpeed = pr;
}

void RobotNav::finishCorner() {
  cornerState = STRAIGHT;
  // Giữ hướng mới cho đoạn thẳng kế tiếp, xóa trạng thái PID cũ để không giật
  _targetYaw = _cornerTarget;
  _turnHeading = _cornerTarget;
  _turnHeadingValid = true;
  long curL = getLeftEncoder(), curR = getRightEncoder();
  _lastEncLeft = _startEncLeft = curL;   // enc PID tính chênh lệch từ đầu đoạn thẳng mới
  _lastEncRight = _startEncRight = curR;
  if (stepCellActive) { stepStartL = curL; stepStartR = curR; } // đoạn thẳng sau cua đếm quãng đường lại từ đầu
  _lastPIDLoopTime = micros();
  Serial.printf(">> [CORNERSM] STRAIGHT | yaw=%.2f heading=%.2f PWM %d/%d\n",
                mpu6050.getYaw(), cornerTargetYaw, currentLeftSpeed, currentRightSpeed);
}

// Test: đi 1 ô (không dừng) -> cua trái 90° bằng state machine -> đi 1 ô -> dừng
void RobotNav::runCornerNavTest() {
  if (!mpuReady) { Serial.println(">> [CORNERNAV] MPU6050 chua san sang, huy"); return; }
  stopPID();
  unsigned long s = millis();
  while (millis() - s < 200) { mpu6050.update(); delay(2); }

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = seqCellPulses;
  mpu6050.update();
  _turnHeading = mpu6050.getYaw();
  _turnHeadingValid = true;
  clearLog();

  auto move = [&](bool noStop) {
    startHold(true);
    _targetYaw = _turnHeading;
    stepNoStop = noStop;
    unsigned long t0 = millis();
    while (pidRunActive && millis() - t0 < 15000) update();
    if (pidRunActive) stopPID();
    stepNoStop = false;
  };

  move(true);                       // đi 1 ô, KHÔNG phanh cuối ô
  if (startCornerLeft()) {          // hoặc startCornerRight()
    unsigned long t0 = millis();
    while (cornerBusy() && millis() - t0 < 6000) update();
  }
  move(false);
  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  Serial.println(">> [CORNERNAV] DONE - PWM = 0");
}
```

Quy trình dùng cua: `stepNoStop = true` → chạy hết ô (không phanh) → `startCornerLeft()/Right()` → lặp `update()` tới khi `!cornerBusy()` → chạy tiếp `startHold()`.

---

## 5. NGÕ CỤT: QUAY 180° → LÙI SÁT VÁCH → CALIB GYRO → TIẾN TỚI TÂM Ô — RobotNav.cpp:1414-1531

### 5a. `navPrintf` (RobotNav.cpp:1549-1558) — calibCellFromBackWall dùng hàm này

```cpp
// khai báo trong RobotNav.h:  String navReport; bool navReportPending = false;
//   void navPrintf(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
void RobotNav::navPrintf(const char *fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  if (navReport.length() < 8000) navReport += buf;
}
```
(Nếu không muốn ghép navPrintf: thay `navPrintf(` bằng `Serial.printf(` trong hàm dưới.)

### 5b. `calibCellFromBackWall()` — HÀM CHÍNH

```cpp
// Dùng khi vào đường cụt, sau khi quay 180°: lùi calibRevMs cho lưng xe sát vách, đứng yên,
// calib lại gyro và đặt yaw = 0, rồi tiến calibFwdPulses xung. Từ điểm dừng này coi như xe đang ở GIỮA Ô hiện tại;
// quãng calibFwdPulses không tính vào quãng 1 ô kế tiếp (ô kế tiếp đếm lại từ 0).
bool RobotNav::calibCellFromBackWall() {
  if (!mpuReady) { navPrintf(">> [CALIBCELL] MPU6050 chua san sang, huy" "\n"); return false; }
  stopPID();
  resetEnc();
  mpu6050.update();
  float target = _turnHeadingValid ? _turnHeading : mpu6050.getYaw(); // giữ hướng sau lần quay 180° trong lúc lùi ngắn

  navPrintf(">> [CALIBCELL] LUI %u ms | PWM %.0f | yaw=%.2f\n", (unsigned)calibRevMs, calibRevPwm, mpu6050.getYaw());
  unsigned long t0 = millis();
  while (millis() - t0 < calibRevMs) {
    mpu6050.update();
    float e = target - mpu6050.getYaw();
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    // Lùi: bánh phải lùi nhanh hơn thì xe xoay phải (yaw tăng), ngược chiều với khi tiến -> đảo dấu so với khi chạy thẳng
    float corr = constrain(hold.yawKp * e, -10.0f, 10.0f);
    int pl = constrain((int)lroundf((calibRevPwm - corr) * motorScaleL), 0, 255);
    int pr = constrain((int)lroundf((calibRevPwm + corr) * motorScaleR), 0, 255);
    analogWrite(M1_IN1, 0); analogWrite(M1_IN2, pl);
    analogWrite(M2_IN1, 0); analogWrite(M2_IN2, pr);
    currentLeftSpeed = -pl;
    currentRightSpeed = -pr;
    delay(2);
  }
  stopMotors();
  long reversed = (labs(getLeftEncoder()) + labs(getRightEncoder())) / 2;
  navPrintf(">> [CALIBCELL] DA LUI %ld xung | yaw=%.2f -> calib gyro, giu xe dung yen\n", reversed, mpu6050.getYaw());

  unsigned long s = millis();
  while (millis() - s < 300) { mpu6050.update(); delay(2); } // đứng yên hẳn trước khi calib gyro
  mpu6050.calibrate();
  mpu6050.resetYaw();                                          // yaw = 0 là hướng mới
  _turnHeading = 0.0f;
  _turnHeadingValid = true;
  navPrintf(">> [CALIBCELL] GYRO CALIB XONG, YAW = %.2f | tien %ld xung toi giua o\n", mpu6050.getYaw(), calibFwdPulses);

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = calibFwdPulses;
  startHold(true);              // resetEnc() trong startHold: đếm lại từ 0
  _targetYaw = _turnHeading;
  unsigned long t1 = millis();
  while (pidRunActive && millis() - t1 < 15000) update();
  if (pidRunActive) stopPID();
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  s = millis();
  while (millis() - s < 300) { mpu6050.update(); delay(2); }
  navPrintf(">> [CALIBCELL] XONG, DANG O GIUA O | tien %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());
  return true;
}
```

Những gì được "calib lại" trong hàm này:
| Thông số | Cách calib |
|---|---|
| Vị trí dọc ô | lùi `calibRevMs` ms @ `calibRevPwm` cho lưng chạm vách → tiến `calibFwdPulses` xung = tâm ô |
| Encoder | `resetEnc()` đầu hàm + `startHold()` reset lại trước khi tiến |
| Gyro offset | `mpu6050.calibrate()` (300 mẫu, xe đứng yên 300ms trước đó) |
| Yaw / heading | `mpu6050.resetYaw()`, `_turnHeading = 0`, `_targetYaw = 0` |
| Quãng ô kế tiếp | `pulsesPerCell` được khôi phục, ô kế đếm lại từ 0 |

### 5c. `runDeadEndTest()` — luồng test (RobotNav.cpp:1473-1531)

```cpp
void RobotNav::runDeadEndTest() {
  if (!mpuReady) { Serial.println(">> [DEADEND] MPU6050 chua san sang, huy"); return; }
  stopPID();

  auto settle = [&](unsigned long ms) {
    unsigned long s = millis();
    while (millis() - s < ms) { mpu6050.update(); delay(2); }
  };

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = 1;
  pulsesPerCell = seqCellPulses;

  settle(200);
  mpu6050.update();
  _turnHeading = mpu6050.getYaw();
  _turnHeadingValid = true;
  clearLog();
  Serial.printf(">> [DEADEND] BAT DAU | %u xung/o | yaw0=%.2f\n", (unsigned)seqCellPulses, _turnHeading);

  Serial.println(">> [DEADEND] 1/3 MOVE 1 O (coi nhu vao ngo cut)");
  startHold(true);
  _targetYaw = _turnHeading;
  unsigned long t0 = millis();
  while (pidRunActive && millis() - t0 < 15000) update();
  if (pidRunActive) stopPID();
  settle(300);
  Serial.printf(">>        MOVE xong: %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());

  Serial.println(">> [DEADEND] 2/3 QUAY 180");
  bool ok = turnLeft90Test(180.0f);
  settle(300);
  Serial.printf(">>        QUAY xong (%s) | yaw=%.2f (chuan %.2f)\n", ok ? "OK" : "TIMEOUT", mpu6050.getYaw(), _turnHeading);

  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  Serial.println(">> [DEADEND] 3/3 CALIB LAI O");
  if (calibCellFromBackWall()) {
    // Đang ở giữa ô sau calib: đi thẳng thêm 2 ô (hướng yaw = 0 vừa calib)
    Serial.println(">> [DEADEND] 4/4 MOVE 2 O");
    hold.cells = 2;
    pulsesPerCell = seqCellPulses;
    startHold(true);
    _targetYaw = _turnHeading;
    unsigned long t2 = millis();
    while (pidRunActive && millis() - t2 < 20000) update();
    if (pidRunActive) stopPID();
    hold.cells = savedCells;
    pulsesPerCell = savedPulses;
    unsigned long s2 = millis();
    while (millis() - s2 < 300) { mpu6050.update(); delay(2); }
    Serial.printf(">>        MOVE xong: %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());
  }

  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  Serial.println(">> [DEADEND] DONE - PWM = 0");
}
```

### 5d. Phụ thuộc MPU6050 của calib (lib/MPU6050)

```cpp
// MPU6050.h
void calibrate(int samples = 300);
void resetYaw();                       // _yaw = 0; _yawSetpoint = 0
void setAutoBias(bool on) { _autoBias = on; _stillTime = 0.0f; }
// MPU6050.cpp:81-112  calibrate(): bỏ 30 mẫu đầu, lấy trung bình gz => _gyroZOffset, rồi _yaw = 0, _lastTime = micros()
```
Xe PHẢI đứng yên khi `calibrate()` (~1 giây: (300+30)×3ms).

---

## 6. NỐI VÀO LUỒNG ĐIỀU HƯỚNG: khi nào quay / cua / ngõ cụt

Logic quyết định trong `runCellNavTest()` (RobotNav.cpp:1653-1678) — đây là chỗ để copy sang main nav của project đích:

```cpp
    if (wf == WS_OPEN) {                         // trước thông: đi thẳng 1 ô
      moveOne();  cellsMoved++;
      curL = nxtL; curR = nxtR;                  // vách hông ô mới = số đọc nhìn-trước
    } else if (curL == WS_OPEN) {                // trước tường, trái thông: dừng → quay trái 90°
      if (!turnDeg(90.0f, "TRAI 90")) break;
      curL = WS_UNKNOWN; curR = WS_WALL;
    } else if (curR == WS_OPEN) {                // phải thông: quay phải 90°
      if (!turnDeg(-90.0f, "PHAI 90")) break;
      curL = WS_WALL;   curR = WS_UNKNOWN;
    } else if (curL == WS_WALL && curR == WS_WALL) {  // 3 phía tường = NGÕ CỤT
      if (!turnDeg(180.0f, "180")) break;
      if (!calibCellFromBackWall()) break;       // lùi → calib gyro → tiến tới tâm ô
      curL = WS_WALL; curR = WS_WALL;
    } else { break; }                            // hông không xác định: không đoán
```
với lambda dùng kèm:
```cpp
  auto settle = [&](unsigned long ms) { unsigned long s = millis(); while (millis() - s < ms) { mpu6050.update(); delay(2); } };
  auto moveOne = [&]() {
    startHold(true);
    _targetYaw = _turnHeading;
    unsigned long t0 = millis();
    while (pidRunActive && millis() - t0 < 15000) update();
    if (pidRunActive) stopPID();
    settle(300);
  };
  auto turnDeg = [&](float angle, const char *name) -> bool {   // dương = trái, âm = phải
    bool ok = turnLeft90Test(angle);
    settle(300);
    return ok;
  };
```
Phát hiện tường dùng: `classifyWall()`, `senseWallsSafe()` (RobotNav.cpp:1537-1594, ngưỡng `navSideWallMm` / `navFrontWallMm`) — copy theo nếu cần quyết định rẽ/ngõ cụt giống project này.

Nhánh gọi trong main.cpp (loop, nút BUTTON) — tham khảo:
```cpp
#if SEQ_MODE == 2      // ngõ cụt
  robotNav.seqCellPulses = SEQ_CELL_PULSES;
  robotNav.calibRevPwm = CALIB_REV_PWM;
  robotNav.calibRevMs = CALIB_REV_MS;
  robotNav.calibFwdPulses = CALIB_FWD_PULSES;
  robotNav.stopLeadPulses = STOP_LEAD_PULSES;
  robotNav.runDeadEndTest();
  robotNav.stopLeadPulses = 0;
#elif SEQ_MODE == 4    // cua state machine
  robotNav.seqCellPulses = SEQ_CELL_PULSES;
  robotNav.runCornerNavTest();
#elif SEQ_MODE == 1    // trái/phải 90 giữa các ô
  ...runCellToCellTest();
#else                  // quay tại chỗ
  robotNav.turnLeft90Test(TURN_RIGHT ? -TURN_ANGLE : TURN_ANGLE);
#endif
// in log quay khi cắm USB:
if (robotNav.turnLogPending() && (bool)Serial) { delay(300); robotNav.printTurnLog(); }
```

---

## 7. KHÔNG cần ghép (không liên quan quay/ngõ cụt)
`runCalibTest` (calib encoder 4 ô), `runSensorStreamTest`, flood-fill (`stepFloodFill`…), `stepAutoWallFollow`, log `printLog`, `saveForwardSpeed`.
Ghi chú: `runCellToCellTest()` (RobotNav.cpp:1359-1411) là test 1 ô → trái 90 → 1 ô → phải 90 → 1 ô, chỉ là ví dụ gọi `turnLeft90Test` giữa các `startHold`; không bắt buộc.
