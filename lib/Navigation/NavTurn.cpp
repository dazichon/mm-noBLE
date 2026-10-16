// QUAY TẠI CHỖ / CUA KHI ĐANG CHẠY / CALIB Ô NGÕ CỤT (MPU6050 feedback)
#include "RobotNav.h"
#include "NavInternal.h"

void RobotNav::turnRight(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    serialLink.println(msg);
    return;
  }
  bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         rightCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
  Serial.println(res);
  serialLink.println(res);
}

void RobotNav::turnLeft(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    serialLink.println(msg);
    return;
  }

  bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         leftCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
  Serial.println(res);
  serialLink.println(res);
}

// ================= TEST QUAY TRÁI TẠI CHỖ (MPU6050 feedback) =================

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
  float stallYaw = prevYaw;     // phát hiện kẹt: yaw không đổi dù đang cấp PWM
  unsigned long stallT = millis();
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
      while (millis() - s < 60) { mpu6050.update(); delay(1); }
      stopMotors();
      s = millis();
      while (millis() - s < 80) { mpu6050.update(); delay(1); }
      yaw = mpu6050.getYaw();
      err = target - yaw;
      prevYaw = yaw;
      prevT = millis();
      if (fabsf(err) <= turnTolerance) done = true;
    } else if (fabsf(err) <= 2.0f * turnTolerance + 1.0f && fabsf(yaw - stallYaw) < 0.1f && nowT - stallT > 400) {
      // Gần target nhưng PWM nhỏ không đủ thắng ma sát (yaw đứng im 400ms): chấp nhận, không đẩy mãi tới timeout
      done = true;
    } else {
      if (fabsf(yaw - stallYaw) >= 0.1f) { stallYaw = yaw; stallT = nowT; }
      // Giảm tốc tuyến tính trong turnDecelZone cuối
      float k = constrain(fabsf(err) / turnDecelZone, 0.0f, 1.0f);
      int pwm = (int)lroundf(turnMinPwm + (turnMaxPwm - turnMinPwm) * k);
      // Cần quay theo chiều làm yaw thay đổi cùng dấu với err.
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

// ================= MOTION PRIMITIVE: CORNER 90° (cua khi đang chạy) =================

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

// ================= CORNER CONTROLLER (state machine không chặn) =================

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

// ================= CALIB LẠI 1 Ô: lùi ngắn -> calib gyro về 0° -> tiến calibFwdPulses xung tới tâm ô =================
// Dùng khi vào đường cụt, sau khi quay 180°: lùi calibRevMs (mặc định ~0.1s) cho lưng xe sát vách, đứng yên,
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
  long stallEnc = 0;
  unsigned long stallT = millis();
  while (millis() - t0 < calibRevMs) {
    // Chạm vách: encoder gần như đứng yên 60ms (sau 150ms đầu) -> dừng ngay, không đẩy tiếp vào tường
    long encNow = labs(getLeftEncoder()) + labs(getRightEncoder());
    if (labs(encNow - stallEnc) > 6) { stallEnc = encNow; stallT = millis(); }
    else if (millis() - t0 > 150 && millis() - stallT > 60) break;
    mpu6050.update();
    float e = target - mpu6050.getYaw();
    while (e > 180.0f) e -= 360.0f;
    while (e < -180.0f) e += 360.0f;
    // Lùi: bánh phải lùi nhanh hơn thì xe xoay phải (yaw tăng), ngược chiều với khi tiến -> đảo dấu so với khi chạy thẳng
    float corr = constrain(hold.yawKp * e, -10.0f, 10.0f);
    // Lùi nhẹ: tăng dần từ 40 lên calibRevPwm trong 150ms đầu
    float revPwm = min(calibRevPwm, 40.0f + (calibRevPwm - 40.0f) * (millis() - t0) / 150.0f);
    int pl = constrain((int)lroundf((revPwm - corr) * motorScaleL), 0, 255);
    int pr = constrain((int)lroundf((revPwm + corr) * motorScaleR), 0, 255);
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
  while (millis() - s < 150) { mpu6050.update(); delay(2); } // đứng yên hẳn trước khi calib gyro
  mpu6050.calibrate(120); // 120 mẫu (~0.45s) thay vì 300 (~1s): đủ chính xác khi xe đứng yên
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
  while (millis() - s < 100) { mpu6050.update(); delay(2); }
  navPrintf(">> [CALIBCELL] XONG, DANG O GIUA O | tien %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());
  return true;
}

// ================= CĂN THEO TƯỜNG TRƯỚC MẶT =================

static int16_t median3(int16_t a, int16_t b, int16_t c) {
  return max(min(a, b), min(max(a, b), c));
}

bool RobotNav::alignFrontWall() {
  if (!frontReady) return false;
  stopPID();
  if (mpuReady) mpu6050.update();
  float yawT = (_turnHeadingValid || !mpuReady) ? _turnHeading : mpu6050.getYaw();
  int16_t buf[3] = {0, 0, 0};
  uint8_t n = 0;
  unsigned long lastF = _tF, t0 = millis(), okSince = 0;
  float d = -1.0f, progD = -100.0f;
  unsigned long progT = millis();
  bool ok = false, firstRead = true;

  while (millis() - t0 < frontAlignMaxMs) {
    updateSensors();
    if (mpuReady) mpu6050.update();
    if (_tF != lastF) {
      lastF = _tF;
      if (_rawF > 0 && _rawF < 400) { buf[n % 3] = _rawF; n++; }
    }
    if (n < 3) { delay(2); continue; }
    d = median3(buf[0], buf[1], buf[2]);
    float e = d - frontAlignMm;  // > 0: còn xa tường -> tiến
    // Lệch ít hơn frontAlignSkip ngay lần đọc đầu: bỏ qua, không căn (chỉ sửa khi lệch rõ)
    if (firstRead) {
      firstRead = false;
      if (fabsf(e) <= frontAlignSkip) { ok = true; break; }
    }
    if (fabsf(e) <= frontAlignTol) {
      stopMotors();
      if (okSince == 0) okSince = millis();
      if (millis() - okSince > 100) { ok = true; break; }  // đứng yên trong dung sai 100ms
      delay(2);
      continue;
    }
    okSince = 0;
    // Xe đứng yên cần lực lớn hơn để nhích: khoảng cách không đổi (> 2mm) thì tăng dần PWM, 1 PWM mỗi 15ms
    if (fabsf(d - progD) > 2.0f) { progD = d; progT = millis(); }
    float boost = (millis() - progT) / 30.0f;
    float pwm = constrain(fabsf(e) * frontAlignKp + boost, frontAlignMinPwm, frontAlignMaxPwm);
    // Giữ hướng: cùng quy ước với PID đi thẳng (corr > 0 -> quay trái); khi lùi đảo dấu
    float corr = 0.0f;
    if (mpuReady) corr = constrain(-hold.yawKp * wrap180(yawT - mpu6050.getYaw()), -10.0f, 10.0f);
    if (e < 0) corr = -corr;
    int pl = constrain((int)lroundf((pwm - corr) * motorScaleL), 0, 255);
    int pr = constrain((int)lroundf((pwm + corr) * motorScaleR), 0, 255);
    if (e > 0) {
      analogWrite(M1_IN1, pl); analogWrite(M1_IN2, 0);
      analogWrite(M2_IN1, pr); analogWrite(M2_IN2, 0);
    } else {
      analogWrite(M1_IN1, 0); analogWrite(M1_IN2, pl);
      analogWrite(M2_IN1, 0); analogWrite(M2_IN2, pr);
    }
    delay(2);
  }
  stopMotors();
  navPrintf(">> [ALIGN] TUONG TRUOC: %s | VL53 truoc = %.0f mm (dich %.0f +-%.0f)\n",
            ok ? "OK" : "HET GIO", d, frontAlignMm, frontAlignTol);
  return ok;
}
