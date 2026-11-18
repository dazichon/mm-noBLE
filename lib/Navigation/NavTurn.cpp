// QUAY TẠI CHỖ / CUA KHI ĐANG CHẠY / CALIB Ô NGÕ CỤT (MPU6050 feedback)
#include "RobotNav.h"
#include "NavInternal.h"

void RobotNav::turnRight(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    return;
  }
  bool completed = mpu6050.rotateToAngle(-angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         rightCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE PHAI XONG" : ">> CANH BAO: RE PHAI TIMEOUT";
  Serial.println(res);
}

void RobotNav::turnLeft(float angle) {
  if (!mpuReady) {
    String msg = "MPU6050 chua san sang, khong the quay!";
    Serial.println(msg);
    return;
  }

  bool completed = mpu6050.rotateToAngle(angle, M1_IN1, M1_IN2, M2_IN1, M2_IN2,
                                         leftCompensation, turnSpeed);
  String res =
      completed ? ">> DA RE TRAI XONG" : ">> CANH BAO: RE TRAI TIMEOUT";
  Serial.println(res);
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
  _absHeading += turnLeftYawSign * angle;          // hướng chuẩn tuyệt đối: cộng đúng góc lý tưởng
  _yawFresh = false;
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
  while (!done && !i2cFault && millis() - t0 < 6000) {
    mpu6050.update();
    if (mpu6050.readFails() >= 50) { enterI2CFault("MPU6050 khong tra loi"); break; }
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

// ================= CALIB LẠI 1 Ô: lùi ngắn -> calib gyro về 0° -> tiến calibFwdPulses xung tới tâm ô =================
// Dùng khi vào đường cụt, sau khi quay 180°: lùi calibRevMs (mặc định ~0.1s) cho lưng xe sát vách, đứng yên,
// calib lại gyro và đặt yaw = 0, rồi tiến calibFwdPulses xung. Từ điểm dừng này coi như xe đang ở GIỮA Ô hiện tại;
// quãng calibFwdPulses không tính vào quãng 1 ô kế tiếp (ô kế tiếp đếm lại từ 0).
bool RobotNav::calibCellFromBackWall(bool noForward) {
  if (!mpuReady) { navPrintf(">> [CALIBCELL] MPU6050 chua san sang, huy" "\n"); return false; }
  stopPID();
  resetEnc();
  mpu6050.update();
  float target = _turnHeadingValid ? _turnHeading : mpu6050.getYaw(); // giữ hướng sau lần quay 180° trong lúc lùi ngắn

  navPrintf(">> [CALIBCELL] LUI %u ms | PWM %.0f | yaw=%.2f\n", (unsigned)calibRevMs, calibRevPwm, mpu6050.getYaw());
  unsigned long t0 = millis();
  long stallEnc = 0;
  unsigned long stallT = millis();
  while (millis() - t0 < calibRevMs && !i2cFault) {
    // Chạm vách: đã lùi đủ calibRevMinPulses VÀ encoder đứng yên 100ms -> dừng (tránh dừng sớm khi xe mới nhích chậm)
    long encNow = labs(getLeftEncoder()) + labs(getRightEncoder());
    if (labs(encNow - stallEnc) > 6) { stallEnc = encNow; stallT = millis(); }
    else if (encNow / 2 >= calibRevMinPulses && millis() - stallT > 100) break;
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
  // Ép thêm 2 bánh đều lực một chút cho lưng xe áp phẳng vào vách (không sửa hướng: vách tự nắn thẳng xe)
  {
    int p = constrain((int)lroundf(calibRevPwm), 0, 255);
    analogWrite(M1_IN1, 0); analogWrite(M1_IN2, (int)(p * motorScaleL));
    analogWrite(M2_IN1, 0); analogWrite(M2_IN2, (int)(p * motorScaleR));
    unsigned long sq = millis();
    while (millis() - sq < calibSquareMs) { mpu6050.update(); delay(2); }
  }
  stopMotors();
  long reversed = (labs(getLeftEncoder()) + labs(getRightEncoder())) / 2;
  navPrintf(">> [CALIBCELL] DA LUI %ld xung | yaw=%.2f -> calib gyro, giu xe dung yen\n", reversed, mpu6050.getYaw());

  // Gyro đã calib 1 lần lúc bật nguồn (RobotNav::init) -> ở đây CHỈ đặt yaw = 0, không calib lại (không đứng chờ)
  const bool doCalib = false;
  unsigned long s = millis();
  while (millis() - s < 30UL) { mpu6050.update(); delay(2); }  // 30ms cho hết nảy sau khi ép vào tường
  mpu6050.resetYaw();                                          // yaw = 0 là hướng mới
  _turnHeading = 0.0f;
  _yawFresh = true;     // tới lần quay/cua kế tiếp: đoạn không tường giữ hướng bằng gyro
  _absHeading = 0.0f;   // lùi kịch tường: xe vuông góc tường -> hướng chuẩn tuyệt đối về 0
  _turnHeadingValid = true;
  navPrintf(">> [CALIBCELL] %s, YAW = %.2f | tien %ld xung toi giua o\n", doCalib ? "GYRO CALIB XONG" : "RESET YAW (khong calib)",
            mpu6050.getYaw(), calibFwdPulses);
  if (noForward) return true;  // speedrun: đoạn thẳng kế tiếp chạy luôn từ vách, cộng thêm calibFwdPulses -> không dừng ở tâm ô

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

  while (millis() - t0 < frontAlignMaxMs && !i2cFault) {
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
