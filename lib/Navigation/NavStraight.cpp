// ĐI THẲNG: startPID / stepCell / PID loop (Encoder + Gyro + Wall) / giữ ô + log chạy
#include "RobotNav.h"
#include "NavInternal.h"

void RobotNav::startPID() {
  if (mpuReady) {
    mpu6050.setAutoBias(false); // đang chạy: không để tự bù trôi "nuốt" chuyển động quay thật của xe
    mpu6050.update();
    _targetYaw = mpu6050.getYaw();
  }
  _startEncLeft = getLeftEncoder();
  _startEncRight = getRightEncoder();
  _lastEncLeft = _startEncLeft;
  _lastEncRight = _startEncRight;
  _lastPIDLoopTime = micros();
  // Đang chạy sẵn (vd. vừa cua xong) thì tăng tốc tiếp từ tốc độ hiện tại, đứng yên thì từ accelStartPwm
  _rampFrom = max(accelStartPwm, (currentLeftSpeed + currentRightSpeed) * 0.5f);
  _rampT0 = millis();
  _yawIUs = 0;
  if (currentLeftSpeed == 0 && currentRightSpeed == 0) _yawI = 0.0f;  // xuất phát từ đứng yên: xoá I; đang chạy liên tục: giữ I
  pidRunActive = true;
}

void RobotNav::stopPID() {
  _carryValid = false;
  stepCarry = 0;
  if (mpuReady) mpu6050.setAutoBias(true);
  holdRun = false;
  pidRunActive = false;
  stepCellActive = false;
  autoWallFollowActive = false;
  stopMotors();
}

void RobotNav::stepCell(int numCells) {
  if (numCells <= 0) return;
  startPID(); // Kích hoạt PID bám tường và giữ hướng thẳng
  stepCellActive = true;
  stepStartTime = millis();
  stepStartL = getLeftEncoder();
  stepStartR = getRightEncoder();
  stepStartPulses = (stepStartL + stepStartR) / 2;
  stepTargetPulses = (long)numCells * pulsesPerCell;
  stepTraveledPulses = 0;
  sideNearL = sideFarL = sideNearR = sideFarR = 0;
  frontNearN = frontFarN = 0;
  _sideLastTF = _tF;
  _sideLastTL = _tL;
  _sideLastTR = _tR;

  String msg = ">> [CELL] BAT DAU TIEN " + String(numCells) + " O (" +
               String(stepTargetPulses) + " xung)...";
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::updatePIDLoop() {
  if (!pidRunActive)
    return;

  _lastPIDLoopTime = micros();

  uint16_t dF = (uint16_t)smoothDF;

  updateWallFilter();
  logSample();

  // Chế độ chạy liên tục: gặp tường trước (<= frontStopDist) thì đứng yên, PID vẫn chạy; tường xa ra > +8mm thì đi tiếp
  if (holdRun) {
    if (hold.useFront && frontReady && dF > 20 && dF <= frontStopDist) {
      if (!_holdStopped) brakeMotors(); // hãm chủ động, tránh trôi quán tính qua tường
      _holdStopped = true;
      _holdFreeSince = 0;
    } else if (_holdStopped) {
      // Chỉ đi tiếp khi tường trước xa ra liên tục >= 500ms (chống gai đọc 999 làm xe lao vào tường)
      if (dF > frontStopDist + 8) {
        if (_holdFreeSince == 0) _holdFreeSince = millis();
        if (millis() - _holdFreeSince >= 500) _holdStopped = false;
      } else {
        _holdFreeSince = 0;
      }
    }
    if (_holdStopped) {
      stopMotors();
      if (mpuReady) _targetYaw = mpu6050.getYaw();
      return;
    }
  }

  // 1. Phanh dừng an toàn khi gặp vách tường trước
  if (!holdRun && frontReady && dF > 20 && dF <= frontStopDist) {
    brakeMotors();
    stopPID();
    String msg =
        ">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)";
    Serial.println(msg);
    serialLink.println(msg);
    return;
  }

  // 2. Kiểm tra cự ly chạy theo số ô Encoder (Cell Stepping)
  if (stepCellActive) {
    long curL = getLeftEncoder();
    long curR = getRightEncoder();
    long distL = labs(curL - stepStartL);
    long distR = labs(curR - stepStartR);
    long distTraveled = (distL + distR) / 2 + stepCarry;  // stepCarry: quãng vượt của ô trước khi chạy liên tục
    stepTraveledPulses = distTraveled;

    // Đếm mẫu vách hông trong cửa sổ của ô cuối (-3 = ngoài tầm = thông; -1/-2 = lỗi, bỏ qua)
    long lastCellStart = stepTargetPulses - pulsesPerCell;
    float frac = (float)(distTraveled - lastCellStart) / (float)pulsesPerCell;
    if (frac >= sideWinFrom && frac <= sideWinTo) {
      if (_tL != _sideLastTL) {
        _sideLastTL = _tL;
        if (_rawL >= 0 && _rawL < navSideWallMm) { if (sideNearL < 255) sideNearL++; }
        else if ((_rawL >= 0 || _rawL == -3) && sideFarL < 255) sideFarL++;
      }
      if (_tR != _sideLastTR) {
        _sideLastTR = _tR;
        if (_rawR >= 0 && _rawR < navSideWallMm) { if (sideNearR < 255) sideNearR++; }
        else if ((_rawR >= 0 || _rawR == -3) && sideFarR < 255) sideFarR++;
      }
    }
    // Tường TRƯỚC của ô đang đi vào: xe ở vị trí frac thì tường đó cách xa hơn lúc đứng tâm ô (1-frac) ô
    // -> ngưỡng = navFrontWallMm + (1-frac)*cellMm. Tường của ô sau nữa xa thêm 1 ô nên vẫn nằm ngoài ngưỡng
    if (frac >= frontWinFrom && frac <= frontWinTo && _tF != _sideLastTF) {
      _sideLastTF = _tF;
      float thr = navFrontWallMm + (1.0f - frac) * cellMm * frontMoveGain;
      if (_rawF >= 0 && _rawF < thr) { if (frontNearN < 255) frontNearN++; }
      else if ((_rawF >= 0 || _rawF == -3) && frontFarN < 255) frontFarN++;
    }

    // Đạt điều kiện dừng khi:
    // a) Quãng đường trung bình 2 bánh đạt đủ targetPulses (loại bỏ trường hợp 1 bánh quay nhanh khi bẻ lái làm dừng sớm)
    // b) HOẶC timeout an toàn 4.5 giây phòng ngừa xe chạy vô tận
    // Chạy liên tục (stepNoStop): KHÔNG phanh sớm, đi đủ quãng (phần vượt chuyển sang ô sau qua stepCarry)
    // Quãng trôi sau phanh tăng theo bình phương tốc độ: stopLeadPulses đo ở stopLeadRefPwm, chạy nhanh hơn thì phanh sớm hơn
    float sk = (float)baseForwardSpeed / stopLeadRefPwm;
    long lead = stepNoStop ? 0 : (long)(stopLeadPulses * sk * sk);
    bool pulseReached = (distTraveled >= stepTargetPulses - lead); // phanh sớm stopLeadPulses để bù quãng trôi sau phanh
    bool timeoutSafe = (millis() - stepStartTime > 4500UL * (unsigned long)max(1L, stepTargetPulses / pulsesPerCell));

    if ((pulseReached || timeoutSafe) && stepNoStop) {
      _carryValid = pulseReached;  // ô sau đếm tiếp phần đã vượt (kể cả quãng chạy trong lúc chuyển ô)
      // Hết quãng đường nhưng giữ nguyên PWM hiện tại để vào corner đang chạy (không phanh, không stopMotors)
      stepNoStop = false;
      stepCellActive = false;
      pidRunActive = false;
      holdRun = false;
      return;
    }
    if (pulseReached || timeoutSafe) {
      bool quiet = holdRun; // chế độ test: dừng xong không in gì, log in riêng
      brakeMotors();
      stopPID();
      if (quiet) return;
      String msg = ">> [CELL] DA HOAN THANH TIEN O! Xung: " + String(distTraveled) +
                   "/" + String(stepTargetPulses) + " xung (L:" + String(distL) +
                   ", R:" + String(distR) + "). Phanh dung.";
      if (timeoutSafe && !pulseReached) {
        msg = ">> [CELL] TIMEOUT AN TOAN 4.5S! Xung: " + String(distTraveled) +
              "/" + String(stepTargetPulses) + " xung. Phanh dung.";
      }
      Serial.println(msg);
      serialLink.println(msg);
      return;
    }
  }

  long curL = getLeftEncoder();
  long curR = getRightEncoder();
  _lastEncLeft = curL;
  _lastEncRight = curR;

  float pidOut = 0.0f;
  float yawCorr = 0.0f;
  float wallErr = 0.0f, wallCorr = 0.0f;

  {
    // ĐƯỜNG PID DUY NHẤT: Encoder P + giữ hướng gyro + Wall P (VL53 trái/phải)
    // Encoder: sai số = chênh quãng đường TÍCH LŨY 2 bánh từ lúc xuất phát
    // (|L| - |R| > 0: bánh trái đi trước -> giảm bánh trái, tăng bánh phải). P trên vị trí nên không bị mất do làm tròn
    float travL = (float)labs(curL - _startEncLeft);
    float travR = (float)labs(curR - _startEncRight);
    float encCorr = constrain(encKp * (travL - travR), -encMaxCorr, encMaxCorr);
    pidOut = encCorr;
    _lastCorr = encCorr;
    // Wall: sai lệch ngang theo baseline từng bên, không dùng trực tiếp L-R.
    // eL = L - baseL > 0: xa tường trái hơn mức giữa (xe lệch phải); eR = R - baseR > 0: xa tường phải (xe lệch trái)
    // wallErr > 0 -> cần quay TRÁI
    wallErr = 0.0f;
    wallCorr = 0.0f;
    if (hold.wallEnable) {
      bool okL = leftReady && _wFiltL > 20.0f && _wFiltL < hold.wallValidMax;
      bool okR = rightReady && _wFiltR > 20.0f && _wFiltR < hold.wallValidMax;
      float eL = _wFiltL - hold.wallBaseL;
      float eR = _wFiltR - hold.wallBaseR;
      if (okL && okR) wallErr = 0.5f * (eL - eR);
      else if (okL)   wallErr = eL;
      else if (okR)   wallErr = -eR;
      wallErr = constrain(wallErr, -hold.wallErrClamp, hold.wallErrClamp);
    }
    bool useGyro = mpuReady && hold.yawKp > 0.0f;
    // Wall -> DỊCH HƯỚNG ĐÍCH của gyro (độ) thay vì cộng PWM: gyro không còn chống lại việc lái về giữa ô,
    // xe đi chéo nhẹ vào tâm rồi thẳng lại. Không có gyro / wallYawGain = 0: cộng PWM như cũ.
    float wallYawOff = 0.0f;
    if (hold.wallEnable && useGyro && hold.wallYawGain > 0.0f) {
      wallYawOff = constrain(hold.wallYawGain * wallErr, -hold.wallYawMax, hold.wallYawMax);
      wallCorr = wallYawOff;  // log: độ dịch hướng
    } else if (hold.wallEnable) {
      wallCorr = constrain(hold.wallKp * wallErr, -hold.wallMaxCorr, hold.wallMaxCorr);
      pidOut += wallCorr;
    }
    // Giữ hướng bằng gyro. Xe lệch PHẢI thì YAW TĂNG; pidOut > 0 => xe quay TRÁI, nên yawCorr = -Kp * yawError.
    // Quay trái làm yaw đổi theo turnLeftYawSign -> muốn lái trái wallYawOff độ thì đích = target + sign * off
    yawCorr = 0.0f;
    if (useGyro) {
      float e = wrap180(_targetYaw + turnLeftYawSign * wallYawOff - mpu6050.getYaw());
      _lastYawErr = e;
      // Khâu I: 2 motor lệch lực theo tốc độ (chạy nhanh lâu thì P không đủ, xe vẫn trôi dần sang 1 bên)
      unsigned long nowUs = micros();
      float dt = _yawIUs ? (nowUs - _yawIUs) * 1e-6f : 0.0f;
      _yawIUs = nowUs;
      if (hold.yawKi > 0.0f && dt > 0.0f && dt < 0.1f) {
        _yawI += e * dt;
        float lim = hold.yawIMax / hold.yawKi;  // kẹp để phần I không vượt yawIMax PWM
        _yawI = constrain(_yawI, -lim, lim);
      }
      yawCorr = constrain(-(hold.yawKp * e + hold.yawKi * _yawI), -yawMaxCorr, yawMaxCorr);
      pidOut += yawCorr;
    }
  }

  // 4. Xuất PWM cho 2 bánh có giảm tốc 2 giai đoạn (Profile Deceleration):
  // - 0% -> 70% quãng đường ô: chạy tốc độ baseForwardSpeed tiêu chuẩn
  // - 70% -> 100% quãng đường ô: hãm về tốc độ bò (crawl speed ~46 PWM) để khi ngắt động cơ xe dừng dứt điểm, không trượt quán tính
  int activeBaseSpeed = baseForwardSpeed;
  // Tăng tốc dần: từ _rampFrom lên baseForwardSpeed trong accelRampMs (tránh bốc đầu khi xuất phát từ đứng yên)
  if (accelRampMs > 0) {
    float t = (millis() - _rampT0) / (float)accelRampMs;
    if (t < 1.0f) activeBaseSpeed = (int)lroundf(_rampFrom + (baseForwardSpeed - _rampFrom) * t);
  }
  if (!holdRun && stepCellActive && !stepNoStop && stepTraveledPulses >= (long)(0.70f * stepTargetPulses)) {
    activeBaseSpeed = 46;
  }

  _lastYawCorr = yawCorr;
  _lastWallErr = wallErr;
  _lastWallCorr = wallCorr;
  int leftSpeed = activeBaseSpeed - (int)lroundf(pidOut);
  int rightSpeed = activeBaseSpeed + (int)lroundf(pidOut);

  leftSpeed = constrain((int)(leftSpeed * motorScaleL), 35, 255);
  rightSpeed = constrain((int)(rightSpeed * motorScaleR), 35, 255);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  analogWrite(M1_IN1, leftSpeed);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, rightSpeed);
  analogWrite(M2_IN2, 0);
}

// ================= GIỮ VỊ TRÍ TRONG Ô =================

void RobotNav::startHold(bool keepLog) {
  // Chạy thẳng bằng đường PID duy nhất của updatePIDLoop: Encoder P + giữ hướng gyro + Wall P (VL53 trái/phải)
  stepCellActive = false;
  autoWallFollowActive = false;
  holdRun = true;
  _holdStopped = false;
  if (!keepLog) clearLog();
  // Ô trước vừa chạy liên tục qua (không phanh): phần xung đã đi quá đích được tính vào ô này, không bị mất khi reset encoder
  long carry = 0;
  if (_carryValid) {
    long travel = (labs(getLeftEncoder() - stepStartL) + labs(getRightEncoder() - stepStartR)) / 2 + stepCarry;
    carry = constrain(travel - stepTargetPulses, 0L, pulsesPerCell / 2);
  }
  _carryValid = false;
  resetEnc();
  frontStopDist = (uint16_t)hold.targetF;
  stepCell(hold.cells);
  stepCarry = carry;
}

void RobotNav::startCells(int cells, bool noStop) {
  if (cells <= 0) return;
  if (mpuReady && !_turnHeadingValid) { mpu6050.update(); _turnHeading = mpu6050.getYaw(); _turnHeadingValid = true; }
  uint8_t savedCells = hold.cells;
  hold.cells = cells;
  startHold(true);
  hold.cells = savedCells;
  if (_turnHeadingValid) _targetYaw = _turnHeading; // bám hướng chuẩn: lỗi quay/cua trước không cộng dồn
  stepNoStop = noStop;
}

void RobotNav::driveCells(int cells, bool noStop) {
  if (cells <= 0) return;
  startCells(cells, noStop);
  unsigned long t0 = millis();
  while (pidRunActive && millis() - t0 < 15000UL * (unsigned long)cells) update();
  if (pidRunActive) stopPID();
  stepNoStop = false;
}

WallState RobotNav::movingSideWall(bool left) const {
  uint8_t nearN = left ? sideNearL : sideNearR;
  uint8_t farN = left ? sideFarL : sideFarR;
  if (nearN + farN < 3) return WS_UNKNOWN;           // quá ít mẫu
  if (nearN * 4 >= (nearN + farN) * 3) return WS_WALL;  // >= 75% mẫu gần
  if (farN * 4 >= (nearN + farN) * 3) return WS_OPEN;   // >= 75% mẫu xa
  return WS_UNKNOWN;                                  // lẫn lộn (thường do chạm cột góc)
}

WallState RobotNav::movingFrontWall() const {
  uint8_t n = frontNearN + frontFarN;
  if (n < 3) return WS_UNKNOWN;
  if (frontNearN * 2 >= n) return WS_WALL;            // >= 50% mẫu gần: coi là có tường (an toàn, tránh lao vào tường)
  if (frontFarN * 4 >= n * 3) return WS_OPEN;         // >= 75% mẫu xa mới coi là thông
  return WS_UNKNOWN;
}

void RobotNav::settle(unsigned long ms) {
  unsigned long s = millis();
  while (millis() - s < ms) { if (mpuReady) mpu6050.update(); delay(2); }
}

// ================= LOG CHẠY =================

void RobotNav::logSample() {
  unsigned long now = millis();
  if (now - _logLastT < 33) return; // ~30Hz
  float dt = (now - _logLastT) / 1000.0f;
  _logLastT = now;
  long l = getLeftEncoder(), r = getRightEncoder();
  LogRec &e = _log[_logHead];
  e.t = now;
  e.eL = l;
  e.eR = r;
  e.dL = _rawL; // VL53 thô (mm), -1 = lỗi/ngoài tầm
  e.dF = _rawF;
  e.dR = _rawR;
  e.vL = (_logCount == 0 || dt > 1.0f) ? 0 : (int16_t)((l - _logLastL) / dt);
  e.vR = (_logCount == 0 || dt > 1.0f) ? 0 : (int16_t)((r - _logLastR) / dt);
  e.pL = currentLeftSpeed;
  e.pR = currentRightSpeed;
  e.c = (int16_t)lroundf(_lastCorr);
  e.yaw = mpuReady ? mpu6050.getYaw() : 0.0f;
  e.yawT = _targetYaw;
  e.yawE = _lastYawErr;
  e.yawC = _lastYawCorr;
  e.wallE = _lastWallErr;
  e.wallC = _lastWallCorr;
  e.fL = (int16_t)min(_wFiltL, 999.0f);
  e.fR = (int16_t)min(_wFiltR, 999.0f);
  _logLastL = l;
  _logLastR = r;
  _logHead = (_logHead + 1) % LOG_SIZE;
  if (_logCount < LOG_SIZE) _logCount++;
}

void RobotNav::printLog() {
  Serial.println("==== LOG: TIME,ENC_L,ENC_R,YAW,VL53_L,VL53_F,VL53_R,VL53_L_MED,VL53_R_MED,WALL_ERROR,WALL_CORRECTION,SPEED_L,SPEED_R,ERROR,PID_CORRECTION,YAW_TARGET,YAW_ERROR,YAW_CORRECTION,PWM_L,PWM_R ====");
  uint16_t start = (_logCount < LOG_SIZE) ? 0 : _logHead;
  for (uint16_t i = 0; i < _logCount; i++) {
    const LogRec &e = _log[(start + i) % LOG_SIZE];
    Serial.printf("%lu,%ld,%ld,%.2f,%d,%d,%d,%d,%d,%.1f,%.2f,%d,%d,%ld,%d,%.2f,%.2f,%.2f,%d,%d\n", (unsigned long)e.t, (long)e.eL, (long)e.eR, e.yaw, e.dL, e.dF, e.dR, e.fL, e.fR, e.wallE, e.wallC, e.vL, e.vR, (long)(e.eL - e.eR), e.c, e.yawT, e.yawE, e.yawC, e.pL, e.pR);
  }
  Serial.printf("==== HET LOG (%u mau) ====\n", _logCount);
}
