// CÁC BÀI TEST (SEQ_MODE 1..6 trong MainParams.h)
#include "RobotNav.h"
#include "NavInternal.h"

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

  move(true);
  if (startCornerLeft()) {
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

// ================= TEST CELL-TO-CELL: 1 ô -> dừng -> trái 90 -> 1 ô -> dừng -> phải 90 -> 1 ô -> dừng =================

void RobotNav::runCellToCellTest() {
  if (!mpuReady) { Serial.println(">> [C2C] MPU6050 chua san sang, huy"); return; }
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
  _turnHeading = mpu6050.getYaw(); // hướng chuẩn: mỗi đoạn thẳng bám hướng này, lỗi quay không cộng dồn
  _turnHeadingValid = true;
  clearLog();
  Serial.printf(">> [C2C] BAT DAU | %u xung/o | yaw0=%.2f\n", (unsigned)seqCellPulses, _turnHeading);

  int step = 0;
  auto move = [&]() {
    step++;
    Serial.printf(">> [C2C] %d/5 MOVE 1 O\n", step);
    startHold(true);
    _targetYaw = _turnHeading;
    unsigned long t0 = millis();
    while (pidRunActive && millis() - t0 < 15000) update();
    if (pidRunActive) stopPID();
    settle(300); // dừng hẳn trước khi quay
    Serial.printf(">>        MOVE xong: %ld xung | yaw=%.2f (chuan %.2f)\n", stepTraveledPulses, mpu6050.getYaw(), _turnHeading);
  };
  auto turn = [&](float angle, const char *name) {
    step++;
    Serial.printf(">> [C2C] %d/5 TURN %s\n", step, name);
    bool ok = turnLeft90Test(angle); // góc dương = trái, âm = phải
    settle(200);
    Serial.printf(">>        TURN xong (%s) | yaw=%.2f (chuan %.2f)\n", ok ? "OK" : "TIMEOUT", mpu6050.getYaw(), _turnHeading);
  };

  move();
  turn(90.0f, "TRAI 90");
  move();
  turn(-90.0f, "PHAI 90");
  move();

  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  Serial.println(">> [C2C] DONE - PWM = 0");
}

// ================= TEST NGÕ CỤT: 1 ô -> dừng -> quay 180° -> calib lại ô =================

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

// ================= ĐỌC TƯỜNG AN TOÀN + TỰ ĐI TỪNG Ô (SEQ_MODE 6) =================

// Phân loại 1 cảm biến từ các mẫu MỚI thu được khi xe đứng yên. Chỉ tin khi đủ mẫu và đa số cùng một kết luận.
// Cảm biến chưa init được, không đủ mẫu mới (timeout) hoặc mẫu mâu thuẫn -> WS_UNKNOWN, KHÔNG coi là "không có tường".
static WallState classifyWall(const int16_t *s, uint8_t n, bool ready, int thr) {
  if (!ready || n < 3) return WS_UNKNOWN;
  uint8_t closeN = 0, farN = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (s[i] == -3 || s[i] >= thr) farN++;  // -3 = ngoài tầm đo (> 1200 mm) = không có vật gần
    else closeN++;
  }
  if (closeN * 2 > n) return WS_WALL;
  if (farN * 2 > n) return WS_OPEN;
  return WS_UNKNOWN;
}

// In ra Serial và lưu vào navReport (giới hạn kích thước) để xem lại khi cắm USB sau lúc chạy rời
void RobotNav::navPrintf(const char *fmt, ...) {
  char buf[220];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.print(buf);
  if (navReport.length() < 8000) navReport += buf;
}

// Trung vị n mẫu (n >= 1) để in ra cho dễ chỉnh ngưỡng; không dùng để quyết định
static int medianOf(const int16_t *s, uint8_t n) {
  if (n == 0) return -9999;
  int16_t t[8];
  memcpy(t, s, n * sizeof(int16_t));
  for (uint8_t i = 0; i + 1 < n; i++)
    for (uint8_t j = i + 1; j < n; j++)
      if (t[j] < t[i]) { int16_t x = t[i]; t[i] = t[j]; t[j] = x; }
  return t[n / 2];
}

static const char *wallStr(WallState w) {
  return w == WS_WALL ? "TUONG" : (w == WS_OPEN ? "THONG" : "LOI?");
}

void RobotNav::senseWallsSafe(WallState &l, WallState &f, WallState &r) {
  int16_t sL[8], sF[8], sR[8];
  uint8_t nL = 0, nF = 0, nR = 0;
  unsigned long lastL = _tL, lastF = _tF, lastR = _tR;
  unsigned long t0 = millis();
  while (millis() - t0 < 250) {
    updateSensors();
    if (mpuReady) mpu6050.update();
    // Chỉ lấy mẫu mới thật sự (mốc thời gian đổi); -1 = init lỗi, -2 = timeout không bao giờ được thêm vào
    if (_tL != lastL) { lastL = _tL; if (_rawL != -1 && _rawL != -2 && nL < 8) sL[nL++] = _rawL; }
    if (_tF != lastF) { lastF = _tF; if (_rawF != -1 && _rawF != -2 && nF < 8) sF[nF++] = _rawF; }
    if (_tR != lastR) { lastR = _tR; if (_rawR != -1 && _rawR != -2 && nR < 8) sR[nR++] = _rawR; }
    delay(2);
  }
  l = classifyWall(sL, nL, leftReady, navSideWallMm);
  f = classifyWall(sF, nF, frontReady, navFrontWallMm);
  r = classifyWall(sR, nR, rightReady, navSideWallMm);
  navPrintf(">> [NAV] L=%s(%u mau) F=%s(%u mau) R=%s(%u mau) | trung vi L/F/R = %d/%d/%d mm\n",
                wallStr(l), nL, wallStr(f), nF, wallStr(r), nR, medianOf(sL, nL), medianOf(sF, nF), medianOf(sR, nR));
}

void RobotNav::runCellNavTest() {
  if (!mpuReady) { navPrintf(">> [NAV] MPU6050 chua san sang, huy\n"); return; }
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
  navReport = "";
  navReportPending = false;
  navPrintf(">> [NAV] BAT DAU | %u xung/o | tuong ben < %u mm, tuong truoc < %u mm | chay lien tuc toi da %u o roi dung\n",
                (unsigned)seqCellPulses, (unsigned)navSideWallMm, (unsigned)navFrontWallMm, (unsigned)navMaxSteps);

  auto moveOne = [&]() {
    startHold(true);
    _targetYaw = _turnHeading;
    unsigned long t0 = millis();
    while (pidRunActive && millis() - t0 < 15000) update();
    if (pidRunActive) stopPID();
    settle(300); // dừng hẳn trước khi đọc tường / quay
    navPrintf(">>        MOVE xong: %ld xung | yaw=%.2f\n", stepTraveledPulses, mpu6050.getYaw());
  };
  auto turnDeg = [&](float angle, const char *name) -> bool { // dương = trái, âm = phải
    bool ok = turnLeft90Test(angle);
    settle(300);
    navPrintf(">>        QUAY %s xong (%s) | yaw=%.2f (chuan %.2f)\n", name, ok ? "OK" : "TIMEOUT", mpu6050.getYaw(), _turnHeading);
    return ok;
  };

  // Cảm biến TRÁI/PHẢI chéo nhìn trước 1 ô: đứng ở ô N thì chúng đọc vách hông của ô N+1.
  // Cảm biến TRƯỚC đọc vách trước của CHÍNH ô N. Nên vách hông của ô hiện tại (curL/curR) phải lấy từ lần đọc ở ô trước (nhớ lại).
  // Ô xuất phát chưa có lần đọc trước -> theo quy ước ô xuất phát có tường 2 bên (NAV_START_SIDES_WALL), hoặc "không biết".
  WallState curL = navStartSidesWall ? WS_WALL : WS_UNKNOWN;
  WallState curR = curL;

  uint16_t cellsMoved = 0;
  const uint16_t maxLoops = navMaxSteps * 4 + 8; // chặn vòng lặp vô hạn (quay tại chỗ không tính là 1 ô)
  for (uint16_t step = 1; step <= maxLoops && cellsMoved < navMaxSteps; step++) {
    WallState nxtL, wf, nxtR; // nxtL/nxtR: vách hông của ô KẾ TIẾP (nhìn trước); wf: vách trước của ô hiện tại
    navPrintf(">> [NAV] BUOC %u\n", (unsigned)step);
    senseWallsSafe(nxtL, wf, nxtR);
    navPrintf(">> [NAV] O HIEN TAI: truoc=%s trai=%s(nho) phai=%s(nho) | O KE TIEP: trai=%s phai=%s\n",
              wallStr(wf), wallStr(curL), wallStr(curR), wallStr(nxtL), wallStr(nxtR));

    if (wf == WS_UNKNOWN) { navPrintf(">> [NAV] VL53 TRUOC KHONG DANG TIN -> DUNG\n"); break; }

    if (wf == WS_OPEN) {                         // ô hiện tại không có tường trước: đi thẳng 1 ô
      navPrintf(">> [NAV] -> DI THANG 1 O (o thu %u)\n", (unsigned)(cellsMoved + 1));
      moveOne();
      cellsMoved++;
      curL = nxtL; // xe vào ô kế tiếp: vách hông của nó chính là số đọc nhìn-trước vừa rồi
      curR = nxtR;
    } else if (curL == WS_OPEN) {                // trước có tường, hông trái CỦA Ô HIỆN TẠI thông: dừng hẳn rồi quay trái 90°
      navPrintf(">> [NAV] -> TRUOC CO TUONG, RE TRAI 90\n");
      if (!turnDeg(90.0f, "TRAI 90")) { navPrintf(">> [NAV] QUAY TIMEOUT -> DUNG\n"); break; }
      curL = WS_UNKNOWN; // hông trái mới = phía sau cũ (không biết chắc)
      curR = WS_WALL;    // hông phải mới = phía trước cũ (vừa xác nhận có tường)
    } else if (curR == WS_OPEN) {                // trái không thông, phải thông: quay phải 90°
      navPrintf(">> [NAV] -> TRUOC CO TUONG, RE PHAI 90\n");
      if (!turnDeg(-90.0f, "PHAI 90")) { navPrintf(">> [NAV] QUAY TIMEOUT -> DUNG\n"); break; }
      curL = WS_WALL;
      curR = WS_UNKNOWN;
    } else if (curL == WS_WALL && curR == WS_WALL) { // cả 3 phía của ô hiện tại có tường: ngõ cụt -> quay 180° + calib lại tâm ô
      navPrintf(">> [NAV] -> NGO CUT, QUAY DAU 180 + CALIB O\n");
      if (!turnDeg(180.0f, "180")) { navPrintf(">> [NAV] QUAY TIMEOUT -> DUNG\n"); break; }
      if (!calibCellFromBackWall()) { navPrintf(">> [NAV] CALIB O LOI -> DUNG\n"); break; }
      curL = WS_WALL; // hông cũ trái/phải giờ đổi bên, vẫn là tường cả hai
      curR = WS_WALL;
    } else {                                     // hông của ô hiện tại chưa biết chắc: không đoán
      navPrintf(">> [NAV] VACH HONG O HIEN TAI KHONG XAC DINH -> KHONG QUYET DINH RE, DUNG\n");
      break;
    }
  }

  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
  navPrintf(">> [NAV] DONE - PWM = 0 | da di %u o\n", (unsigned)cellsMoved);
  navReportPending = !(bool)Serial; // chạy rời USB: lưu nhật ký, main in ra khi cắm USB
}

// ================= TEST ĐỌC 3 VL53L0X: ghi 100 mẫu vào RAM, in khi cắm USB. KHÔNG chạy động cơ =================
// Dùng lại pollSensor() có sẵn (đọc không chặn), không lọc, không hiệu chuẩn. Mục tiêu 30Hz (33.3ms/mẫu).
// Trong lúc ghi không in gì. Đo thực tế: thời gian đọc I2C 3 cảm biến và khoảng giữa 2 mẫu MỚI của từng cảm biến.

static const char *vlStatus(int16_t raw) {
  if (raw >= 0) return "OK";
  if (raw == -1) return "INIT_FAIL";
  if (raw == -2) return "TIMEOUT";
  return "OUT_RANGE";
}

void RobotNav::runSensorStreamTest() {
  const unsigned long PERIOD_US = 33333; // 30 Hz
  unsigned long lastNew[3] = {_tL, _tF, _tR};
  unsigned long ivN[3] = {0, 0, 0}, ivSum[3] = {0, 0, 0}, ivMin[3] = {0xFFFFFFFFUL, 0xFFFFFFFFUL, 0xFFFFFFFFUL}, ivMax[3] = {0, 0, 0};
  unsigned long puSum[3] = {0, 0, 0}, totSum = 0, totMax = 0, pollN = 0;

  _sensorLogN = 0;
  unsigned long firstUs = micros(), nextUs = firstUs;
  unsigned long lastRecUs = 0, recSum = 0, recMax = 0;

  while (_sensorLogN < SENSOR_LOG_SIZE) {
    unsigned long now = millis();
    unsigned long u0 = micros();
    pollSensor(sensorLeft, leftReady, _rawL, _tL, now);
    unsigned long u1 = micros();
    pollSensor(sensorFront, frontReady, _rawF, _tF, now);
    unsigned long u2 = micros();
    pollSensor(sensorRight, rightReady, _rawR, _tR, now);
    unsigned long u3 = micros();
    puSum[0] += u1 - u0; puSum[1] += u2 - u1; puSum[2] += u3 - u2;
    unsigned long tot = u3 - u0;
    totSum += tot;
    if (tot > totMax) totMax = tot;
    pollN++;

    unsigned long tn[3] = {_tL, _tF, _tR};
    for (int i = 0; i < 3; i++) {
      if (tn[i] != lastNew[i]) {
        if (lastNew[i] != 0) { // bỏ mẫu đầu tiên (chưa có mốc trước)
          unsigned long iv = tn[i] - lastNew[i];
          ivN[i]++; ivSum[i] += iv;
          if (iv < ivMin[i]) ivMin[i] = iv;
          if (iv > ivMax[i]) ivMax[i] = iv;
        }
        lastNew[i] = tn[i];
      }
    }

    if ((long)(micros() - nextUs) >= 0) {
      unsigned long nu = micros();
      if (lastRecUs != 0) { unsigned long p = nu - lastRecUs; recSum += p; if (p > recMax) recMax = p; }
      lastRecUs = nu;
      SensorRec &r = _sensorLog[_sensorLogN++];
      r.t = (uint32_t)now;
      r.l = _rawL; r.f = _rawF; r.r = _rawR;
      nextUs += PERIOD_US;
    }
    delay(1); // nhường I2C giữa các lần hỏi trạng thái
  }

  // Tổng hợp thống kê thời gian đo thực tế
  const char *nm[3] = {"L", "F", "R"};
  float worstMs = 0.0f;
  sensorReport = "==== SENSOR STAT (" + String(_sensorLogN) + " mau, muc tieu 33.3 ms) ====\n";
  sensorReport += "doc I2C avg L/F/R = " + String(puSum[0] / pollN) + "/" + String(puSum[1] / pollN) + "/" + String(puSum[2] / pollN) +
                  " us | tong avg = " + String(totSum / pollN) + " us, max = " + String(totMax) + " us\n";
  for (int i = 0; i < 3; i++) {
    if (ivN[i] == 0) { sensorReport += String(nm[i]) + ": khong co mau moi\n"; worstMs = 9999.0f; continue; }
    float avg = (float)ivSum[i] / (float)ivN[i];
    if (avg > worstMs) worstMs = avg;
    sensorReport += String(nm[i]) + ": mau moi moi " + String(avg, 1) + " ms (min " + String(ivMin[i]) + ", max " + String(ivMax[i]) +
                    ") = " + String(1000.0f / avg, 1) + " Hz | " + String(ivN[i]) + " mau\n";
  }
  if (_sensorLogN > 1)
    sensorReport += "chu ky ghi mau thuc te: avg " + String((float)recSum / (float)(_sensorLogN - 1) / 1000.0f, 1) +
                    " ms, max " + String((float)recMax / 1000.0f, 1) + " ms\n";
  if (totSum / pollN > PERIOD_US)
    sensorReport += "CANH BAO: doc 3 cam bien mat " + String(totSum / pollN) + " us > 33333 us -> khong the dat 30 Hz\n";
  if (worstMs > 33.4f)
    sensorReport += "GIOI HAN THUC TE: cam bien chi co mau moi moi " + String(worstMs, 1) + " ms => toi da ~" +
                    String(1000.0f / worstMs, 1) + " Hz (< 30 Hz), mot so dong ben duoi lap lai gia tri cu\n";
  else
    sensorReport += "Toc do mau moi cua ca 3 cam bien >= 30 Hz\n";
  _sensorLogPrinted = false;
}

void RobotNav::printSensorLog() {
  Serial.println(sensorReport);
  Serial.println("==== LOG: TIME_MS,L_MM,F_MM,R_MM,L_ST,F_ST,R_ST (mm = -1 khi loi) ====");
  for (uint16_t i = 0; i < _sensorLogN; i++) {
    const SensorRec &r = _sensorLog[i];
    Serial.printf("%lu,%d,%d,%d,%s,%s,%s\n", (unsigned long)r.t, r.l >= 0 ? r.l : -1, r.f >= 0 ? r.f : -1, r.r >= 0 ? r.r : -1,
                  vlStatus(r.l), vlStatus(r.f), vlStatus(r.r));
  }
  Serial.printf("==== HET LOG (%u mau) ====\n", _sensorLogN);
  _sensorLogPrinted = true;
}

// ================= TEST CALIBRATION ENCODER (chạy thẳng N ô liên tục) =================

void RobotNav::runCalibTest(uint8_t cells) {
  if (cells == 0) cells = 1;
  stopPID();
  unsigned long s = millis();
  if (mpuReady) { while (millis() - s < 200) { mpu6050.update(); delay(2); } }

  uint8_t savedCells = hold.cells;
  long savedPulses = pulsesPerCell;
  hold.cells = cells;
  pulsesPerCell = seqCellPulses;  // 4200 xung/ô: giá trị TẠM để test, không phải calibration chính thức
  long target = (long)cells * pulsesPerCell;
  Serial.printf(">> [CALIB] BAT DAU: %u o x %ld xung/o (tam) = %ld xung\n", (unsigned)cells, pulsesPerCell, target);

  startHold();  // Encoder PID + yaw correction + VL53 wall correction như đường chạy thẳng hiện tại
  unsigned long t0 = millis();
  while (pidRunActive && millis() - t0 < 30000) update();
  if (pidRunActive) stopPID();
  stopMotors();
  currentLeftSpeed = currentRightSpeed = 0;

  // Chờ xe trôi hết quán tính rồi mới đọc encoder
  s = millis();
  while (millis() - s < 500) { if (mpuReady) mpu6050.update(); delay(2); }

  long l = getLeftEncoder(), r = getRightEncoder();
  float avg = (labs(l) + labs(r)) / 2.0f;
  float perCell = avg / (float)cells;
  calibReport = "==== CALIB " + String((unsigned)cells) + " O (target " + String(target) + " xung) ====\n" +
                "ENC_L = " + String(l) + "\n" +
                "ENC_R = " + String(r) + "\n" +
                "AVG_TICKS = " + String(avg, 1) + "\n" +
                "TICKS_PER_CELL = " + String(perCell, 1) + "\n" +
                "==== HET CALIB ====";
  calibPrinted = false;
  Serial.println(calibReport);
  Serial.println(">> [CALIB] DONE - PWM = 0");
  hold.cells = savedCells;
  pulsesPerCell = savedPulses;
}
