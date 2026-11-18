# Code sử dụng tường TRƯỚC (VL53 front) – bản tự chứa, không kèm đường dẫn

Quy ước: `_rawF` = đọc thô (mm), `smoothDF` = đã lọc EMA, `frontReady` = cảm biến init OK.
Mã đặc biệt của `_rawF`: **-1** init lỗi, **-2** timeout (>200ms không mẫu mới), **-3** ngoài tầm (>1200mm). Quá gần kẹp về 15.
Mỗi mục: tên hàm/khối → code → ghi chú.

---

## 0. Tham số (MainParams.h)

```cpp
#define HOLD_USE_FRONT    0       // 0: không dùng VL53 trước để phanh trong holdRun
#define HOLD_TARGET_FRONT 150.0f  // tăng: dừng xa tường hơn

#define NAV_FRONT_WALL_MM 150     // VL53 trước < mức này (xe đứng giữa ô) = tường chắn ngay cuối ô hiện tại (đo ~113-153). Tường cách 1 ô ~200-260

#define MAP_ALIGN_FRONT 1         // 1: có tường trước thì tiến/lùi căn đúng FRONT_ALIGN_MM rồi mới quay
#define FRONT_ALIGN_MM  95.0f     // VL53 trước đọc bao nhiêu khi xe ĐÚNG TÂM Ô có tường trước
#define FRONT_ALIGN_SKIP 18.0f    // lệch ít hơn thì KHÔNG căn
#define FRONT_ALIGN_TOL 10.0f     // đã căn, vào dải này là dừng
#define FRONT_ALIGN_MIN_PWM 45.0f // PWM khởi đầu khi căn
#define FRONT_ALIGN_KP  0.3f      // PWM / mm
```

Gán vào robot (App.cpp):
```cpp
n.hold.useFront = HOLD_USE_FRONT;
n.hold.targetF = HOLD_TARGET_FRONT;
n.navFrontWallMm = NAV_FRONT_WALL_MM;
mapper.alignFront = MAP_ALIGN_FRONT;
n.frontAlignMm = FRONT_ALIGN_MM;
n.frontAlignTol = FRONT_ALIGN_TOL;
n.frontAlignSkip = FRONT_ALIGN_SKIP;
n.frontAlignMaxPwm = 55.0f;     // ghi đè default 70
n.frontAlignMaxMs = 800;        // ghi đè default 1500
n.frontAlignKp = FRONT_ALIGN_KP;
n.frontAlignMinPwm = FRONT_ALIGN_MIN_PWM;
```

Default trong RobotNav.h / RobotNav.cpp (bị ghi đè bởi đoạn trên, trừ 3 biến cuối):
```cpp
float frontAlignMm = 120.0f;
float frontAlignTol = 4.0f;
float frontAlignSkip = 12.0f;
float frontAlignKp = 1.0f;
float frontAlignMinPwm = 45.0f;
float frontAlignMaxPwm = 70.0f;
uint16_t frontAlignMaxMs = 1500;
uint16_t navFrontWallMm = 200;

// cửa sổ đọc tường trước khi đang chạy
float frontWinFrom = 0.55f, frontWinTo = 0.78f;
float frontMoveGain = 0.5f;   // VL53 đọc xa bị nén (tường xa thêm 1 ô chỉ tăng ~70-100mm) -> chỉ cộng 1 phần
uint8_t frontNearN = 0, frontFarN = 0;

// RobotNav.cpp (constructor)
frontStopDist = 123;   // phanh khi cách tường trước <= 123mm (tâm ô thực tế 121mm)
frontWallDist = 360;   // dF <= 360mm: có tường trước của Ô PHÍA TRƯỚC (~300mm)
smoothDF = 999.0f;
```

---

## 1. Đọc cảm biến (không chặn)

```cpp
uint16_t pollSensor(VL53L0X &sensor, bool ready, int16_t &raw,
                    unsigned long &lastNew, unsigned long now) {
  if (!ready) { raw = -1; return 999; }  // -1: init thất bại
  if (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    uint16_t r = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    lastNew = now;
    raw = (r > 1200) ? -3 : (r < 15 ? 15 : (int16_t)r);  // -3: ngoài tầm; quá gần kẹp 15
  } else if (now - lastNew > 200) {
    raw = -2;  // lâu không có mẫu mới => timeout
  }
  return raw < 0 ? 999 : (uint16_t)raw;
}

void RobotNav::updateSensors() {
  unsigned long now = millis();
  if (now - _lastSensorReadTime < 33) return;   // ~33ms/lần
  _lastSensorReadTime = now;

  uint16_t raw_dF = pollSensor(sensorFront, frontReady, _rawF, _tF, now);
  // (trái/phải tương tự)

  // EMA cho cảm biến trước (phản ứng nhanh để dừng kịp). Không có median như hai bên.
  if (raw_dF < 800) smoothDF = (0.6f * (float)raw_dF) + (0.4f * smoothDF);
  else              smoothDF = (float)raw_dF;
}
```
Lưu ý: lỗi (`raw<0`) -> trả 999 -> `smoothDF` coi như "rất xa / không có tường".

---

## 2. Phân loại tường trước/ô trước (xe đứng yên hoặc vừa dừng)

```cpp
WallStatus RobotNav::senseCurrentWalls() {
  updateSensors();
  WallStatus walls;
  // Tường sát xe (<=180mm): vách trước của CHÍNH ô hiện tại
  walls.hasFrontNear = frontReady && smoothDF > 20.0f && smoothDF <= 180.0f;
  // 180 < smoothDF <= frontWallDist: vách trước của Ô PHÍA TRƯỚC
  walls.hasFront = !walls.hasFrontNear && frontReady && smoothDF > 180.0f && smoothDF <= (float)frontWallDist;

  if (walls.hasFrontNear) {
    // Cảm biến hông 45° phản xạ từ vách trước ở cự ly ~ smoothDF*1.414 -> loại để không nhận nhầm
    float frontDiag = smoothDF * 1.414f;
    bool leftIsFrontReflection  = (fabs(smoothDL - frontDiag) < 32.0f && smoothDL > 125.0f);
    bool rightIsFrontReflection = (fabs(smoothDR - frontDiag) < 32.0f && smoothDR > 125.0f);
    walls.hasLeft  = leftReady  && smoothDL > 20.0f && smoothDL < (float)wallThreshold      && !leftIsFrontReflection;
    walls.hasRight = rightReady && smoothDR > 20.0f && smoothDR < (float)rightWallThreshold && !rightIsFrontReflection;
  } else {
    walls.hasLeft  = (leftReady  && smoothDL > 20.0f && smoothDL < (float)wallThreshold);
    walls.hasRight = (rightReady && smoothDR > 20.0f && smoothDR < (float)rightWallThreshold);
  }
  return walls;
}
```
Default: `wallThreshold = 180` (trái), `rightWallThreshold = 160` (phải).

Sau khi vừa tới ô mới (trong hàm đi 1 ô):
```cpp
curCellWalls.hasFront = status.hasFrontNear;
curCellWalls.hasFrontNear = status.hasFrontNear;
curWalls = curCellWalls;

// Chưa đi được nửa ô -> coi bị chặn
lastMoveBlocked = stepTraveledPulses < pulsesPerCell / 2;
if (lastMoveBlocked) {
  status.hasFrontNear = true;
  status.hasFront = false;
}
```

Ghi vào flood-fill:
```cpp
static void ffRecordAhead(ParentMaze &m, int x, int y, int h, const WallStatus &ws) {
  ffSetWall(m, x, y, h, ws.hasFrontNear, true); // vách trước ô đang đứng: đo lúc đứng yên -> ghi đè
  if (ws.hasFrontNear) return;                  // bị chắn: không nhìn được ô phía trước
  int ax = x + DX[h], ay = y + DY[h];
  if (ax < 0 || ax >= MAZE_SIZE || ay < 0 || ay >= MAZE_SIZE) return;
  ffSetWall(m, ax, ay, (h + 3) % 4, ws.hasLeft);
  ffSetWall(m, ax, ay, (h + 1) % 4, ws.hasRight);
  ffSetWall(m, ax, ay, h, ws.hasFront);
  m.cell(ax, ay).known = true;
}
```

---

## 3. Đọc an toàn khi đứng yên (WS_WALL / WS_OPEN / WS_UNKNOWN)

```cpp
static WallState classifyWall(const int16_t *s, uint8_t n, bool ready, int thr) {
  if (!ready || n < 3) return WS_UNKNOWN;
  uint8_t closeN = 0, farN = 0;
  for (uint8_t i = 0; i < n; i++) {
    if (s[i] == -3 || s[i] >= thr) farN++;   // -3 = ngoài tầm = không có vật gần
    else closeN++;
  }
  if (closeN * 2 > n) return WS_WALL;
  if (farN * 2 > n)   return WS_OPEN;
  return WS_UNKNOWN;
}

void RobotNav::senseWallsSafe(WallState &l, WallState &f, WallState &r) {
  int16_t sL[8], sF[8], sR[8];
  uint8_t nL = 0, nF = 0, nR = 0;
  unsigned long lastL = _tL, lastF = _tF, lastR = _tR;
  unsigned long t0 = millis();
  while (millis() - t0 < 250) {
    updateSensors();
    if (mpuReady) mpu6050.update();
    // chỉ lấy mẫu MỚI thật sự; -1, -2 không thêm
    if (_tF != lastF) { lastF = _tF; if (_rawF != -1 && _rawF != -2 && nF < 8) sF[nF++] = _rawF; }
    // (L, R tương tự)
    delay(2);
  }
  f = classifyWall(sF, nF, frontReady, navFrontWallMm);
  // l, r dùng navSideWallMm
}
```

Mapping – đọc tường lúc đứng yên ở tâm ô:
```cpp
WallState l, f, r;
docTuong(l, f, r);
if (f == WS_UNKNOWN) { endReason = "VL53 TRUOC KHONG DANG TIN"; break; }   // dừng dò đường
setWall(x, y, h, f == WS_WALL, true);
if (f == WS_WALL && alignFront) canTuongTruoc();   // căn lại đúng tâm ô theo VL53 trước
```

---

## 4. Đọc tường trước KHI ĐANG CHẠY (bỏ phiếu theo cửa sổ)

Trong `updatePIDLoop`, khối `stepCellActive`:
```cpp
long lastCellStart = stepTargetPulses - pulsesPerCell;
float frac = (float)(distTraveled - lastCellStart) / (float)pulsesPerCell;

// Tường TRƯỚC của ô đang đi vào: xe ở vị trí frac thì tường cách xa hơn lúc đứng tâm ô (1-frac) ô
// -> ngưỡng = navFrontWallMm + (1-frac)*cellMm*frontMoveGain. Tường ô sau nữa vẫn nằm ngoài ngưỡng.
if (frac >= frontWinFrom && frac <= frontWinTo && _tF != _sideLastTF) {
  _sideLastTF = _tF;
  float thr = navFrontWallMm + (1.0f - frac) * cellMm * frontMoveGain;
  if (_rawF >= 0 && _rawF < thr) { if (frontNearN < 255) frontNearN++; }
  else if ((_rawF >= 0 || _rawF == -3) && frontFarN < 255) frontFarN++;
}
```
Reset ở đầu mỗi lần chạy: `frontNearN = frontFarN = 0; _sideLastTF = _tF;`

Kết luận từ phiếu:
```cpp
WallState RobotNav::movingFrontWall() const {
  uint8_t n = frontNearN + frontFarN;
  if (n < 3) return WS_UNKNOWN;
  if (frontNearN * 2 >= n) return WS_WALL;     // >=50% gần: coi là có tường (an toàn)
  if (frontFarN * 4 >= n * 3) return WS_OPEN;  // >=75% xa mới coi là thông
  return WS_UNKNOWN;
}
```

Dùng trong chạy liền (`runStraight`) để quyết định phanh hay không:
```cpp
WallState fw = robotNav.movingFrontWall();
if (fw != WS_UNKNOWN) setWall(nx, ny, h, fw == WS_WALL, false);  // đứng yên đọc lại sẽ ghi đè
if (!inTarget(nx, ny) && fw == WS_OPEN && run + 1 < maxRun) {
  maze.floodfill_update(tX, tY, false, false, tS);
  keepGoing = (chooseDir(nx, ny, h) == h);
}
if (!keepGoing) robotNav.stepNoStop = false;   // giảm tốc bò + phanh ở tâm ô
```
Chỉ chạy tiếp không phanh khi `fw == WS_OPEN`.

---

## 5. Phanh / dừng theo tường trước trong PID đi thẳng

Trong `updatePIDLoop`:
```cpp
uint16_t dF = (uint16_t)smoothDF;

// holdRun: gặp tường trước thì đứng yên, PID vẫn chạy; tường xa ra > +8mm thì đi tiếp
if (holdRun) {
  if (hold.useFront && frontReady && dF > 20 && dF <= frontStopDist) {
    if (!_holdStopped) brakeMotors();     // hãm chủ động
    _holdStopped = true;
    _holdFreeSince = 0;
  } else if (_holdStopped) {
    // chỉ đi tiếp khi tường trước xa ra liên tục >= 500ms (chống gai đọc 999)
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

// Phanh dừng an toàn khi gặp vách tường trước (KHÔNG kiểm tra hold.useFront!)
if (!holdRun && frontReady && dF > 20 && dF <= frontStopDist) {
  brakeMotors();
  stopPID();
  Serial.println(">> [PID] PHANH DUNG: GAP VAC TUONG TRUOC (" + String(dF) + " mm)");
  return;
}
```
`startHold()` đặt: `frontStopDist = (uint16_t)hold.targetF;`
Lệnh Serial `FRONT_STOP_DIST=<v>` đổi `frontStopDist` lúc chạy.

⚠ Nhánh phanh thứ hai không phụ thuộc `HOLD_USE_FRONT`; và `FRONT_ALIGN_MM=95 < frontStopDist=123`, nên xe đã căn đúng tâm ô vẫn nằm trong vùng phanh.

---

## 6. CĂN TƯỜNG TRƯỚC – alignFrontWall()

```cpp
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

    // Lệch ít hơn frontAlignSkip ngay lần đọc đầu: bỏ qua, không căn
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

    // Xe đứng yên cần lực lớn hơn để nhích: khoảng cách không đổi (>2mm) thì tăng dần PWM
    if (fabsf(d - progD) > 2.0f) { progD = d; progT = millis(); }
    float boost = (millis() - progT) / 30.0f;
    float pwm = constrain(fabsf(e) * frontAlignKp + boost, frontAlignMinPwm, frontAlignMaxPwm);

    // Giữ hướng bằng gyro (P): corr > 0 -> quay trái; khi lùi đảo dấu
    float corr = 0.0f;
    if (mpuReady) corr = constrain(-hold.yawKp * wrap180(yawT - mpu6050.getYaw()), -10.0f, 10.0f);
    if (e < 0) corr = -corr;
    int pl = constrain((int)lroundf((pwm - corr) * motorScaleL), 0, 255);
    int pr = constrain((int)lroundf((pwm + corr) * motorScaleR), 0, 255);
    if (e > 0) {   // tiến
      analogWrite(M1_IN1, pl); analogWrite(M1_IN2, 0);
      analogWrite(M2_IN1, pr); analogWrite(M2_IN2, 0);
    } else {       // lùi
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
```
Wrapper: `inline bool canTuongTruoc() { return robotNav.alignFrontWall(); }`

### Các nơi gọi alignFrontWall / canTuongTruoc

Dò đường (sau khi đọc tường): xem mục 3 (`if (f == WS_WALL && alignFront) canTuongTruoc();`).

Speedrun – trước khi quay:
```cpp
int rel = (d - h + 4) % 4;
if (rel != 0 && alignFront && isKnown(x, y, h) && hasWall(x, y, h)) canTuongTruoc();
if (rel == 1) ok = quayPhai(); else if (rel == 3) ok = quayTrai(); else if (rel == 2) ok = quayDau();
```

Tự hành từng ô (mode 9) – ưu tiên rẽ trái > thẳng > rẽ phải > ngõ cụt:
```cpp
if (lastSideL == WS_OPEN) {                 // hông trái thông -> rẽ trái (kể cả trước thông)
  if (wf == WS_WALL) { alignFrontWall(); settle(30); }
  turnLeft90Test(90.0f); settle(30);
} else if (wf == WS_OPEN) {                 // trước thông -> đi thẳng
} else if (lastSideR == WS_OPEN) {          // trước chắn, phải thông -> rẽ phải
  alignFrontWall(); settle(30);
  turnLeft90Test(-90.0f); settle(30);
} else {                                    // ngõ cụt
  alignFrontWall(); settle(30);
  turnLeft90Test(180.0f); settle(30);
  calibCellFromBackWall(); settle(30);
}
```

Tự hành chạy liền (mode 11) – đọc `senseWallsSafe(dummyL, wf, dummyR)` rồi:
```cpp
if (lastSideL == WS_OPEN) {
  if (wf == WS_WALL) { alignFrontWall(); settle(30); }
  turnLeft90Test(90.0f); settle(30);
} else if (wf == WS_WALL) {
  if (lastSideR == WS_OPEN) {
    alignFrontWall(); settle(30);
    turnLeft90Test(-90.0f); settle(30);
  } else {
    alignFrontWall(); settle(30);
    turnLeft90Test(180.0f); settle(30);
    calibCellFromBackWall(); settle(30);
  }
}
```

Quyết định rẽ theo `hasFront` (tự hành bám tay trái/phải, patrol):
```cpp
walls.hasFront = senseCurrentWalls().hasFrontNear;   // chỉ bị chắn khi tường sát xe
if (patrolMode) {
  if (walls.hasFront) turnAroundAndStep();   // gặp tường: quay đầu 180
  else                moveOneCell();
}
// bàn tay phải:  !hasRight -> rẽ phải ; !hasFront -> thẳng ; !hasLeft -> rẽ trái ; else quay đầu
// bàn tay trái:  !hasLeft  -> rẽ trái ; !hasFront -> thẳng ; !hasRight -> rẽ phải ; else quay đầu
```

---

## 7. Báo cáo / log / giao diện (chỉ in hoặc hiển thị)

```cpp
// gói Serial/BLE:      fmtDist(front, sizeof(front), 'F', robotNav._rawF);
// gói số:              int dF = robotNav._rawF >= 0 ? robotNav._rawF : 999;
// status:              "TOF (L / F / R)     : " + dLeft + " / " + dFront + " / " + dRight
//                      "TOF RAW (L / F / R) : " + _rawL + " / " + _rawF + " / " + _rawR
// vách ô phía trước:   "Trước=" + (ws.hasFront ? "CÓ" : "TRỐNG")
// JSON vách:           "wf": hasFront, "wn": hasFrontNear, "pf": pre->hasFront
// log chạy (CSV):      cột VL53_F = e.dF = _rawF
// web-controller:      thẻ cảm biến phía trước (id cardSensorF) trong index.html, cập nhật bởi app.js
```

---

## 8. Bảng ngưỡng tổng hợp

| Ngưỡng | Giá trị | Dùng ở |
|---|---|---|
| sàn hợp lệ | 20 mm | mục 2, 5 |
| `frontStopDist` | 123 (default) / 150 (`HOLD_TARGET_FRONT`) | mục 5: phanh |
| `FRONT_ALIGN_MM` | 95 | mục 6: đích căn |
| `NAV_FRONT_WALL_MM` | 150 | mục 3, 4: phân loại tường |
| `hasFrontNear` | ≤ 180 (cứng) | mục 2 |
| `frontWallDist` | 360 | mục 2: tường ô phía trước |
| lọc mẫu căn | < 400 (cứng) | mục 6 |
| ngoài tầm | > 1200 -> -3 | mục 1 |
| cửa sổ đọc khi chạy | 0.55 – 0.78 ô | mục 4 |

## 9. Điểm không nhất quán
1. `FRONT_ALIGN_MM`(95), `frontStopDist`(123) và comment "tâm ô thực tế 121mm" cùng mô tả "tường trước cách bao nhiêu khi ở tâm ô" nhưng 95 ≠ 121. Nên đo lại một lần rồi dùng chung.
2. `NAV_FRONT_WALL_MM`(150, đứng yên) và `hasFrontNear`(180 cứng) là hai hệ phân loại khác nhau cho cùng việc.
3. Nhánh phanh trong PID (mục 5, khối thứ hai) bỏ qua `HOLD_USE_FRONT`.
4. Tường trước không có median/wall-drop như hai bên, chỉ EMA 0.6 và gán thẳng khi ≥ 800.
