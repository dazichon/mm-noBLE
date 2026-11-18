#include "BleLink.h"
#include "RobotNav.h"
#include "MainParams.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BleLink bleLink;

#define NUS_SERVICE "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define NUS_RX      "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  // web ghi vào đây
#define NUS_TX      "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  // xe notify ra đây

static BLECharacteristic *txChar = nullptr;

class SrvCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleLink._connected = true; }
  void onDisconnect(BLEServer *) override { bleLink._connected = false; bleLink._restartAdv = true; }
};

class RxCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { bleLink._onRx(String(c->getValue().c_str())); }
};

// ---- Bảng thông số có thể tune (tên giống MainParams.h) ----
enum PType : uint8_t { T_F, T_U8, T_U16, T_LONG, T_BOOL };
struct Param { const char *grp, *name; void *ptr; PType t; float mn, mx, step; };
static long bleStopLead = STOP_LEAD_PULSES;  // phanh sớm (xung) khi chạy thẳng bằng nút web
static RobotNav &N = robotNav;
static Param params[] = {
  // Ô / quãng đường
  {"O",     "pulsesPerCell",  &N.pulsesPerCell,        T_LONG, 500, 20000, 10},     // CELL_PULSES: xung của 1 ô
  {"O",     "cellMm",         &N.cellMm,               T_F,    100, 300,   0.5f},   // chiều dài 1 ô (mm)
  {"O",     "stopLead",       &bleStopLead,            T_LONG, 0,   1500,  10},     // STOP_LEAD_PULSES
  // Tốc độ
  {"Toc do","baseSpeed",      &N.baseForwardSpeed,     T_U8,   40,  255,   1},      // FORWARD_PWM
  {"Toc do","accelStartPwm",  &N.accelStartPwm,        T_F,    20,  150,   1},      // ACCEL_START_PWM
  {"Toc do","accelRampMs",    &N.accelRampMs,          T_U16,  0,   2000,  10},     // ACCEL_RAMP_MS
  {"Toc do","motorScaleL",    &N.motorScaleL,          T_F,    0.5f,1.5f,  0.005f}, // MOTOR_TRIM_LEFT
  {"Toc do","motorScaleR",    &N.motorScaleR,          T_F,    0.5f,1.5f,  0.005f}, // MOTOR_TRIM_RIGHT
  // Encoder
  {"Encoder","encKp",         &N.encKp,                T_F,    0,   2,     0.01f},  // ENC_STRAIGHT_KP
  {"Encoder","encMaxCorr",    &N.encMaxCorr,           T_F,    0,   40,    0.5f},   // ENC_MAX_CORR
  {"Encoder","launchMm",      &N.launchMm,             T_F,    0,   720,   10},     // LAUNCH_MM
  {"Toc do","launchScaleL",   &N.launchScaleL,         T_F,    0.5f,1.5f,  0.005f}, // LAUNCH_TRIM_LEFT
  {"Toc do","launchScaleR",   &N.launchScaleR,         T_F,    0.5f,1.5f,  0.005f}, // LAUNCH_TRIM_RIGHT
  // Gyro giữ hướng
  {"Yaw",   "useGyro",        &N.straightUseGyro,      T_BOOL, 0,   1,     1},      // STRAIGHT_USE_GYRO: 0 = đi thẳng chỉ encoder + tường
  {"Yaw",   "yawKp",          &N.hold.yawKp,           T_F,    0,   20,    0.1f},   // HOLD_YAW_KP
  {"Yaw",   "yawKi",          &N.hold.yawKi,           T_F,    0,   10,    0.1f},   // HOLD_YAW_KI
  {"Yaw",   "yawIMax",        &N.hold.yawIMax,         T_F,    0,   40,    0.5f},
  {"Yaw",   "yawMaxCorr",     &N.yawMaxCorr,           T_F,    0,   60,    0.5f},
  // Bám tường hông
  {"Tuong", "wallEnable",     &N.hold.wallEnable,      T_BOOL, 0,   1,     1},      // WALL_ENABLE
  {"Tuong", "wallKp",         &N.hold.wallKp,          T_F,    0,   5,     0.05f},  // WALL_KP
  {"Tuong", "wallKd",         &N.wallKdPerDeg,         T_F,    0,   20,    0.1f},   // WALL_KD: PWM / độ chéo so với tường (khi useGyro = 0)
  {"Tuong", "wallMaxCorr",    &N.hold.wallMaxCorr,     T_F,    0,   30,    0.5f},   // WALL_MAX_CORR
  {"Tuong", "wallYawGain",    &N.hold.wallYawGain,     T_F,    0,   1,     0.01f},  // WALL_YAW_GAIN
  {"Tuong", "wallYawMax",     &N.hold.wallYawMax,      T_F,    0,   20,    0.5f},   // WALL_YAW_MAX
  {"Tuong", "wallBaseL",      &N.hold.wallBaseL,       T_F,    50,  300,   1},      // WALL_BASE_L
  {"Tuong", "wallBaseR",      &N.hold.wallBaseR,       T_F,    50,  300,   1},      // WALL_BASE_R
  {"Tuong", "wallValidMax",   &N.hold.wallValidMax,    T_F,    100, 400,   1},      // WALL_VALID_MAX
  {"Tuong", "useFront",       &N.hold.useFront,        T_BOOL, 0,   1,     1},      // HOLD_USE_FRONT
  {"Tuong", "targetF",        &N.hold.targetF,         T_F,    30,  200,   1},      // HOLD_TARGET_FRONT
};
static const int NPARAM = sizeof(params) / sizeof(params[0]);

static float getP(const Param &p) {
  switch (p.t) {
    case T_F: return *(float *)p.ptr;
    case T_U8: return *(uint8_t *)p.ptr;
    case T_U16: return *(uint16_t *)p.ptr;
    case T_LONG: return (float)*(long *)p.ptr;
    default: return *(bool *)p.ptr ? 1.0f : 0.0f;
  }
}
static void setP(const Param &p, float v) {
  v = constrain(v, p.mn, p.mx);
  switch (p.t) {
    case T_F: *(float *)p.ptr = v; break;
    case T_U8: *(uint8_t *)p.ptr = (uint8_t)lroundf(v); break;
    case T_U16: *(uint16_t *)p.ptr = (uint16_t)lroundf(v); break;
    case T_LONG: *(long *)p.ptr = lroundf(v); break;
    default: *(bool *)p.ptr = v >= 0.5f; break;
  }
}

void BleLink::begin(const char *name) {
  BLEDevice::init(name);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new SrvCb());
  BLEService *svc = srv->createService(NUS_SERVICE);
  txChar = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
  txChar->addDescriptor(new BLE2902());
  BLECharacteristic *rx = svc->createCharacteristic(
      NUS_RX, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new RxCb());
  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf(">> [BLE] Dang quang ba: %s\n", name);
}

void BleLink::_onRx(const String &data) {
  for (size_t i = 0; i < data.length(); i++) {
    char c = data[i];
    if (c == '\n' || c == '\r') {
      _rx.trim();
      if (_rx.equalsIgnoreCase("STOP")) _abort = true;  // dừng khẩn: xử lý ngay trong vòng chạy
      if (_rx.length() && !_cmdReady) { _cmd = _rx; _cmdReady = true; }
      _rx = "";
    } else if (_rx.length() < 80) {
      _rx += c;
    }
  }
}

void BleLink::send(const String &line) {
  if (!_connected || !txChar) return;
  String s = line + "\n";
  for (size_t i = 0; i < s.length(); i += 20) {
    String part = s.substring(i, min((size_t)s.length(), i + 20));
    txChar->setValue((uint8_t *)part.c_str(), part.length());
    txChar->notify();
    delay(6);
  }
}

void BleLink::sendParam(int i) {
  const Param &p = params[i];
  send(String("P ") + p.name + "=" + String(getP(p), 4) + "|" + String(p.mn, 3) + "|" + String(p.mx, 3) + "|" +
       String(p.step, 4) + "|" + p.grp);
}

void BleLink::sendAllParams() {
  for (int i = 0; i < NPARAM; i++) sendParam(i);
  send("PEND");
}

void BleLink::setParam(const String &arg) {
  int eq = arg.indexOf('=');
  if (eq < 1) return;
  String name = arg.substring(0, eq);
  float v = arg.substring(eq + 1).toFloat();
  for (int i = 0; i < NPARAM; i++) {
    if (name.equalsIgnoreCase(params[i].name)) {
      setP(params[i], v);
      sendParam(i);
      Serial.printf(">> [BLE] SET %s = %.4f\n", params[i].name, getP(params[i]));
      return;
    }
  }
  send("ERR unknown " + name);
}

struct Rec { uint32_t t; int16_t l, f, r; float yaw; long el, er; int pl, pr; };

static Rec snap(unsigned long t0) {
  return {(uint32_t)(millis() - t0), N._rawL, N._rawF, N._rawR, N.mpu6050.getYaw(),
          N.getLeftEncoder(), N.getRightEncoder(), N.currentLeftSpeed, N.currentRightSpeed};
}

void BleLink::sendRecs(const Rec *rec, int n) {
  char buf[96];
  for (int i = 0; i < n; i++) {
    const Rec &e = rec[i];
    snprintf(buf, sizeof(buf), "LOG %d,%lu,%d,%d,%d,%.2f,%ld,%ld,%d,%d", i + 1, (unsigned long)e.t, e.l, e.f, e.r, e.yaw, e.el, e.er, e.pl, e.pr);
    send(buf);
  }
  send("LOGEND");
}

void BleLink::runLog20() {
  Rec rec[20];
  unsigned long t0 = millis(), next = t0;
  for (int i = 0; i < 20; i++) {
    while (millis() < next) { N.update(); delay(1); }
    next += 50;
    rec[i] = snap(t0);
  }
  sendRecs(rec, 20);
}

// Đi thẳng n ô, ghi mẫu cảm biến mỗi 100ms (tối đa 200 mẫu) rồi in ra sau khi xong
void BleLink::runCells(int n) {
  static Rec rec[200];
  n = constrain(n, 1, 20);
  _abort = false;
  N.stopLeadPulses = bleStopLead;
  send("RUN start " + String(n));
  N.resetEnc();
  N.startCells(n, false);
  unsigned long t0 = millis(), next = t0;
  int cnt = 0;
  while (N.pidRunActive && !_abort && millis() - t0 < 15000UL * n) {
    N.update();
    if (millis() >= next && cnt < 200) { next += 100; rec[cnt++] = snap(t0); }
  }
  if (N.pidRunActive) N.stopPID();
  N.stopLeadPulses = 0;
  unsigned long dur = millis() - t0;
  delay(300);
  N.mpu6050.update();
  send(String("RUN ") + (_abort ? "aborted" : "done") + " " + String(dur) + "ms | encL=" + N.getLeftEncoder() +
       " encR=" + N.getRightEncoder() + " yaw=" + String(N.mpu6050.getYaw(), 2) + " | " + cnt + " mau");
  sendRecs(rec, cnt);
}

void BleLink::update() {
  if (_restartAdv) { _restartAdv = false; delay(100); BLEDevice::startAdvertising(); }
  // Phát cảm biến sống ~5Hz: "S L,F,R,yaw,encL,encR" (VL53 mm, -1 = lỗi)
  static unsigned long lastS = 0;
  if (_connected && !_cmdReady && millis() - lastS >= 200) {
    lastS = millis();
    char b[64];
    snprintf(b, sizeof(b), "S %d,%d,%d,%.2f,%ld,%ld", N._rawL, N._rawF, N._rawR, N.mpu6050.getYaw(),
             N.getLeftEncoder(), N.getRightEncoder());
    send(b);
  }
  if (!_cmdReady) return;
  String c = _cmd;
  _cmdReady = false;
  String u = c; u.toUpperCase();
  if (u == "PARAMS") sendAllParams();
  else if (u.startsWith("SET ")) setParam(c.substring(4));
  else if (u == "LOG20") runLog20();
  else if (u.startsWith("RUN ")) runCells(u.substring(4).toInt());
  else if (u == "ZERO") {  // reset yaw + encoder về 0
    N.mpu6050.resetYaw(); N._turnHeading = 0.0f; N._turnHeadingValid = true; N.resetEnc();
    send("MSG da reset yaw + encoder ve 0");
  } else if (u == "CALIB") {  // calib gyro: xe phải đứng yên ~1.5s
    N.stopPID(); send("MSG dang calib MPU, giu xe dung yen...");
    N.mpu6050.calibrate(400); N._turnHeading = 0.0f; N._turnHeadingValid = true;
    send("MSG calib MPU xong, yaw = 0");
  }
  else if (u == "STOP") { N.stopPID(); send("RUN aborted"); }
}
