
#include "RobotNav.h"
#include <Preferences.h>

RobotNav robotNav;

RobotNav::RobotNav()
    : mpu6050(MPU6050_ADDR) {
  leftReady = false;
  frontReady = false;
  rightReady = false;
  mpuReady = false;
  pidRunActive = false;

  // Thông số quay đã căn ở Chế độ 1 - dùng chung cho mọi lần quay (chế độ 1, 2, bám tường, flood-fill)
  leftCompensation = 5.0f;
  rightCompensation = 6.0f;
  turnSpeed = 65;
  baseForwardSpeed = 75;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  _targetYaw = 0.0f;
  _lastPIDLoopTime = 0;

  // Giá trị thực tế đo được tại tâm ô (Theo kết quả đo mới nhất trên sa hình)
  wallThreshold = 180;      // Tường TRÁI: dL < 180mm là có tường
  rightWallThreshold = 160; // Tường PHẢI: dR < 160mm là có tường
  frontStopDist = 123; // Phanh dừng khi cách tường trước <= 123mm (tâm ô thực tế là 121mm)
  frontWallDist = 360; // dF <= 360mm: có tường trước của Ô PHÍA TRƯỚC (~300mm)

  // Giá trị cảm biến lọc & Vùng chết tâm ô
  smoothDL = 171.0f;
  smoothDF = 999.0f;
  smoothDR = 146.0f;
  _lastSensorReadTime = 0;

  // Cấu hình đồng bộ bánh bằng Encoder (Cascaded Inner Loop)
  encoderSyncActive = true;
  encKp = 0.35f;
  invertEncLeft = false;
  invertEncRight = false;
  _lastEncLeft = 0;
  _lastEncRight = 0;
  _startEncLeft = 0;

  // Chạy từng ô theo xung Encoder (Cell Stepping)
  pulsesPerCell = 4600; // Đo thực tế từ tâm ô 1 tới giữa ô 2 = 4400 xung
  stepCellActive = false;
  stepStartPulses = 0;
  stepStartL = 0;
  stepStartR = 0;
  stepTargetPulses = 0;
  stepTraveledPulses = 0;
  stepStartTime = 0;

  // Chế Độ 3: Bám Tường Tự Động (Autonomous Wall Follower)
  autoWallFollowActive = false;
  patrolMode = false;
  sideSampleFrac = 0.30f;
  curWalls = {false, false, false};
  followRightHand = true; // Mặc định ưu tiên luật bàn tay phải
  autoCellCount = 0;
  autoMaxCells = 60; // Tối đa 60 ô phòng lặp vô tận khi thử nghiệm
  lastAutoDecision = "SẴN SÀNG";

  // Chế Độ 4: Flood-fill
  ffActive = false;
  ffMode = 0;
  ffGoal = {7, 7};
  ffGoalSize = 2; // 4 ô trung tâm mê cung 16x16
  ffCols = ffRows = MAZE_SIZE;
  lastMoveBlocked = false;
  resetFloodFill(0, 0, 0);
}

long RobotNav::getLeftEncoder() const {
  return invertEncLeft ? -enc1A_count : enc1A_count;
}

long RobotNav::getRightEncoder() const {
  return invertEncRight ? -enc2A_count : enc2A_count;
}

void RobotNav::resetEnc() {
  resetEncoders();
  _lastEncLeft = 0;
  _lastEncRight = 0;
  _startEncLeft = 0;
  _startEncRight = 0;
}

void RobotNav::init() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  stopMotors();

  // Nạp tốc độ chạy thẳng đã lưu (chưa lưu lần nào -> giữ mặc định)
  {
    Preferences p;
    if (p.begin("mmparams", true)) {
      baseForwardSpeed = p.getUChar("fspd", baseForwardSpeed);
      p.end();
    }
  }

  // Khởi tạo phần cứng Encoder đọc xung bánh xe
  setupEncoders();
  resetEnc();
  Serial.println("ENCODERS INITIALIZED");

  initI2CSensors();
}

// Khởi tạo bus I2C + 3 VL53L0X + MPU6050 (calib gyro: xe phải đứng yên). Dùng lúc bật nguồn và khi khôi phục lỗi bus
bool RobotNav::initI2CSensors() {
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);

  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  leftReady = initVL53(sensorLeft, XSHUT_LEFT, ADDRESS_LEFT, "LEFT");
  frontReady = initVL53(sensorFront, XSHUT_FRONT, ADDRESS_FRONT, "FRONT");
  rightReady = initVL53(sensorRight, XSHUT_RIGHT, ADDRESS_RIGHT, "RIGHT");

  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("MPU6050 FOUND @ 0x68");
    mpuReady = mpu6050.begin();
    if (mpuReady) {
      mpu6050.calibrate();
      Serial.println("MPU6050 READY - Z AXIS ONLY");
      serialLink.println(">> MPU6050 & SENSORS READY!");
    } else {
      Serial.println("MPU6050 INIT FAIL");
    }
  } else {
    Serial.println("MPU6050 NOT FOUND @ 0x68");
  }
  _i2cBadCycles = 0;
  return leftReady && frontReady && rightReady && mpuReady;
}

// Gọi sau mỗi lần đọc cảm biến: VL53 nào đang dùng mà giao tiếp lỗi (last_status != 0) hoặc MPU đọc lỗi liên tiếp
// -> đếm; lỗi liên tục ~5 chu kỳ (~175ms) thì coi là hỏng bus
void RobotNav::checkI2CFault() {
  if (i2cFault) return;
  bool bad = (leftReady && sensorLeft.last_status != 0) || (frontReady && sensorFront.last_status != 0) ||
             (rightReady && sensorRight.last_status != 0);
  if (bad) { if (_i2cBadCycles < 255) _i2cBadCycles++; }
  else _i2cBadCycles = 0;
  if (_i2cBadCycles >= 5) enterI2CFault("VL53 khong tra loi");
  else if (mpuReady && mpu6050.readFails() >= 50) enterI2CFault("MPU6050 khong tra loi");
}

void RobotNav::enterI2CFault(const char *why) {
  i2cFault = true;
  stopPID();  // dừng motor + huỷ chạy thẳng
  cornerState = STRAIGHT;
  ffActive = false;
  // Ngừng đọc mọi cảm biến I2C: hết in lỗi liên tục, các vòng chạy thấy cảm biến không sẵn sàng thì thoát
  leftReady = frontReady = rightReady = false;
  mpuReady = false;
  _i2cRetryT = millis();
  navPrintf(">> [I2C] LOI BUS I2C (%s) -> DUNG XE, HUY LUOT CHAY. Kiem tra nguon pin / day cam bien\n", why);
  navReportPending = !(bool)Serial;
}

// Xe đứng yên: khởi động lại bus I2C và các cảm biến. Thành công thì xoá lỗi (bấm nút để chạy lại)
bool RobotNav::recoverI2C() {
  if (!i2cFault || pidRunActive) return false;
  if (millis() - _i2cRetryT < i2cRetryMs) return false;
  _i2cRetryT = millis();
  stopMotors();
  Serial.println(">> [I2C] THU KHOI DONG LAI BUS I2C...");
  Wire.end();
  delay(50);
  bool ok = initI2CSensors();
  if (ok) {
    i2cFault = false;
    if (frontSignalLimit > 0.0f) sensorFront.setSignalRateLimit(frontSignalLimit);  // khởi tạo lại làm mất cấu hình này
    Serial.println(">> [I2C] KHOI PHUC XONG, BAM NUT DE CHAY LAI");
  } else {
    leftReady = frontReady = rightReady = false;
    mpuReady = false;
    _i2cBadCycles = 0;
    Serial.println(">> [I2C] CHUA KHOI PHUC DUOC, SE THU LAI");
  }
  return ok;
}

void RobotNav::saveForwardSpeed() {
  Preferences p;
  if (!p.begin("mmparams", false)) return;
  p.putUChar("fspd", baseForwardSpeed);
  p.end();
}

bool RobotNav::initVL53(VL53L0X &sensor, uint8_t xshutPin, uint8_t address,
                        const char *name) {
  digitalWrite(xshutPin, HIGH);
  delay(50);
  sensor.setTimeout(50);

  if (!sensor.init()) {
    Serial.print(name);
    Serial.println(" VL53 FAIL");
    return false;
  }

  sensor.setAddress(address);
  sensor.startContinuous();
  Serial.print(name);
  Serial.print(" VL53 OK @ 0x");
  Serial.println(address, HEX);
  return true;
}

void RobotNav::stopMotors() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);  // chân motor đang ở chế độ PWM: digitalWrite() sẽ báo lỗi "not set as GPIO", analogWrite 0 là đủ
}

void RobotNav::brakeMotors() {
  analogWrite(M1_IN1, 255);
  analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255);
  analogWrite(M2_IN2, 255);
  delay(brakeMs);
  // Giữ phanh cho tới khi cả 2 bánh đứng yên hẳn (tối đa brakeMaxMs) rồi mới thả: thả sớm thì bánh còn đà sẽ lăn thêm
  unsigned long t0 = millis();
  long pl = getLeftEncoder(), pr = getRightEncoder();
  while (millis() - t0 < brakeMaxMs) {
    delay(15);
    long cl = getLeftEncoder(), cr = getRightEncoder();
    bool still = labs(cl - pl) <= 2 && labs(cr - pr) <= 2;
    pl = cl;
    pr = cr;
    if (still) break;
  }
  stopMotors();
}


void RobotNav::update() {
  if (i2cFault) { stopMotors(); return; }
  updateSensors();

  if (cornerState != STRAIGHT) {
    if (mpuReady) mpu6050.update();
    updateCorner();
    return;
  }

  if (ffActive) {
    stepFloodFill();
    return;
  }

  if (autoWallFollowActive) {
    stepAutoWallFollow();
    return;
  }

  if (pidRunActive) {
    updatePIDLoop();
  }

  if (mpuReady) {
    mpu6050.update();
  }
}
