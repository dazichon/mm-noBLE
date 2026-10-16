#pragma once
// ###########################################################################################################
// #  ĐIỀU KHIỂN XE - CÁC HÀM DÙNG HẰNG NGÀY                                                                  #
// #  Chỉ cần: #include "Motion.h"  rồi gọi các hàm bên dưới (trong setup()/loop() hoặc sau khi bấm nút).    #
// #  Tất cả đều CHẶN: hàm chạy xong (xe đã dừng / đã quay xong) mới trả về.                                 #
// #  Thông số (tốc độ, Kp, xung/ô, PWM quay...) chỉnh ở lib/MainParams/MainParams.h                         #
// ###########################################################################################################
//
// Ví dụ:
//   diThang(2);          // đi thẳng 2 ô rồi dừng ở tâm ô
//   quayTrai();          // quay trái 90° tại chỗ
//   quayDau();           // quay 180°
//   diThang(1, true);    // đi 1 ô KHÔNG dừng ...
//   cuaTrai();           // ... cua trái 90° khi đang chạy ...
//   diThang(1);          // ... rồi đi tiếp 1 ô và dừng
//
//   WallState l, f, r;
//   docTuong(l, f, r);   // xe đứng yên: đọc tường trái / trước / phải (WS_WALL, WS_OPEN, WS_UNKNOWN)

#include "RobotNav.h"

// ----- ĐI THẲNG -----
// Đi thẳng soO ô (PID encoder + gyro + tường), phanh dừng ở tâm ô.
// khongDung = true: hết quãng đường giữ nguyên tốc độ để gọi cuaTrai()/cuaPhai() ngay sau đó.
inline void diThang(int soO = 1, bool khongDung = false) {
  robotNav.driveCells(soO, khongDung);
  if (!khongDung) robotNav.settle(120);  // chờ xe đứng hẳn
}

// ----- QUAY TẠI CHỖ (gyro MPU6050) -----
// Trả về true nếu quay tới đúng góc, false nếu timeout 6s / chưa có MPU.
inline bool quayTrai(float goc = 90.0f) {
  bool ok = robotNav.turnLeft90Test(goc);
  robotNav.settle(80);
  return ok;
}
inline bool quayPhai(float goc = 90.0f) {
  bool ok = robotNav.turnLeft90Test(-goc);
  robotNav.settle(80);
  return ok;
}
inline bool quayDau() { return quayTrai(180.0f); }

// ----- CUA 90° KHI ĐANG CHẠY -----
// Gọi ngay sau diThang(n, true). Xe vẫn tiến, 2 bánh lệch tốc độ tới khi đủ 90°.
inline bool cuaTrai() {
  if (!robotNav.startCornerLeft()) return false;
  unsigned long t0 = millis();
  while (robotNav.cornerBusy() && millis() - t0 < 6000) robotNav.update();
  return !robotNav.cornerBusy();
}
inline bool cuaPhai() {
  if (!robotNav.startCornerRight()) return false;
  unsigned long t0 = millis();
  while (robotNav.cornerBusy() && millis() - t0 < 6000) robotNav.update();
  return !robotNav.cornerBusy();
}

// ----- DỪNG -----
inline void phanh() { robotNav.stopPID(); robotNav.brakeMotors(); }  // hãm chủ động tới khi bánh đứng yên
inline void thaMotor() { robotNav.stopPID(); }                       // ngắt motor, xe trôi tự do

// ----- NGÕ CỤT -----
// Gọi sau quayDau() ở ngõ cụt: lùi sát vách sau -> calib lại gyro (yaw = 0) -> tiến về tâm ô.
inline bool calibNgoCut() { return robotNav.calibCellFromBackWall(); }

// ----- CĂN THEO TƯỜNG TRƯỚC -----
// Xe đứng yên, trước mặt có tường: tiến/lùi tới khi VL53 trước = FRONT_ALIGN_MM (đúng tâm ô).
inline bool canTuongTruoc() { return robotNav.alignFrontWall(); }

// ----- CẢM BIẾN -----
// Xe phải đứng yên. Ngưỡng: NAV_SIDE_WALL_MM / NAV_FRONT_WALL_MM (gán robotNav.navSideWallMm / navFrontWallMm).
inline void docTuong(WallState &trai, WallState &truoc, WallState &phai) {
  robotNav.senseWallsSafe(trai, truoc, phai);
}

// ----- HƯỚNG GỐC -----
// Lấy hướng hiện tại của xe làm hướng chuẩn (mọi lần đi thẳng/quay sau đó tính theo hướng này).
// Gọi 1 lần khi đặt xe vào ô xuất phát.
inline void datHuongGoc() {
  robotNav.settle(200);
  robotNav.mpu6050.update();
  robotNav._turnHeading = robotNav.mpu6050.getYaw();
  robotNav._turnHeadingValid = true;
}

// Đứng yên ms mili-giây (vẫn cập nhật gyro). Dùng thay delay() giữa các lệnh.
inline void cho(unsigned long ms) { robotNav.settle(ms); }
