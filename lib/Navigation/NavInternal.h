#pragma once
// Hàm tiện ích dùng chung giữa các file Nav*.cpp (chỉ include trong lib/Navigation)
#include <Arduino.h>
#include <VL53L0X.h>

// Đọc VL53L0X không chặn: trả mm hoặc 999 nếu lỗi; raw = -1 khi lỗi/timeout (định nghĩa ở NavSensors.cpp)
// statusOut: mã trạng thái phép đo mới nhất (11 = hợp lệ). vl53CheckStatus = true: bỏ phép đo không hợp lệ (coi là ngoài tầm)
uint16_t pollSensor(VL53L0X &sensor, bool ready, int16_t &raw, unsigned long &lastNew, unsigned long now,
                    uint8_t *statusOut = nullptr);
extern bool vl53CheckStatus;

// Đưa góc về [-180, 180]
inline float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
