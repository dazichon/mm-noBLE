// CẢM BIẾN VL53L0X: đọc không chặn + lọc trung vị
#include "RobotNav.h"
#include "NavInternal.h"

// Trả về khoảng cách (mm) hoặc 999 nếu lỗi; cập nhật raw = -1 khi lỗi/timeout, không chặn vòng lặp
bool vl53CheckStatus = true;

uint16_t pollSensor(VL53L0X &sensor, bool ready, int16_t &raw,
                           unsigned long &lastNew, unsigned long now, uint8_t *statusOut) {
  if (!ready) { raw = -1; return 999; }  // -1: init thất bại
  if (sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) {
    // Mã trạng thái phép đo (bit 6..3): 11 = hợp lệ. Mã khác = tín hiệu yếu / nhiễu / không thấy vật
    // -> chip vẫn trả 1 con số rác (vd ~85mm khi phía trước trống) nên KHÔNG được dùng
    uint8_t st = (sensor.readReg(VL53L0X::RESULT_RANGE_STATUS) & 0x78) >> 3;
    uint16_t r = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    lastNew = now;
    if (statusOut) *statusOut = st;
    if (vl53CheckStatus && st != 11) raw = -3;  // đo không hợp lệ = coi như không thấy gì (ngoài tầm)
    else raw = (r > 1200) ? -3 : (r < 15 ? 15 : (int16_t)r);  // -3: ngoài tầm đo; quá gần thì kẹp về 15 (tường sát cảm biến)
  } else if (now - lastNew > 200) {
    raw = -2;  // lâu không có mẫu mới => timeout
  }
  return raw < 0 ? 999 : (uint16_t)raw;
}

void RobotNav::updateSensors() {
  unsigned long now = millis();
  // Giới hạn chu kỳ đọc cảm biến khoảng 35ms một lần (tương thích timing budget 33ms của VL53L0X)
  if (now - _lastSensorReadTime < 35) {
    return;
  }
  _lastSensorReadTime = now;

  // Đọc không chặn: chỉ lấy mẫu khi cảm biến đã có dữ liệu mới, ngược lại giữ giá trị cũ
  uint16_t raw_dL = pollSensor(sensorLeft, leftReady, _rawL, _tL, now);
  uint16_t raw_dF = pollSensor(sensorFront, frontReady, _rawF, _tF, now);
  uint16_t raw_dR = pollSensor(sensorRight, rightReady, _rawR, _tR, now);

  // Lọc EMA cho cảm biến trái
  if (raw_dL < 800) {
    smoothDL = (0.55f * (float)raw_dL) + (0.45f * smoothDL);
  } else {
    smoothDL = (0.2f * 999.0f) + (0.8f * smoothDL);
  }

  // Lọc EMA cho cảm biến phải
  if (raw_dR < 800) {
    smoothDR = (0.55f * (float)raw_dR) + (0.45f * smoothDR);
  } else {
    smoothDR = (0.2f * 999.0f) + (0.8f * smoothDR);
  }

  // Lọc EMA cho cảm biến trước (phản ứng nhanh để dừng kịp thời)
  if (raw_dF < 800) {
    smoothDF = (0.6f * (float)raw_dF) + (0.4f * smoothDF);
  } else {
    smoothDF = (float)raw_dF;
  }

}

// Trung vị 5 mẫu VL53 thô (mẫu lỗi/ngoài tầm tính là 999): chống gai nhiễu mà không trễ như EMA nặng
static int16_t median5(const int16_t *b) {
  int16_t t[5];
  memcpy(t, b, sizeof(t));
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 5; j++)
      if (t[j] < t[i]) { int16_t x = t[i]; t[i] = t[j]; t[j] = x; }
  return t[2];
}

void RobotNav::updateWallFilter() {
  if (_tL != _wLastTL) { _wLastTL = _tL; _wBufL[_wIdxL] = (_rawL > 0) ? _rawL : 999; _wIdxL = (_wIdxL + 1) % 5; _wFiltL = median5(_wBufL); }
  if (_tR != _wLastTR) { _wLastTR = _tR; _wBufR[_wIdxR] = (_rawR > 0) ? _rawR : 999; _wIdxR = (_wIdxR + 1) % 5; _wFiltR = median5(_wBufR); }
}
