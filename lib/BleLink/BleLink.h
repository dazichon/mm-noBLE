#pragma once
// BLE (Nordic UART) cho web tune PID: web gửi lệnh dạng text kết thúc '\n', xe trả lời bằng notify.
//   PARAMS          -> xe trả về mọi thông số: dòng "P name=value|min|max|step"
//   SET name=value  -> đặt thông số (có kẹp min/max), xe trả "P name=value|..." mới
//   RUN n           -> đi thẳng n ô (bám PID), "STOP" dừng khẩn
//   LOG20           -> đọc 20 mẫu cảm biến (50ms/mẫu): "LOG i,ms,L,F,R,yaw,encL,encR,pwmL,pwmR", cuối cùng "LOGEND"
#include <Arduino.h>

class BleLink {
public:
  void begin(const char *name = "MM-Robot-BLE");
  void update();                       // gọi trong loop(): xử lý lệnh nhận được
  bool connected() const { return _connected; }
  void send(const String &line);       // gửi 1 dòng (tự thêm '\n'), chia gói 20 byte

  // Phần dưới chỉ để callback BLE dùng
  void _onRx(const String &data);
  volatile bool _connected = false;
  volatile bool _restartAdv = false;

private:
  String _rx;
  String _cmd;
  volatile bool _cmdReady = false;
  void sendParam(int i);
  void sendAllParams();
  void setParam(const String &arg);
  void runLog20();
  void runCells(int n);
  void sendRecs(const struct Rec *rec, int n);
  volatile bool _abort = false;
};

extern BleLink bleLink;
