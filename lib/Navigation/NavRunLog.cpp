// LƯU LOG LẦN CHẠY VÀO FLASH (LittleFS): chạy rời USB rồi cắm máy tính (kể cả khi mở serial làm ESP32 reset) vẫn còn log để in ra.
#include "RobotNav.h"
#include <LittleFS.h>

static const char *RUNLOG_PATH = "/runlog.txt";
static bool fsMounted = false;

static bool mountFs() {
  if (!fsMounted) fsMounted = LittleFS.begin(true);   // true: lần đầu chưa có hệ file thì tự format
  return fsMounted;
}

// Gọi khi kết thúc lần chạy: ghi nhật ký + log các lần cua ra flash
bool RobotNav::saveRunLog() {
  if (!mountFs()) return false;
  File f = LittleFS.open(RUNLOG_PATH, "w");
  if (!f) return false;
  f.print(navReport);
  f.print("\n");
  f.print(slalomLogText());
  f.close();
  return true;
}

// Gọi lúc khởi động: có log chưa in từ lần chạy trước thì đánh dấu để in khi cắm USB
void RobotNav::checkFlashLog() {
  flashLogPending = mountFs() && LittleFS.exists(RUNLOG_PATH);
}

void RobotNav::printFlashLog() {
  if (!mountFs()) { flashLogPending = false; return; }
  File f = LittleFS.open(RUNLOG_PATH, "r");
  if (!f) { flashLogPending = false; return; }
  Serial.println("==== LOG LAN CHAY TRUOC (luu trong flash) ====");
  uint8_t buf[256];
  while (f.available()) {
    size_t n = f.read(buf, sizeof(buf));
    Serial.write(buf, n);
    delay(2);   // nhường cho USB CDC, tránh rớt dữ liệu
  }
  f.close();
  Serial.println("==== HET LOG FLASH ====");
  LittleFS.remove(RUNLOG_PATH);
  flashLogPending = false;
}

// Log đã in ra màn hình trực tiếp rồi: bỏ bản trong flash để lần khởi động sau không in lại
void RobotNav::dropFlashLog() {
  if (mountFs()) LittleFS.remove(RUNLOG_PATH);
  flashLogPending = false;
}
