#pragma once
// Các bước khởi động / chạy chế độ / in log, tách khỏi main.cpp cho gọn

void applyParams();       // gán toàn bộ thông số MainParams.h vào robotNav
void startupAction();     // TEST_MOTOR_DRIVE hoặc HOLD_AUTO_START (nếu bật trong MainParams.h)
bool motorTestLoop();     // true: đang ở chế độ TEST_MOTOR_DRIVE (loop() không làm gì khác)
bool buttonFired();       // true đúng 1 lần sau khi bấm nút BUTTON và chờ TURN_DELAY_MS
void runSeqMode();        // chạy chế độ SEQ_MODE đã chọn
void printPendingLogs();  // cắm USB thì in các log còn chờ (mỗi log 1 lần)


extern int seqMode;                         // chế độ đang chọn (DS1, hoặc SEQ_MODE nếu MODE_SWITCH_ENABLE = 0)
void applyModeParams();                     // gán thông số phụ thuộc chế độ
void initModeSwitch();                      // cấu hình GPIO0 (BW) đọc ADC + đọc DS1 lần đầu
void updateModeSwitch(bool force = false);  // đọc DS1 khi xe đứng yên -> chế độ chờ
void commitModeSwitch();                    // bấm nút: kích hoạt chế độ chờ
