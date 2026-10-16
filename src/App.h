#pragma once
// Các bước khởi động / chạy chế độ / in log, tách khỏi main.cpp cho gọn

void applyParams();       // gán toàn bộ thông số MainParams.h vào robotNav
void startupAction();     // TEST_MOTOR_DRIVE hoặc HOLD_AUTO_START (nếu bật trong MainParams.h)
bool motorTestLoop();     // true: đang ở chế độ TEST_MOTOR_DRIVE (loop() không làm gì khác)
bool buttonFired();       // true đúng 1 lần sau khi bấm nút BUTTON và chờ TURN_DELAY_MS
void runSeqMode();        // chạy chế độ SEQ_MODE đã chọn
void printPendingLogs();  // cắm USB thì in các log còn chờ (mỗi log 1 lần)

void kichBan();           // SEQ_MODE 7: viết trong main.cpp
