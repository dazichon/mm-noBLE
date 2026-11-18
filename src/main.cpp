#include <Arduino.h>
#include "RobotConfig.h"
#include "SerialLink.h"
#include "RobotNav.h"
#include "CommandHandler.h"
#include "Motion.h"
#include "App.h"
#include "MainParams.h"
// BLE chỉ được biên dịch + gọi khi chạy mode 3 (hoặc khi dùng công tắc DS1, vì có thể gạt sang mode 3 lúc chạy)
#define BLE_BUILD (SEQ_MODE == 3 || MODE_SWITCH_ENABLE)
#if BLE_BUILD
#include "BleLink.h"
static void startBle(bool &on) { if (!on) { bleLink.begin(); on = true; Serial.println(">> [MODE] BAT BLE"); } }
#else
static void startBle(bool &) {}
#endif

void setup() {
    delay(500);
    serialLink.begin();
    robotNav.init();
    applyParams();
    robotNav.checkFlashLog();  // có log lần chạy trước trong flash thì in khi cắm USB
    initModeSwitch();  // đọc công tắc DS1 -> seqMode

    Serial.println("==========================================");
    Serial.println(">> ROBOT MICROMOUSE READY!");
    Serial.println("==========================================");

    startupAction();
}

void loop() {
    if (motorTestLoop()) return;

    robotNav.update();
    robotNav.recoverI2C();  // lỗi bus I2C: xe đứng yên thì thử khởi động lại cảm biến
    commandHandler.update();
    updateModeSwitch();
    static bool bleOn = false;
#if BLE_BUILD
    if (bleOn) bleLink.update();
#endif
#if AUTO_RUN_ON_BOOT
    // Bật nguồn: chờ -> calib MPU -> chạy luôn (1 lần). Mode 3: bật BLE
    static bool autoDone = false;
    if (!autoDone && millis() >= AUTO_RUN_DELAY_MS) {
        autoDone = true;
        if (seqMode == 3) {
            startBle(bleOn);
        } else if (!robotNav.i2cFault && robotNav.mpuReady) {
            Serial.printf(">> [AUTO] CALIB MPU... GIU XE DUNG YEN\n");
            robotNav.mpu6050.calibrate();
            robotNav.mpu6050.resetYaw();
            robotNav._turnHeadingValid = false;
            Serial.printf(">> [AUTO] CALIB XONG -> CHAY CHE DO %d\n", seqMode);
            runSeqMode();
        } else Serial.printf(">> [AUTO] KHONG CHAY: %s\n", robotNav.i2cFault ? "LOI I2C" : "MPU CHUA SAN SANG");
    }
#endif
    if (buttonFired()) {
        commitModeSwitch();  // gạt DS1 xong phải bấm nút thì chế độ mới có hiệu lực
        if (seqMode == 3) {  // CHỈ chế độ 3 bật BLE (bật rồi giữ tới khi tắt nguồn / reset)
            startBle(bleOn);
        } else if (!robotNav.i2cFault) runSeqMode();
    }
    printPendingLogs();
}
