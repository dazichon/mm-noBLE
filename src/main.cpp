#include <Arduino.h>
#include "RobotConfig.h"
#include "SerialLink.h"
#include "RobotNav.h"
#include "CommandHandler.h"
#include "Motion.h"
#include "App.h"

// SEQ_MODE 7 (MainParams.h): kịch bản tự viết bằng các hàm trong lib/Motion/Motion.h, bấm nút để chạy
void kichBan() {
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
    quayPhai(90.0f);
}

void setup() {
    delay(500);
    serialLink.begin();
    robotNav.init();
    applyParams();
    pinMode(BUTTON, INPUT_PULLUP);

    Serial.println("==========================================");
    Serial.println(">> ROBOT MICROMOUSE READY!");
    Serial.println("==========================================");

    startupAction();
}

void loop() {
    if (motorTestLoop()) return;

    robotNav.update();
    commandHandler.update();
    if (buttonFired()) runSeqMode();
    printPendingLogs();
}
