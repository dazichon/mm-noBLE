#include <Arduino.h>
#include "App.h"
#include "RobotConfig.h"
#include "RobotNav.h"
#include "CommandHandler.h"
#include "MainParams.h"
#include "Mapping.h"

// ================= THÔNG SỐ =================

void applyParams() {
    RobotNav &n = robotNav;
    n.motorScaleL = 1.0f + MOTOR_TRIM_LEFT / 100.0f;
    n.motorScaleR = 1.0f + MOTOR_TRIM_RIGHT / 100.0f;
    n.accelStartPwm = ACCEL_START_PWM;
    n.accelRampMs = ACCEL_RAMP_MS;

    // Quay tại chỗ
    n.turnMaxPwm = TURN_MAX_PWM;
    n.turnMinPwm = TURN_MIN_PWM;
    n.turnDecelZone = TURN_DECEL_ZONE;
    n.turnTolerance = TURN_TOLERANCE;
    n.turnLeftYawSign = TURN_LEFT_YAW_SIGN;
    n.turnBrakeLead = TURN_BRAKE_LEAD;

    // Cua khi đang chạy
    n.cornerSpeed = CORNER_SPEED;
    n.cornerInnerRatio = CORNER_INNER_RATIO;
    n.cornerDecelZone = CORNER_DECEL_ZONE;
    n.cornerTolerance = CORNER_TOLERANCE;
    n.cornerLead = CORNER_LEAD;

    // PID đi thẳng
    n.encKp = ENC_STRAIGHT_KP;
    n.encMaxCorr = ENC_MAX_CORR;
    n.hold.yawKp = HOLD_YAW_KP;
    n.hold.yawKi = HOLD_YAW_KI;
    n.hold.wallEnable = WALL_ENABLE;
    n.hold.wallKp = WALL_KP;
    n.hold.wallMaxCorr = WALL_MAX_CORR;
    n.hold.wallYawGain = WALL_YAW_GAIN;
    n.hold.wallYawMax = WALL_YAW_MAX;
    n.hold.wallBaseL = WALL_BASE_L;
    n.hold.wallBaseR = WALL_BASE_R;
    n.hold.wallValidMax = WALL_VALID_MAX;
    n.hold.useFront = HOLD_USE_FRONT;
    n.hold.targetF = HOLD_TARGET_FRONT;

    // Ô, calib ngõ cụt, đọc tường
    n.pulsesPerCell = CELL_PULSES;
    n.seqCellPulses = SEQ_CELL_PULSES;
    n.hold.cells = TEST_CELLS;
    n.calibRevPwm = CALIB_REV_PWM;
    n.calibRevMs = CALIB_REV_MS;
    n.calibFwdPulses = CALIB_FWD_PULSES;
    n.navSideWallMm = NAV_SIDE_WALL_MM;
    n.navFrontWallMm = NAV_FRONT_WALL_MM;
    n.navMaxSteps = NAV_MAX_STEPS;
    n.navStartSidesWall = NAV_START_SIDES_WALL;

    // Flood-fill
    n.ffContinuous = FF_CONTINUOUS;
    n.ffMaxRun = FF_MAX_RUN;

    // Dò đường + vẽ bản đồ
    mapper.cols = MAP_COLS;
    mapper.rows = MAP_ROWS;
    mapper.goalX = MAP_GOAL_X;
    mapper.goalY = MAP_GOAL_Y;
    mapper.goalSize = MAP_GOAL_SIZE;
    mapper.maxCells = MAP_MAX_CELLS;
    mapper.calibBack = MAP_CALIB_BACK;
    mapper.alignFront = MAP_ALIGN_FRONT;
    n.frontAlignMm = FRONT_ALIGN_MM;
    n.frontAlignTol = FRONT_ALIGN_TOL;
    n.frontAlignSkip = FRONT_ALIGN_SKIP;
    n.frontAlignMaxPwm = 55.0f;
    n.frontAlignMaxMs = 800;
    n.frontAlignKp = FRONT_ALIGN_KP;
    n.frontAlignMinPwm = FRONT_ALIGN_MIN_PWM;
    n.sideWinFrom = MAP_SIDE_FROM;
    n.sideWinTo = MAP_SIDE_TO;
    mapper.startSidesWall = NAV_START_SIDES_WALL;
}

// ================= KHỞI ĐỘNG =================

void startupAction() {
#if TEST_MOTOR_DRIVE
    delay(HOLD_START_DELAY_MS);
    robotNav.resetEnc();
    robotNav.clearLog();
    // Test cân bằng motor: tắt PID/tường/gyro/trim, cùng 1 PWM cho 2 bánh
    robotNav.motorScaleL = 1.0f;
    robotNav.motorScaleR = 1.0f;
    robotNav.currentLeftSpeed = TEST_PWM;
    robotNav.currentRightSpeed = TEST_PWM;
    analogWrite(M1_IN1, TEST_PWM);
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, TEST_PWM);
    analogWrite(M2_IN2, 0);
#elif HOLD_AUTO_START
    // Chờ (vẫn đọc cảm biến để giá trị lọc ổn định), động cơ đứng yên
    unsigned long t0 = millis();
    while (millis() - t0 < HOLD_START_DELAY_MS) {
        robotNav.updateSensors();
        commandHandler.update();
    }
    // Hiệu chuẩn lại gyro (xe phải đứng yên) và đưa yaw về 0 rồi mới chạy PID
    if (robotNav.mpuReady) {
        Serial.println(">> CALIB MPU6050... GIU XE DUNG YEN");
        robotNav.mpu6050.calibrate();
        robotNav.mpu6050.resetYaw();
        Serial.println(">> CALIB XONG, YAW = 0");
    }
    robotNav.startHold();
#endif
}

// ================= TEST LỆCH 2 ĐỘNG CƠ (TEST_MOTOR_DRIVE) =================

bool motorTestLoop() {
#if TEST_MOTOR_DRIVE
    static bool done = false, printed = false;
    static long rawL = 0, rawR = 0, diff = 0;
    static float pct = 0.0f;
    if (!done) {
        robotNav.logSample();
        long l = labs(robotNav.getLeftEncoder());
        long r = labs(robotNav.getRightEncoder());
        if ((l + r) / 2 >= TEST_PULSES) {
            robotNav.stopMotors();
            robotNav.logSample();
            done = true;
            rawL = robotNav.getLeftEncoder();
            rawR = robotNav.getRightEncoder();
            diff = l - r;
            pct = 100.0f * diff / (float)((l + r) / 2);
        }
    } else if (!printed && (bool)Serial) {
        // Xe đã dừng: khi cắm USB in kết quả + log đúng 1 lần
        printed = true;
        delay(300);
        Serial.println("TEST MOTOR BALANCE (open-loop)");
        Serial.printf("PWM = %d (ca 2 banh), trim = 0\n", TEST_PWM);
        Serial.printf("ENC_L = %ld\n", rawL);
        Serial.printf("ENC_R = %ld\n", rawR);
        Serial.printf("DIFF = %ld (|L| - |R|)\n", diff);
        Serial.printf("DIFF_PERCENT = %.2f%%\n", pct);
        Serial.printf("%s quay nhieu hon\n", diff > 0 ? "BANH TRAI" : "BANH PHAI");
        robotNav.printLog();
    }
    commandHandler.update();
    return true;
#else
    return false;
#endif
}

// ================= NÚT BẤM =================

// Nhấn = LOW (chỉ nhận khi xe không đang chạy thẳng). Mỗi lần bấm = 1 lần chạy: phải nhả nút mới nhận lần sau
bool buttonFired() {
    static unsigned long pressedAt = 0;
    static bool armed = true;
    if (!armed && pressedAt == 0 && digitalRead(BUTTON) == HIGH) {
        delay(30);
        if (digitalRead(BUTTON) == HIGH) armed = true;
    }
    if (armed && !robotNav.pidRunActive && pressedAt == 0 && digitalRead(BUTTON) == LOW) {
        delay(30);
        if (digitalRead(BUTTON) == LOW) { pressedAt = millis(); armed = false; }
    }
    if (pressedAt != 0 && millis() - pressedAt >= TURN_DELAY_MS) {
        pressedAt = 0;
        return true;
    }
    return false;
}

// ================= CHẾ ĐỘ SEQ_MODE =================

// Phanh sớm STOP_LEAD_PULSES chỉ dùng trong các chế độ 1, 2, 6
static void withStopLead(void (RobotNav::*fn)()) {
    robotNav.stopLeadPulses = STOP_LEAD_PULSES;
    (robotNav.*fn)();
    robotNav.stopLeadPulses = 0;
}

void runSeqMode() {
#if SEQ_MODE == 1
    withStopLead(&RobotNav::runCellToCellTest);
#elif SEQ_MODE == 2
    withStopLead(&RobotNav::runDeadEndTest);
#elif SEQ_MODE == 3
    robotNav.runCalibTest(4);
#elif SEQ_MODE == 4
    robotNav.runCornerNavTest();
#elif SEQ_MODE == 5
    robotNav.runSensorStreamTest();
#elif SEQ_MODE == 6
    withStopLead(&RobotNav::runCellNavTest);
#elif SEQ_MODE == 7
    kichBan();
#elif SEQ_MODE == 8
    robotNav.stopLeadPulses = STOP_LEAD_PULSES;
    mapper.explore();
    robotNav.stopLeadPulses = 0;
#elif SEQ_MODE == 10
    robotNav.stopLeadPulses = STOP_LEAD_PULSES;
    mapper.explore(true);  // chạy liên tục qua ô thẳng
    robotNav.stopLeadPulses = 0;
#else
    robotNav.turnLeft90Test(TURN_RIGHT ? -TURN_ANGLE : TURN_ANGLE); // góc dương = trái, âm = phải
#endif
}

// ================= IN LOG KHI CẮM USB =================

void printPendingLogs() {
    if (!(bool)Serial) return;
    if (!robotNav.calibPrinted) {
        delay(300);
        Serial.println(robotNav.calibReport);
        robotNav.calibPrinted = true;
    }
    if (robotNav.navReportPending) {
        delay(300);
        Serial.println("==== NHAT KY CHE DO 6 ====");
        Serial.print(robotNav.navReport);
        Serial.println("==== HET NHAT KY ====");
        robotNav.navReportPending = false;
    }
    if (mapper.reportPending) {
        delay(300);
        Serial.println("==== BAN DO TUONG ====");
        Serial.print(mapper.report);
        mapper.reportPending = false;
    }
    if (robotNav.sensorLogPending()) {
        delay(300);
        robotNav.printSensorLog();
    }
    if (robotNav.turnLogPending()) {
        delay(300);
        robotNav.printTurnLog();
    }
    // Log chạy thẳng: in đúng 1 lần sau mỗi lần xe dừng
    static bool printedDone = false;
    if (robotNav.pidRunActive) printedDone = false;
    else if (!printedDone && robotNav.logCount() > 0) {
        printedDone = true;
        delay(300);
        robotNav.printLog();
    }
}
