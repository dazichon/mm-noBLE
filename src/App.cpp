#include <Arduino.h>
#include "App.h"
#include "RobotConfig.h"
#include "RobotNav.h"
#include "CommandHandler.h"
#include "MainParams.h"
#include "Mapping.h"
#include "Motion.h"

// ================= THÔNG SỐ =================

void applyParams() {
    RobotNav &n = robotNav;
    // VL53 trước: bỏ phản xạ yếu (mặt sàn / mép khung trong hình nón nhìn) -> không còn báo tường giả ~85mm
    n.frontSignalLimit = VL53_FRONT_SIGNAL_LIMIT;
    if (n.frontReady && VL53_FRONT_SIGNAL_LIMIT > 0.0f) n.sensorFront.setSignalRateLimit(VL53_FRONT_SIGNAL_LIMIT);
    n.motorScaleL = 1.0f + MOTOR_TRIM_LEFT / 100.0f;
    n.motorScaleR = 1.0f + MOTOR_TRIM_RIGHT / 100.0f;
    if (FORWARD_PWM > 0) n.baseForwardSpeed = FORWARD_PWM;
    n.stopLeadRefPwm = STOP_LEAD_REF_PWM;
    n.accelStartPwm = ACCEL_START_PWM;
    n.accelRampMs = ACCEL_RAMP_MS;

    // Quay tại chỗ
    n.turnMaxPwm = TURN_MAX_PWM;
    n.turnMinPwm = TURN_MIN_PWM;
    n.turnDecelZone = TURN_DECEL_ZONE;
    n.turnTolerance = TURN_TOLERANCE;
    n.turnLeftYawSign = TURN_LEFT_YAW_SIGN;
    n.turnBrakeLead = TURN_BRAKE_LEAD;


    // PID đi thẳng
    n.encKp = ENC_STRAIGHT_KP;
    n.encMaxCorr = ENC_MAX_CORR;
    n.launchMm = LAUNCH_MM;
    n.launchScaleL = 1.0f + LAUNCH_TRIM_LEFT / 100.0f;
    n.launchScaleR = 1.0f + LAUNCH_TRIM_RIGHT / 100.0f;
    n.hold.yawKp = HOLD_YAW_KP;
    n.hold.yawKi = HOLD_YAW_KI;
    n.hold.wallEnable = WALL_ENABLE;
    n.hold.wallKp = WALL_KP;
    n.hold.wallMaxCorr = WALL_MAX_CORR;
    n.straightUseGyro = STRAIGHT_USE_GYRO;
    n.wallKdPerDeg = WALL_KD;
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

    mapper.speedTurnCost = MAP_SPEED_TURN_COST;
    mapper.alignFront = MAP_ALIGN_FRONT;
    mapper.printMap = PRINT_MAP;
    mapper.tieTowardCenter = MAP_TIE_TOWARD_CENTER;
    mapper.presetTrust = MAP_PRESET_TRUST;
    n.frontAlignMm = FRONT_ALIGN_MM;
    n.frontEmergencyMm = FRONT_EMERGENCY_MM;
    n.frontStopEnable = FRONT_STOP_ENABLE;
    n.frontStopMaxMm = FRONT_STOP_MAX_MM;
    n.frontStopMaxExtend = FRONT_STOP_MAX_EXTEND;
    n.frontStopSamples = FRONT_STOP_SAMPLES;
    n.frontStopConsistMm = FRONT_STOP_CONSIST_MM;
    n.frontStopMinMoveMm = FRONT_STOP_MIN_MOVE_MM;
    n.frontAlignTol = FRONT_ALIGN_TOL;
    n.frontAlignSkip = FRONT_ALIGN_SKIP;
    n.frontAlignMaxPwm = 55.0f;
    n.frontAlignMaxMs = 800;
    n.frontAlignKp = FRONT_ALIGN_KP;
    n.frontAlignMinPwm = FRONT_ALIGN_MIN_PWM;
    n.sideWinFrom = MAP_SIDE_FROM;
    n.sideWinTo = MAP_SIDE_TO;
    n.sideEchoEnable = SIDE_ECHO_ENABLE;
    n.sideEchoFrontMax = SIDE_ECHO_FRONT_MAX;
    n.sideEchoRatio = SIDE_ECHO_RATIO;
    n.sideEchoTol = SIDE_ECHO_TOL;
    applyModeParams();
}

// Thông số phụ thuộc chế độ (gọi lại mỗi lần đổi chế độ bằng công tắc DS1)
void applyModeParams() {
    const int m = seqMode;
    const bool slalom = (m == 2 || m == 5 || m == 6 || m == 7 || m == 9 || m == 11);
    mapper.cols = MAP_COLS;
    mapper.rows = MAP_ROWS;
    mapper.startX = MAP_START_X;
    mapper.startY = MAP_START_Y;
    mapper.goalX = MAP_GOAL_X;
    mapper.goalY = MAP_GOAL_Y;
    mapper.goalSize = MAP_GOAL_SIZE;
    mapper.maxCells = MAP_MAX_CELLS;
    mapper.speedrunSpeed = MAP_SPEEDRUN_PWM;  // mọi mode đi thẳng cùng PWM + PID của mode 3; mode cua chỉ dùng PID cua khi cua
    mapper.slalomTurns = slalom;
    mapper.calibBack = MAP_CALIB_BACK && !slalom;  // chạy bằng cua: không lùi kịch tường
    mapper.calibBackErrDeg = 0.0f;                 // mode 1: lùi calib mọi lần có tường sau
    if ((m >= 7 && m <= 11) || m == 2 || m == 5) {   // mọi mode có speedrun (mode cua 2/5/7/9/11 + quay tại chỗ 8/10)                       // map vòng loại / APEC: chỉ lùi calib khi MPU lệch quá SPEED_CALIB_ERR_DEG
        mapper.calibBack = true;
        mapper.calibBackErrDeg = SPEED_CALIB_ERR_DEG;
    }
    mapper.usePreset = (MAP_PRESET || m == 2 || m >= 7) && m != 5 && m != 6;  // mode 2, 7-10 bắt buộc dùng map nạp sẵn
    mapper.startSidesWall = NAV_START_SIDES_WALL;
    mapper.presetQual = (m == 7 || m == 8);
    mapper.presetApec = (m == 9 || m == 10);
    mapper.presetApec2 = (m == 11);
    if (m == 5) {
        // Mode 5: dò map 16x16 (KHÔNG nạp sẵn) như kích thước mode 11 -> dò tới đích, dò về, speedrun bằng cua
        mapper.cols = 16; mapper.rows = 16;
        mapper.startX = MAP_APEC_START_X; mapper.startY = MAP_APEC_START_Y;
        mapper.goalX = MAP_APEC_GOAL_X; mapper.goalY = MAP_APEC_GOAL_Y; mapper.goalSize = MAP_APEC_GOAL_SIZE;
        mapper.maxCells = 400;
    }
    if (mapper.presetQual) {
        // Map vòng loại 16x16 (mode 7: cua, mode 8: quay tại chỗ): xuất phát (0,0) mặt Bắc, đích 2x2 tại (7,7)
        mapper.cols = 16; mapper.rows = 16;
        mapper.startX = 0; mapper.startY = 0;
        mapper.goalX = 7; mapper.goalY = 7; mapper.goalSize = 2;
        mapper.maxCells = 400;
    }
    if (mapper.presetApec || mapper.presetApec2) {
        // Map APEC 16x16 (mode 9: cua, mode 10: quay tại chỗ)
        mapper.cols = 16; mapper.rows = 16;
        mapper.startX = MAP_APEC_START_X; mapper.startY = MAP_APEC_START_Y;
        mapper.goalX = MAP_APEC_GOAL_X; mapper.goalY = MAP_APEC_GOAL_Y; mapper.goalSize = MAP_APEC_GOAL_SIZE;
        mapper.maxCells = 400;
    }
}

// ================= CÔNG TẮC CHỌN CHẾ ĐỘ (DS1) + NÚT KEY2 =================
// DS1 + điện trở tạo 8 mức điện áp trên BW (GPIO0). Mức thật ĐO TRÊN XE ghi vào MODE_LEVELS_MV (MainParams.h),
// không tính theo schematic (đo thực tế khác: tắt hết ~2900 mV). KEY2 kéo BW lên 3.3V -> bấm nút ~3365 mV.
int seqMode = SEQ_MODE;

static uint32_t readBwMv() {
    uint32_t s = 0;
    for (int i = 0; i < 32; i++) s += analogReadMilliVolts(BUTTON);  // ADC nhiễu ~±130 mV -> lấy trung bình nhiều mẫu
    return s / 32;
}

// Vị trí 0..7 = ô trong MODE_LEVELS_MV gần nhất; -1 = đang bấm nút / không khớp mức nào (mức 0 = chưa đo, bỏ qua)
static int switchCodeFromMv(uint32_t mv) {
    if (mv >= MODE_BTN_MIN_MV) return -1;
    static const int LV[8] = MODE_LEVELS_MV;
    int best = -1;
    long bestErr = 1000000;
    for (int c = 0; c < 8; c++) {
        if (LV[c] <= 0) continue;
        long e = labs((long)mv - (long)LV[c]);
        if (e < bestErr) { bestErr = e; best = c; }
    }
    return bestErr <= MODE_SW_TOL_MV ? best : -1;
}

void initModeSwitch() {
#if MODE_SWITCH_ENABLE
    pinMode(BUTTON, INPUT);  // KHÔNG pull-up: điện trở kéo trong làm lệch mức điện áp
    analogSetPinAttenuation(BUTTON, ADC_11db);
    updateModeSwitch(true);
#else
    pinMode(BUTTON, INPUT_PULLUP);
#endif
}

// Đọc DS1 (chỉ khi xe không chạy) -> nhớ vào "chế độ chờ". Gạt công tắc KHÔNG đổi chế độ ngay:
// chỉ khi bấm KEY2 (commitModeSwitch) chế độ chờ mới được kích hoạt. force = true (lúc bật nguồn): kích hoạt luôn.
static int pendingMode = SEQ_MODE, pendingCode = -1;
static uint32_t pendingMv = 0;

void updateModeSwitch(bool force) {
#if MODE_SWITCH_ENABLE
    static unsigned long lastT = 0;
    static int lastCode = -2, stable = 0;
    if (!force && (robotNav.pidRunActive || millis() - lastT < 100)) return;
    lastT = millis();
    uint32_t mv = readBwMv();
    int code = switchCodeFromMv(mv);
    if (force) Serial.printf(">> [MODE] BW = %lu mV\n", (unsigned long)mv);
    static uint32_t lastPrintMv = 0;  // chẩn đoán: in điện áp BW mỗi khi đổi > 40 mV (gạt công tắc / bấm nút)
    if (labs((long)mv - (long)lastPrintMv) > 15) {
        lastPrintMv = mv;
        Serial.printf(">> [MODE] BW = %lu mV -> %s\n", (unsigned long)mv,
                      mv >= MODE_BTN_MIN_MV ? "COI LA DANG BAM NUT" : code < 0 ? "KHONG KHOP MUC NAO" : "khop cong tac");
    }
    if (code < 0) return;  // đang bấm nút hoặc mức lạ: giữ chế độ chờ cũ
    stable = (code == lastCode) ? stable + 1 : 0;
    lastCode = code;
    if (!force && stable < 3) return;  // chống dội khi đang gạt
    static const int TABLE[8] = MODE_SWITCH_TABLE;
    if (code != pendingCode) {
        pendingCode = code;
        pendingMode = TABLE[code];
        pendingMv = mv;
        if (!force) Serial.printf(">> [MODE] VI TRI %d (BW %lu mV) -> CHE DO %d (bam nut de kich hoat)\n", code, (unsigned long)mv, pendingMode);
    }
    if (force) commitModeSwitch();
#else
    (void)force;
#endif
}

// Bấm KEY2: kích hoạt chế độ đang gạt trên DS1
void commitModeSwitch() {
#if MODE_SWITCH_ENABLE
    seqMode = pendingMode;
    applyModeParams();
    if (pendingCode < 0)
        Serial.printf(">> [MODE] KICH HOAT CHE DO %d (CHUA DOC DUOC CONG TAC -> dung SEQ_MODE mac dinh)\n", seqMode);
    else
        Serial.printf(">> [MODE] KICH HOAT CHE DO %d (vi tri %d, BW %lu mV)\n", seqMode, pendingCode, (unsigned long)pendingMv);
#endif
}

static bool buttonDown() {
#if MODE_SWITCH_ENABLE
    return readBwMv() >= MODE_BTN_MIN_MV;  // KEY2 kéo BW lên 3.3V
#else
    return digitalRead(BUTTON) == LOW;
#endif
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

// Nhấn KEY2 (chỉ nhận khi xe không đang chạy thẳng). Mỗi lần bấm = 1 lần chạy: phải nhả nút mới nhận lần sau
bool buttonFired() {
    static unsigned long pressedAt = 0;
    static bool armed = true;
    if (!armed && pressedAt == 0 && !buttonDown()) {
        delay(30);
        if (!buttonDown()) armed = true;
    }
    if (armed && !robotNav.pidRunActive && pressedAt == 0 && buttonDown()) {
        delay(30);
        if (buttonDown()) { pressedAt = millis(); armed = false; }
    }
    if (pressedAt != 0 && millis() - pressedAt >= TURN_DELAY_MS) {
        pressedAt = 0;
        return true;
    }
    return false;
}

// ================= CHẾ ĐỘ SEQ_MODE =================

// Lấy mốc tường hông tại ô xuất phát: xe đặt GIỮA ô (2 bên có tường), đứng yên, đọc VL53 trái/phải
// WALL_CALIB_MS rồi lấy trung bình làm wallBaseL/R. Số đọc lệch quá WALL_CALIB_MAX_DEV so với WALL_BASE_* thì bỏ
// (không có tường / đặt xe lệch) và giữ giá trị mặc định
static void calibWallBase() {
    const uint32_t ms = WALL_CALIB_MS;
    float sumL = 0, sumR = 0; int nL = 0, nR = 0;
    unsigned long lastL = robotNav._tL, lastR = robotNav._tR;
    unsigned long t0 = millis();
    while (millis() - t0 < ms) {
        robotNav.updateSensors();
        if (robotNav._tL != lastL) {
            lastL = robotNav._tL;
            int16_t v = robotNav._rawL;
            if (v > 0 && fabsf(v - WALL_BASE_L) <= WALL_CALIB_MAX_DEV) { sumL += v; nL++; }
        }
        if (robotNav._tR != lastR) {
            lastR = robotNav._tR;
            int16_t v = robotNav._rawR;
            if (v > 0 && fabsf(v - WALL_BASE_R) <= WALL_CALIB_MAX_DEV) { sumR += v; nR++; }
        }
        delay(2);
    }
    const int minN = 6;
    if (nL >= minN) robotNav.hold.wallBaseL = sumL / nL;
    if (nR >= minN) robotNav.hold.wallBaseR = sumR / nR;
    robotNav.navPrintf(">> [WALLCAL] moc tuong L=%.1f R=%.1f (mau %d/%d)%s\n", robotNav.hold.wallBaseL, robotNav.hold.wallBaseR,
                       nL, nR, (nL >= minN && nR >= minN) ? "" : " - THIEU MAU, GIU MAC DINH ben do");
}

void runSeqMode() {
    applyModeParams();
    if (WALL_CALIB_MS > 0 && seqMode != 3 && seqMode != 4) calibWallBase();
    Serial.printf(">> [MODE] CHAY CHE DO %d\n", seqMode);
    robotNav.stopLeadPulses = STOP_LEAD_PULSES;
    switch (seqMode) {
    case 1: mapper.fullRun(); break;                // dò tới đích -> dò về xuất phát -> speedrun (quay tại chỗ)
    case 2: mapper.runKnownMap(); break;            // map 5x5 nạp sẵn, rẽ bằng cua
    case 5: mapper.fullRunSlalom(); break;          // như mode 1 nhưng rẽ bằng cua Slalom
    case 6: mapper.roamSlalom(ROAM_CELLS); break;   // chạy tự do ROAM_CELLS ô
    case 7: case 8:                                 // map vòng loại: 7 cua / 8 quay tại chỗ
    case 9: case 10: mapper.runKnownMap(); break;   // map APEC: 9 cua / 10 quay tại chỗ
    case 11: mapper.runKnownMap(); break;           // map APEC 2: cua (như 9)
    case 4: robotNav.runSlalomTest(); break;        // thử 1 lần cua Slalom
    default: break;                                 // 3: chỉ BLE, bấm nút không làm gì
    }
    robotNav.stopLeadPulses = 0;
}

// ================= IN LOG KHI CẮM USB =================

void printPendingLogs() {
    if (!(bool)Serial) return;
    if (robotNav.flashLogPending) {
        delay(300);
        robotNav.printFlashLog();
    }
    if (!robotNav.calibPrinted) {
        delay(300);
        Serial.println(robotNav.calibReport);
        robotNav.calibPrinted = true;
    }
    if (robotNav.sensorLogPending()) {
        delay(300);
        robotNav.printSensorLog();
    }
    if (robotNav.slalomLogPending()) {
        delay(300);
        robotNav.printSlalomLog();
    }
    if (PRINT_TURN_LOG && robotNav.turnLogPending()) {
        delay(300);
        robotNav.printTurnLog();
    }
    // Log chạy thẳng (dài, ~1500 dòng) in TRƯỚC; nhật ký + bản đồ in SAU CÙNG để luôn nằm cuối terminal,
    // không bị log dài đẩy ra khỏi bộ nhớ cuộn
    static bool printedDone = false;
    bool runLogPrinted = false;
    if (robotNav.pidRunActive) printedDone = false;
    else if (PRINT_RUN_LOG && !printedDone && robotNav.logCount() > 0) {
        printedDone = true;
        delay(300);
        robotNav.printLog();
        runLogPrinted = true;
    }
    if (robotNav.navReportPending || (runLogPrinted && robotNav.navReport.length() > 0)) {
        delay(300);
        Serial.println("==== NHAT KY CHE DO 6 ====");
        Serial.print(robotNav.navReport);
        Serial.println("==== HET NHAT KY ====");
        robotNav.navReportPending = false;
        robotNav.dropFlashLog();  // đã in trực tiếp, không in lại từ flash
    }
    if (mapper.reportPending) {
        delay(300);
        Serial.println("==== BAN DO TUONG ====");
        Serial.print(mapper.report);
        mapper.reportPending = false;
    }
}
