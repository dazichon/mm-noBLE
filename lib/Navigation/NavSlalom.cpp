// CUA SLALOM 90° (MODE 4): encoder = biến tiến độ quỹ đạo, MPU = phản hồi yaw/gyro.
// Yaw mục tiêu theta(s) = tích phân độ cong kappa(s) theo quãng đường s (3 pha: tăng / giữ / giảm).
// omegaCmd = omegaFF + Kp*yawErr + Kd*(omegaFF - omegaĐo) -> tốc độ 2 bánh -> PI theo encoder -> PWM.
// Không đụng PID đi thẳng, không dùng VL53 / tường.
#include "RobotNav.h"
#include "NavInternal.h"
#include "MainParams.h"

namespace {

struct SlalomRec {
  uint32_t t;
  uint8_t ph;            // 0 = chạy thẳng trước cua, 1/2/3 = pha cua, 4 = dồn nốt yaw sau S_TOTAL
  float s, ty, ay, ey;   // mm, độ (dương = về phía rẽ)
  float tw, gw, cw;      // target / đo / lệnh omega (độ/s)
  float tvL, tvR, mvL, mvR;
  int pL, pR;
  long eL, eR;
};
const int SL_MAX = 260;
SlalomRec slRec[SL_MAX];
String slText;               // log của TẤT CẢ các lần cua trong lần chạy (dạng text, in ra khi cắm USB)
int slTurnNo = 0;            // số thứ tự lần cua trong lần chạy
bool slPending = false;      // true: log chưa in ra Serial

// PWM có dấu: dương = tiến, âm = chiều ngược lại (hãm bánh: đảo chiều có điều khiển để bánh GIẢM TỐC nhanh, không chỉ thả trôi)
static void driveWheel(uint8_t in1, uint8_t in2, int p) {
  if (p >= 0) { analogWrite(in1, constrain(p, 0, 255)); analogWrite(in2, 0); }
  else        { analogWrite(in1, 0); analogWrite(in2, constrain(-p, 0, 255)); }
}
void driveFwdRaw(int l, int r) {
  driveWheel(M1_IN1, M1_IN2, l);
  driveWheel(M2_IN1, M2_IN2, r);
}

// Hình học quỹ đạo (mm, rad)
struct Geo { float R, D, ang, S1, S2, S3, Stot; };

// Độ cong mục tiêu (1/mm) theo quãng đường s
float calculateTargetCurvature(const Geo &g, float s) {
  if (s <= 0.0f) return 0.0f;
  if (s < g.S1) return (s / g.S1) / g.R;
  if (s < g.S1 + g.S2) return 1.0f / g.R;
  if (s < g.Stot) return ((g.Stot - s) / g.S3) / g.R;
  return 0.0f;
}

// Yaw mục tiêu (rad) = tích phân độ cong theo s; liên tục giữa các pha; = ang tại Stot
float calculateTargetYaw(const Geo &g, float s) {
  if (s <= 0.0f) return 0.0f;
  if (s < g.S1) return s * s / (2.0f * g.S1 * g.R);
  float th2 = g.S1 / (2.0f * g.R) + g.S2 / g.R;      // yaw cuối pha 2
  if (s < g.S1 + g.S2) return g.S1 / (2.0f * g.R) + (s - g.S1) / g.R;
  if (s < g.Stot) {
    float u = s - g.S1 - g.S2;                       // quãng đã đi trong pha 3
    return th2 + (u - u * u / (2.0f * g.S3)) / g.R;
  }
  return g.ang;
}

// Điều khiển tốc độ 1 bánh: PWM = feedforward(v) + PI theo tốc độ đo từ encoder
struct WheelPI {
  float integ = 0.0f, meas = 0.0f;
  int update(float vt, float dtS) {
    float ff = (vt > 1.0f) ? SLALOM_PWM_DEAD + SLALOM_PWM_PER_MMS * vt : 0.0f;
    float e = vt - meas;
    if (vt <= 1.0f && meas <= 8.0f) { integ = 0.0f; return 0; }   // đã dừng và không cần chạy: không đảo chiều
    float u = ff + SLALOM_WHEEL_KP * e + integ;
    bool sat = (u > SLALOM_PWM_MAX && e > 0) || (u < -SLALOM_PWM_REV_MAX && e < 0);
    if (!sat) integ = constrain(integ + SLALOM_WHEEL_KI * e * dtS, -110.0f, 110.0f);  // anti-windup
    if (vt <= 1.0f) integ = min(integ, 0.0f);   // cần giảm tốc về 0: bỏ phần dư tăng ga
    u = ff + SLALOM_WHEEL_KP * e + integ;
    return (int)lroundf(constrain(u, -(float)SLALOM_PWM_REV_MAX, (float)SLALOM_PWM_MAX));   // âm = hãm bằng đảo chiều
  }
};

}  // namespace

// Quãng xe tự tiến theo hướng cũ (= quãng dịch sang ngang) của một cua: tích phân cos(yaw mục tiêu) theo s.
// Dùng để rút ngắn đoạn thẳng trước cua (xe bắt đầu cua sớm chừng này) và đoạn thẳng sau cua.
float RobotNav::slalomAdvanceMm() {
  Geo g;
  g.R = SLALOM_RADIUS_MM; g.D = SLALOM_DELTA_MM; g.ang = SLALOM_ANGLE_DEG * DEG_TO_RAD;
  g.S1 = g.D; g.S3 = g.D; g.S2 = g.R * g.ang - g.D; g.Stot = g.R * g.ang + g.D;
  const int N = 400;
  float f = 0.0f;
  for (int i = 0; i < N; i++) f += cosf(calculateTargetYaw(g, (i + 0.5f) * g.Stot / N));
  return f * g.Stot / N;
}

// MODE 4: cua một lần từ đứng yên (calib MPU, yaw về 0), rồi dừng + lưu log
void RobotNav::runSlalomTest() {
  if (!mpuReady) { Serial.println("# SLALOM ABORT: MPU6050 chua san sang"); return; }
  if (i2cFault) { Serial.println("# SLALOM ABORT: loi bus I2C"); return; }
  slalomLogReset();
  stopPID();
  stopMotors();
  mpu6050.setAutoBias(false);
  mpu6050.calibrate(400);   // xe đang đứng yên: calib offset gyro, yaw về 0
  mpu6050.resetYaw();
  mpu6050.update();
  slalomTurn(SLALOM_TURN_LEFT, false);
  saveRunLog();
}

// Một lần cua Slalom. moving = false: xe đứng yên, cua xong phanh dừng.
// moving = true (speedrun mode 2): xe đang chạy thẳng không phanh; cua xong KHÔNG dừng, để lại hướng chuẩn + PWM cho đoạn thẳng kế tiếp.
// Trả về true nếu cua đủ góc.
bool RobotNav::slalomTurn(bool left, bool moving) {
  float v = SLALOM_SPEED_MM_S;   // chạy liên tục: sẽ lấy theo tốc độ thật lúc vào cua
  const float B = SLALOM_WHEEL_TRACK_MM;
  const float dir = left ? 1.0f : -1.0f;
  Geo g;
  // Bù lệch góc theo chiều cua (log: cua phải vượt ~1 độ, cua trái hụt ~0.3 độ). Hướng chuẩn sau cua vẫn là đúng SLALOM_ANGLE_DEG
  g.R = SLALOM_RADIUS_MM; g.D = SLALOM_DELTA_MM; g.ang = (SLALOM_ANGLE_DEG + (left ? SLALOM_ANGLE_TRIM_LEFT_DEG : SLALOM_ANGLE_TRIM_RIGHT_DEG)) * DEG_TO_RAD;
  g.S1 = g.D; g.S3 = g.D; g.S2 = g.R * g.ang - g.D; g.Stot = g.R * g.ang + g.D;

  if (!mpuReady || i2cFault || v <= 1.0f || g.R <= 1.0f || g.S2 <= 0.0f || pulsesPerCell <= 0) {
    Serial.println("# SLALOM ABORT: MPU/I2C/thong so khong hop le");
    stopPID();
    return false;
  }
  const float mmPerPulse = cellMm / (float)pulsesPerCell;
  char slHeader[220];
  snprintf(slHeader, sizeof(slHeader), "# v=%.0f R=%.0f D=%.0f ang=%.0f B=%.0f | S1=%.1f S2=%.1f Stot=%.1f mm | %.4f mm/xung | Kp=%.2f Kd=%.2f",
                v, g.R, g.D, SLALOM_ANGLE_DEG, B, g.S1, g.S2, g.Stot, mmPerPulse, SLALOM_KP_YAW, SLALOM_KD_YAW);
  _carryValid = false; stepCarry = 0;

  mpu6050.update();
  const float yawSign = dir * turnLeftYawSign;   // nhân với (yaw - yaw0) và gyro => dương về phía rẽ
  // Chạy liên tục: lấy hướng chuẩn của đoạn thẳng vừa chạy làm mốc (không cộng dồn sai số các cua trước)
  // Đi thẳng không gyro: tường đã nắn xe thẳng -> cua tính từ hướng THỰC lúc bắt đầu cua (hướng chuẩn cũ không còn được giữ)
  const float yaw0 = (moving && _turnHeadingValid && straightUseGyro) ? _turnHeading : mpu6050.getYaw();
  _yawFresh = false;   // cua xong: yaw không còn là hướng chuẩn chắc chắn
  const long e0L = getLeftEncoder(), e0R = getRightEncoder();
  long lastL = e0L, lastR = e0R, baseL = e0L, baseR = e0R;  // base: mốc encoder lúc bắt đầu cua
  WheelPI piL, piR;
  float vCmd = 0.0f, s = 0.0f;
  if (moving) {   // xe đang chạy: đo tốc độ thật 2 bánh trong ~20ms (giữ nguyên PWM) để bắt đầu từ đúng tốc độ đó
    unsigned long t0 = millis();
    long a0L = getLeftEncoder(), a0R = getRightEncoder();
    while (millis() - t0 < 20) { mpu6050.update(); delay(1); }
    float dtm = (millis() - t0) * 1e-3f;
    piL.meas = (getLeftEncoder() - a0L) * SLALOM_ENC_SIGN * mmPerPulse / dtm;
    piR.meas = (getRightEncoder() - a0R) * SLALOM_ENC_SIGN * mmPerPulse / dtm;
    vCmd = max(0.0f, 0.5f * (piL.meas + piR.meas));
    lastL = getLeftEncoder(); lastR = getRightEncoder();   // mốc encoder cho vòng đầu (nếu không, vòng đầu chia 20ms xung cho dt 5ms)
    v = constrain(vCmd, v, SLALOM_RUN_VMAX_MM_S);   // cua theo tốc độ đang chạy (bánh ngoài ~1.8 x v phải nằm trong khả năng motor); nhanh hơn mức này thì hãm dần
  }
  int nRec = 0, maxPwmL = 0, maxPwmR = 0;
  float maxYawErr = 0.0f, ayDeg = 0.0f, eyDeg = 0.0f, tyDeg = 0.0f;
  const char *reason = "TIMEOUT";
  const unsigned long tStart = millis();
  unsigned long tLog = 0, tCurve = 0;
  const unsigned long preEnd = tStart + SLALOM_PRE_STRAIGHT_MS;
  uint32_t nextUs = micros() + SLALOM_LOOP_US, prevUs = micros();

  while (true) {
    while ((int32_t)(micros() - nextUs) < 0) { }   // chu kỳ cố định
    uint32_t nowUs = micros();
    nextUs += SLALOM_LOOP_US;
    float dt = (nowUs - prevUs) * 1e-6f;
    if (dt <= 0.0f || dt > 0.05f) dt = SLALOM_LOOP_US * 1e-6f;
    prevUs = nowUs;
    unsigned long nowMs = millis();

    mpu6050.update();
    if (mpu6050.readFails() > 10) { reason = "MPU_FAIL"; break; }
    long cl = getLeftEncoder(), cr = getRightEncoder();
    piL.meas += 0.4f * (((cl - lastL) * SLALOM_ENC_SIGN * mmPerPulse / dt) - piL.meas);   // mm/s, lọc nhẹ
    piR.meas += 0.4f * (((cr - lastR) * SLALOM_ENC_SIGN * mmPerPulse / dt) - piR.meas);
    lastL = cl; lastR = cr;
    // Giữ bộ lọc tường hông luôn mới: sau cua (~1s không đọc) bộ lọc trung vị 5 mẫu còn toàn số cũ của hành lang trước -> bám tường sai ở đoạn thẳng ngắn sau cua
    updateSensors();
    updateWallFilter();

    const bool pre = nowMs < preEnd;
    if (pre) { baseL = cl; baseR = cr; }
    else if (!tCurve) tCurve = nowMs;
    s = (((cl - baseL) + (cr - baseR)) * 0.5f) * SLALOM_ENC_SIGN * mmPerPulse;   // quãng đường trung bình có dấu từ lúc bắt đầu cua

    ayDeg = yawSign * wrap180(mpu6050.getYaw() - yaw0);
    float gyroDps = yawSign * mpu6050.getGyroRate();
    float aYaw = ayDeg * DEG_TO_RAD, aOmega = gyroDps * DEG_TO_RAD;

    vCmd += constrain(v - vCmd, -SLALOM_ACCEL_MM_S2 * dt, SLALOM_ACCEL_MM_S2 * dt);   // tiến dần tới tốc độ cua (tăng từ đứng yên / giảm từ tốc độ chạy thẳng)

    float kappa = pre ? 0.0f : calculateTargetCurvature(g, s);
    // Feedforward nhìn trước SLALOM_LOOKAHEAD_MM: bù độ trễ của motor/quán tính (yaw luôn đi sau quỹ đạo vài độ), nên cho độ cong tăng/giảm sớm hơn một chút
    float kappaFF = pre ? 0.0f : calculateTargetCurvature(g, s + SLALOM_LOOKAHEAD_MM);
    float tYaw = pre ? 0.0f : calculateTargetYaw(g, s);
    float yawErr = tYaw - aYaw;
    // Yaw trễ quỹ đạo -> GIẢM tốc độ tiến (không đẩy bánh ngoài lên): quỹ đạo theo quãng đường nên đi chậm lại thì tốc độ quay cần cũng nhỏ lại, bám kịp
    float vEff = vCmd * constrain(1.0f - max(0.0f, (float)(yawErr * RAD_TO_DEG)) / SLALOM_LAG_SLOW_DEG, SLALOM_LAG_MIN_SCALE, 1.0f);
    float omegaFF = vEff * kappaFF;
    float omegaCmd = omegaFF + SLALOM_KP_YAW * yawErr + SLALOM_KD_YAW * (omegaFF - aOmega);
    omegaCmd = constrain(omegaCmd, -SLALOM_OMEGA_MAX, SLALOM_OMEGA_MAX);

    // omega dương = về phía rẽ: bánh phía trong chậm, bánh ngoài nhanh (trái: dir=+1 -> bánh trái chậm)
    float tvL = max(0.0f, vEff - dir * omegaCmd * B * 0.5f);
    float tvR = max(0.0f, vEff + dir * omegaCmd * B * 0.5f);
    int pwmL = piL.update(tvL, dt);
    int pwmR = piR.update(tvR, dt);
    driveFwdRaw(pwmL, pwmR);
    currentLeftSpeed = max(pwmL, 0); currentRightSpeed = max(pwmR, 0);
    maxPwmL = max(maxPwmL, pwmL); maxPwmR = max(maxPwmR, pwmR);

    uint8_t ph = pre ? 0 : (s < g.S1 ? 1 : (s < g.S1 + g.S2 ? 2 : (s < g.Stot ? 3 : 4)));
    tyDeg = tYaw * RAD_TO_DEG; eyDeg = yawErr * RAD_TO_DEG;
    if (!pre) maxYawErr = max(maxYawErr, fabsf(eyDeg));

    if (nowMs - tLog >= SLALOM_LOG_MS && nRec < SL_MAX) {
      tLog = nowMs;
      slRec[nRec++] = {(uint32_t)(nowMs - tStart), ph, s, tyDeg, ayDeg, eyDeg, (float)(omegaFF * RAD_TO_DEG), gyroDps,
                       (float)(omegaCmd * RAD_TO_DEG), tvL, tvR, piL.meas, piR.meas, pwmL, pwmR, cl - e0L, cr - e0R};
    }

    // --- điều kiện kết thúc (không chỉ theo thời gian)
    if (!pre) {
      if (nowMs - tCurve > 400 && s < 5.0f) { reason = "ENCODER_DIR_OR_STALL"; break; }  // 0.4s mà chưa tiến: sai chiều encoder hoặc xe kẹt
      if (!moving && fabsf(eyDeg) > SLALOM_ABORT_YAW_DEG) { reason = "YAW_RUNAWAY"; break; }   // chạy liên tục: KHÔNG hủy cua giữa chừng (xe đang lệch hướng thì càng phải quay cho đủ)
      if (s >= g.Stot) {
        if (fabsf(eyDeg) <= (moving ? SLALOM_RUN_YAW_TOL_DEG : SLALOM_YAW_TOL_DEG) && fabsf(gyroDps) <= (moving ? SLALOM_RUN_GYRO_TOL_DPS : SLALOM_GYRO_TOL_DPS)) { reason = "SUCCESS"; break; }
        if (s >= g.Stot + (moving ? SLALOM_RUN_EXTRA_MM : SLALOM_EXTRA_MM)) { reason = "DISTANCE_LIMIT"; break; }
      }
    }
    if (nowMs - tStart > (unsigned long)(SLALOM_TIMEOUT_MS + SLALOM_PRE_STRAIGHT_MS)) { reason = "TIMEOUT"; break; }
  }

  const bool success = strcmp(reason, "SUCCESS") == 0;
  // Chạy liên tục: KHÔNG dừng dù cua chưa đạt chuẩn (DISTANCE_LIMIT, TIMEOUT...): đoạn thẳng sau tự kéo hướng về hướng chuẩn bằng gyro. Chỉ dừng khi mất MPU
  const bool soft = moving && strcmp(reason, "MPU_FAIL") != 0;
  const bool handOver = soft;
  unsigned long durMs = millis() - (tCurve ? tCurve : tStart);
  float yawAtStop = yawSign * wrap180(mpu6050.getYaw() - yaw0);
  long eL = getLeftEncoder() - e0L, eR = getRightEncoder() - e0R;
  float yawFinal = yawAtStop;
  if (handOver) {
    _absHeading += yawSign * SLALOM_ANGLE_DEG;          // hướng chuẩn tuyệt đối: +-90 lý tưởng (không theo yaw thực)
    _turnHeading = yaw0 + yawSign * SLALOM_ANGLE_DEG;   // hướng chuẩn mới (lý tưởng, không lấy yaw thực để khỏi cộng dồn sai số)
    _turnHeadingValid = true;
    _targetYaw = _turnHeading;
  } else {
    brakeMotors();
    unsigned long sw = millis();
    while (millis() - sw < 400) { mpu6050.update(); delay(2); }   // chờ xe đứng hẳn rồi đo yaw lần nữa
    stopMotors();
    currentLeftSpeed = currentRightSpeed = 0;
    mpu6050.setAutoBias(true);
    yawFinal = yawSign * wrap180(mpu6050.getYaw() - yaw0);
  }

  char slSummary[720];
  int n = 0;
  n += snprintf(slSummary + n, sizeof(slSummary) - n, "# SLALOM RESULT\n");
  n += snprintf(slSummary + n, sizeof(slSummary) - n, "Target Angle: %.2f\nActual Angle: %.2f (luc dung) / %.2f (sau 400ms)\nAngle Error: %.2f\n",
                (float)SLALOM_ANGLE_DEG, yawAtStop, yawFinal, SLALOM_ANGLE_DEG - yawFinal);
  n += snprintf(slSummary + n, sizeof(slSummary) - n, "Target Distance: %.1f\nActual Distance: %.1f\nTurn Duration: %lu ms\n", g.Stot, s, durMs);
  n += snprintf(slSummary + n, sizeof(slSummary) - n, "Encoder Left: %ld\nEncoder Right: %ld\nMax Yaw Error: %.2f\nMax PWM Left: %d\nMax PWM Right: %d\n",
                eL, eR, maxYawErr, maxPwmL, maxPwmR);
  snprintf(slSummary + n, sizeof(slSummary) - n, "Stop Reason: %s\n", reason);
  // Ghi vào log text của lần chạy (giữ TẤT CẢ các lần cua; in ra khi cắm USB, và được lưu vào flash khi kết thúc lần chạy)
  if (slText.length() < 36000) {
    char row[160];
    slTurnNo++;
    snprintf(row, sizeof(row), "# ===== CUA #%d (%s)%s =====\n", slTurnNo, left ? "TRAI" : "PHAI", moving ? " chay lien tuc" : "");
    slText += row;
    slText += slHeader;
    slText += "\ntime_ms,phase,distance_mm,target_yaw,actual_yaw,yaw_error,target_omega,actual_omega,omega_command,target_vL,target_vR,meas_vL,meas_vR,pwm_L,pwm_R,encoder_L,encoder_R\n";
    for (int i = 0; i < nRec; i++) {
      const SlalomRec &r = slRec[i];
      snprintf(row, sizeof(row), "%lu,%u,%.1f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%.0f,%.0f,%.0f,%.0f,%d,%d,%ld,%ld\n", (unsigned long)r.t, r.ph, r.s,
               r.ty, r.ay, r.ey, r.tw, r.gw, r.cw, r.tvL, r.tvR, r.mvL, r.mvR, r.pL, r.pR, r.eL, r.eR);
      slText += row;
    }
    slText += slSummary;
  }
  slPending = true;
  if ((bool)Serial && !handOver) printSlalomLog();   // đang chạy liên tục thì không in (mất thời gian), log lưu lại in sau
  return success || soft;
}

// Đi thẳng distMm với tốc độ điều khiển bằng encoder (PI từng bánh, SLALOM_SPEED_MM_S) + giữ hướng _turnHeading bằng yaw/gyro MPU.
// moving: xe đang chạy (đo tốc độ thật để bắt đầu). handOver: hết quãng KHÔNG phanh, giữ nguyên PWM để vào cua Slalom ngay.
// VL53 hông bám tường (dịch hướng đích gyro như PID chạy thẳng). frontWall + handOver: VL53 trước chốt lại độ dài ô - vào cua khi đọc <= frontAlignMm + quãng xe cua sớm;
// encoder báo đủ mà tường còn xa thì đi tiếp (tối đa frontStopMaxExtend ô). Không ghi đè bản đồ.
// Mode 5: xe lệch ngang về phía bên kia thì mắt bên này đọc tường XA hơn bình thường (dễ vượt ngưỡng THÔNG).
// Bên kia đang thấy tường gần hơn mức chuẩn bao nhiêu thì nâng ngưỡng THÔNG bên này lên bấy nhiêu.
static int sideDriftMm(int otherRaw, int otherNom) {
  if (otherRaw <= 0 || otherRaw >= NAV_SIDE_WALL_MM) return 0;   // bên kia thông / không đọc được: không bù
  return max(0, otherNom - otherRaw);
}

bool RobotNav::slalomStraight(float distMm, bool moving, bool handOver, bool frontWall) {
  const float v = SLALOM_SPEED_MM_S, B = SLALOM_WHEEL_TRACK_MM;
  if (!mpuReady || i2cFault || pulsesPerCell <= 0 || distMm <= 0.0f) { stopPID(); return distMm <= 0.0f; }
  const float mmPerPulse = cellMm / (float)pulsesPerCell;
  _carryValid = false; stepCarry = 0;
  mpu6050.setAutoBias(false);
  mpu6050.update();
  if (!_turnHeadingValid) { _turnHeading = mpu6050.getYaw(); _turnHeadingValid = true; }
  _targetYaw = _turnHeading;

  WheelPI piL, piR;
  float vCmd = 0.0f, s = 0.0f;
  long l0 = getLeftEncoder(), r0 = getRightEncoder(), lastL = l0, lastR = r0;
  if (moving) {   // đang chạy: đo tốc độ thật ~20ms (giữ nguyên PWM) rồi bắt đầu từ đó
    unsigned long t0 = millis();
    while (millis() - t0 < 20) { mpu6050.update(); delay(1); }
    float dtm = (millis() - t0) * 1e-3f;
    piL.meas = (getLeftEncoder() - l0) * SLALOM_ENC_SIGN * mmPerPulse / dtm;
    piR.meas = (getRightEncoder() - r0) * SLALOM_ENC_SIGN * mmPerPulse / dtm;
    vCmd = max(0.0f, 0.5f * (piL.meas + piR.meas));
    lastL = getLeftEncoder(); lastR = getRightEncoder();
  }
  slFront0 = (int)_rawF;
  const float vStart = max(vCmd, 40.0f);
  // PID đi thẳng (dùng chung mode 3): khâu I yaw riêng cho đoạn; tăng tốc từ PWM đang chạy (vừa cua xong) hoặc accelStartPwm
  float yawI = 0.0f;
  wallAngleReset();
  _encOff = 0.0f;
  _launchActive = !moving && launchMm > 0.0f;   // đoạn thẳng xuất phát từ đứng yên / sau lùi calib
  bool yawResetDone = !moving;
  // Lịch sử sai lệch tường theo quãng đường (mỗi 10 mm 1 mẫu) để biết xe có đang đi SONG SONG tường không
  float wallHist[8] = {0};
  int wallHistN = 0;
  float wallHistS = -1000.0f;   // vừa cua xong: đi được SLALOM_YAW_RESET_MM thì reset yaw = 0 (hướng hiện tại thành hướng chuẩn)
  // Vết đoạn thẳng (in sau khi hết đoạn, không in trong vòng lặp): mỗi ~60ms "s:loiYaw/gyro/tuong/L/R/F/pid/pwmL/pwmR/v"
  String trace;
  trace.reserve(1600);
  unsigned long tTrace = 0;
  const float rampFrom = moving ? max(accelStartPwm, 0.5f * (float)(currentLeftSpeed + currentRightSpeed)) : accelStartPwm;   // tốc độ vào đoạn (từ đứng yên thì 40 mm/s để PI có PWM > vùng chết)
  const float vExit = handOver ? v : SLALOM_RUN_STOP_SPEED_MM_S;   // tốc độ cuối đoạn: vào cua Slalom, hoặc trước khi phanh dừng
  // VL53 trước đọc bao nhiêu thì kết thúc đoạn: vào cua = frontAlignMm + quãng xe cua sớm; đoạn cuối (phanh dừng) = frontAlignMm + quãng trôi
  const float trigMm = frontAlignMm + (handOver ? slalomAdvanceMm() : SLALOM_STOP_LEAD_MM) + SLALOM_FRONT_TRIG_BIAS_MM;
  const char *endBy = "ENCODER";
  // Chế độ DÒ (slExplore): chưa biết tường trước -> VL53 trước vừa phát hiện tường vừa chốt điểm vào cua; đồng thời đếm mẫu vách hông (cửa sổ theo vị trí slPStartMm + s)
  const bool chkFront = frontWall || slExplore;
  if (slExplore) {
    sideNearL = sideFarL = sideNearR = sideFarR = 0; sideEchoL = sideEchoR = 0; _openRunL = _openRunR = 0; _rMin = 9999; _rMax = 0; _rN = 0;
    _sideLastTL = _tL; _sideLastTR = _tR;
    slSideTrace = "";
    _sideTraceTL = _tL;
  }
  const float extendMm = max(frontStopMaxExtend * cellMm, slExtendMm);
  uint8_t trigN = 0;
  unsigned long lastTF = _tF;
  // Mép tường hông bên sắp rẽ
  bool edgeSawWall = false;
  int edgeOpenN = 0;        // số mẫu TRỐNG liên tiếp (1 mẫu lẻ dễ là nhiễu khi xe lệch xa tường)
  float edgeOpenS = 0.0f;   // vị trí mẫu TRỐNG đầu tiên
  slEdgeAtMm = -1.0f;
  unsigned long lastTE = slEdgeSide == 1 ? _tL : _tR;
  unsigned long lastCoverTF = _tF;
  uint8_t coverN = 0;
  frontCovered = false;
  const char *reason = "TIMEOUT";
  const unsigned long tStart = millis();
  const unsigned long tLimit = 2500 + (unsigned long)(distMm / v * 1000.0f * 3.0f);
  uint32_t nextUs = micros() + SLALOM_LOOP_US, prevUs = micros();
  while (true) {
    while ((int32_t)(micros() - nextUs) < 0) { }
    uint32_t nowUs = micros();
    nextUs += SLALOM_LOOP_US;
    float dt = (nowUs - prevUs) * 1e-6f;
    if (dt <= 0.0f || dt > 0.05f) dt = SLALOM_LOOP_US * 1e-6f;
    prevUs = nowUs;
    unsigned long nowMs = millis();
    mpu6050.update();
    if (mpu6050.readFails() > 10) { reason = "MPU_FAIL"; break; }
    long cl = getLeftEncoder(), cr = getRightEncoder();
    piL.meas += 0.4f * (((cl - lastL) * SLALOM_ENC_SIGN * mmPerPulse / dt) - piL.meas);
    piR.meas += 0.4f * (((cr - lastR) * SLALOM_ENC_SIGN * mmPerPulse / dt) - piR.meas);
    lastL = cl; lastR = cr;
    s = (((cl - l0) + (cr - r0)) * 0.5f) * SLALOM_ENC_SIGN * mmPerPulse;
    updateSensors();      // VL53 (không chặn, ~35ms/lần)
    updateWallFilter();
    if (i2cFault) { reason = "I2C_FAULT"; break; }
    // Mép tường hông: thấy tường rồi thấy TRỐNG (2 mẫu liên tiếp, hoặc 1 mẫu rất xa) -> điểm cua = vị trí hiện tại + SLALOM_EDGE_TURN_MM
    if (slEdgeSide && slEdgeAtMm < 0.0f) {
      unsigned long tE = slEdgeSide == 1 ? _tL : _tR;
      if (tE != lastTE) {
        lastTE = tE;
        int16_t raw = slEdgeSide == 1 ? _rawL : _rawR;
        int openMm = slEdgeSide == 1 ? SLALOM_EXPLORE_OPEN_MM_L : SLALOM_EXPLORE_OPEN_MM_R;
        bool isOpen = raw == -3 || raw >= openMm;
        bool isWall = raw >= 0 && raw < navSideWallMm && !sideIsFrontEcho(raw);
        bool strongOpen = raw == -3 || raw >= SLALOM_EDGE_OPEN_STRONG_MM;
        if (isWall) { edgeSawWall = true; edgeOpenN = 0; }
        else if (isOpen && edgeSawWall && s >= slEdgeMinS) {
          if (edgeOpenN == 0) edgeOpenS = s;
          edgeOpenN++;
        } else edgeOpenN = 0;
        // Chốt mép: 1 mẫu đọc rất xa (chắc chắn trống) hoặc SLALOM_EDGE_OPEN_N mẫu trống liên tiếp
        if (edgeOpenN > 0 && (strongOpen || edgeOpenN >= SLALOM_EDGE_OPEN_N)) {
          float sEdge = edgeOpenS;
          slEdgeAtMm = sEdge;
          float newDist = sEdge + SLALOM_EDGE_TURN_MM;
          navPrintf(">> [MEP] %s TRONG tai s=%.0f mm (raw %d) -> cua sau %.0f mm (encoder du kien %.0f mm, lech %+.0f)\n",
                    slEdgeSide == 1 ? "TRAI" : "PHAI", sEdge, (int)raw, (float)SLALOM_EDGE_TURN_MM, distMm, newDist - distMm);
          distMm = newDist;
        }
      }
    }
    if (slExplore) {
      float p = slPStartMm + s;   // mm tính từ tâm ô trước (cùng quy ước cửa sổ sideWinFrom/To của chạy thẳng)
      // Ghi vết từng mẫu hông (vị trí p, VL53 trái/phải) để đối chiếu với bản đồ thật
      if (_tL != _sideTraceTL && slSideTrace.length() < 900) {
        _sideTraceTL = _tL;
        char tb[24];
        snprintf(tb, sizeof(tb), " %d:%d/%d", (int)p, (int)_rawL, (int)_rawR);
        slSideTrace += tb;
      }
      // "Thông" tính ở BẤT KỲ lúc nào trong đoạn (1 mẫu là đủ, không dùng trung vị / đa số); "có tường" chỉ tính trong cửa sổ p
      const bool inWin = p >= SLALOM_EXPLORE_SIDE_FROM_MM && p <= SLALOM_EXPLORE_SIDE_TO_MM;
      if (_tL != _sideLastTL) {
        _sideLastTL = _tL;
        if (sideIsFrontEcho(_rawL)) { if (sideEchoL < 255) sideEchoL++; }
        else if (_rawL >= 0 && _rawL < navSideWallMm) { if (inWin && sideNearL < 255) sideNearL++; }
        if (!sideIsFrontEcho(_rawL)) {
          // "Thông" CHỈ tính trong cửa sổ p (ngoài cửa sổ tia chéo còn chiếu vào chỗ hở/vách của ô trước) và phải LIÊN TIẾP
          if (inWin && (_rawL >= SLALOM_EXPLORE_OPEN_MM_L + sideDriftMm(_rawR, SLALOM_EXPLORE_R_NOM_MM) || _rawL == -3)) { if (++_openRunL >= SLALOM_EXPLORE_OPEN_MIN_SAMPLES && sideFarL < 255) sideFarL++; }
          else _openRunL = 0;
        }
      }
      if (_tR != _sideLastTR) {
        _sideLastTR = _tR;
        if (sideIsFrontEcho(_rawR)) { if (sideEchoR < 255) sideEchoR++; }
        else if (_rawR >= 0 && _rawR < navSideWallMm) { if (inWin && sideNearR < 255) sideNearR++; }
        if (!sideIsFrontEcho(_rawR)) {
          if (inWin && _rawR >= 0) {   // độ phân tán: tường đọc ổn định, thông đọc nhảy mạnh (log: tường chạy <= ~43 mm, thông >= ~65 mm)
            if (_rawR < _rMin) _rMin = _rawR;
            if (_rawR > _rMax) _rMax = _rawR;
            if (_rN < 255) _rN++;
            if (_rN >= SLALOM_EXPLORE_R_SPREAD_MIN_N && _rMax - _rMin >= SLALOM_EXPLORE_R_SPREAD_MM && sideFarR < 1) sideFarR = 1;
          }
          if (inWin && (_rawR >= SLALOM_EXPLORE_OPEN_MM_R + sideDriftMm(_rawL, SLALOM_EXPLORE_L_NOM_MM) || _rawR == -3)) { if (++_openRunR >= SLALOM_EXPLORE_OPEN_MIN_SAMPLES_R && sideFarR < 255) sideFarR++; }
          else _openRunR = 0;
        }
      }
    }
    // Che kín mắt trước bằng tay: đọc ~0 liên tiếp -> dừng khẩn cấp toàn bộ lần chạy
    if (frontGuard && frontReady && _tF != lastCoverTF) {
      lastCoverTF = _tF;
      if (_rawF >= 0 && _rawF <= SLALOM_FRONT_COVER_MM) coverN++; else coverN = 0;
      if (coverN >= SLALOM_FRONT_COVER_N) { frontCovered = true; reason = "FRONT_COVERED"; break; }
    }
    // Phanh khẩn: quá sát tường trước (chỉ khi mapper bật frontGuard)
    if (frontGuard && frontReady && _rawF > 20 && smoothDF <= frontEmergencyMm) { frontEmergency = true; reason = "FRONT_EMERGENCY"; break; }
    // VL53 trước chốt vị trí trong ô (không tin xung tuyệt đối): có tường trước đã biết thì điểm kết thúc đoạn = lúc VL53 đọc <= trigMm,
    // dù encoder chưa đủ xung (xe đi quá) hoặc đã quá xung (xe chưa tới)
    if (chkFront && frontReady && _tF != lastTF) {   // mẫu VL53 trước mới
      lastTF = _tF;
      bool valid = _rawF > 20 && (float)_rawF <= frontStopMaxMm;
      bool early = valid && (float)_rawF <= trigMm - 12.0f;   // tường gần hơn dự kiến rõ rệt: xe đã quá điểm -> kết thúc ngay
      if (valid && (float)_rawF <= trigMm && (early || s >= max(distMm - 0.6f * cellMm, 0.5f * distMm))) trigN++;   // bình thường: đi ít nhất nửa đoạn để căn lại
      else trigN = 0;
    }
    if (chkFront && trigN >= 2) { reason = "SUCCESS"; endBy = "TUONG"; break; }   // tường trước tới điểm: kết thúc đoạn
    // Đang chờ mép tường hông mà encoder đã đủ: đi tiếp tối đa nửa ô nữa chờ mép (encoder đếm dư là nguyên nhân cua sớm đâm tường)
    const bool waitEdge = slEdgeSide && slEdgeAtMm < 0.0f && s < distMm + 0.15f * distMm + 0.5f * cellMm;  // encoder đếm dư ~10%
    if (s >= distMm && !waitEdge) {
      // Encoder đủ mà tường trước còn xa: xe chưa tới (encoder đếm dư) -> đi tiếp.
      // Tường gần (<= frontStopMaxMm): đi thêm tối đa extendMm. Map BIẾT có tường cuối đoạn: tin VL53 tới SLALOM_FRONT_FAR_MM,
      // đi thêm tối đa 15% đoạn + 1 ô (log: đoạn 14 ô encoder báo đủ mà VL53 còn 415 mm -> cua sớm 1.4 ô, đâm tường)
      // ƯU TIÊN TƯỜNG TRƯỚC: map biết có tường cuối đoạn mà VL53 trước còn đọc xa hơn điểm cua -> xe CHƯA tới (chưa đủ ô),
      // KHÔNG cua theo encoder / mép hông, đi tiếp tới khi VL53 trước tới ngưỡng. Giới hạn an toàn: quá kế hoạch 25% đoạn + 1.5 ô.
      // VL53 không đọc được (-3 / xa hơn SLALOM_FRONT_FAR_MM) thì mới tin encoder.
      bool far = frontWall && (float)_rawF <= SLALOM_FRONT_FAR_MM && s < distMm + 0.25f * distMm + 1.5f * cellMm;
      bool near = (float)_rawF <= frontStopMaxMm && s < distMm + extendMm;
      bool wallFar = chkFront && frontReady && _rawF > 20 && (float)_rawF > trigMm + frontAlignTol && (near || far);
      if (!wallFar) { reason = "SUCCESS"; endBy = (s > distMm + 3.0f) ? "TUONG+" : "ENCODER"; break; }
    }
    // ===== ĐI THẲNG: DÙNG CHUNG PID VỚI MODE 3 / MODE 1 (updatePIDLoop): encoder P + gyro PI + bám tường dịch hướng gyro =====
    // Chỉ khi CUA mới dùng bộ điều khiển Slalom (slalomTurn). Tốc độ = baseForwardSpeed (PWM, tăng dần accelRampMs).
    // Encoder: chênh quãng đường tích lũy 2 bánh từ đầu đoạn (|L| - |R| > 0: bánh trái đi trước -> giảm trái, tăng phải)
    float travL = (float)labs(cl - l0), travR = (float)labs(cr - r0);
    const bool anyWallNow = hold.wallEnable && s >= slNoWallMm &&   // vùng không bám tường sau calib: encoder giữ thẳng
        ((leftReady && _wFiltL > 20.0f && _wFiltL < hold.wallValidMax) || (rightReady && _wFiltR > 20.0f && _wFiltR < hold.wallValidMax));
    float pidOut = 0.0f;
    float scL, scR;
    launchScale(travL, travR, scL, scR);   // hệ số motor: launch lúc đề, motorScale khi chạy
    // Không tường hông + yaw vừa calib (lùi kịch tường, chưa cua): giữ hướng bằng gyro thay encoder
    // (log: sau calib ô không tường, đề pa lệch hướng, encoder không biết -> đi chéo 440 mm)
    const bool freshGyro = !straightUseGyro && _yawFresh && mpuReady && !anyWallNow;
    if (anyWallNow || freshGyro) _encOff = travL - travR;   // có tường: tường lái, encoder đứng ngoài (log: encoder kéo ngược ~-9 PWM, xe trôi phải)
    else pidOut = constrain(encKp * ((travL - travR) - _encOff), -encMaxCorr, encMaxCorr);
    float wallErr = 0.0f;
    bool wallSeen = false;
    if (hold.wallEnable) {
      bool okL = leftReady && _wFiltL > 20.0f && _wFiltL < hold.wallValidMax;
      bool okR = rightReady && _wFiltR > 20.0f && _wFiltR < hold.wallValidMax;
      float eL = _wFiltL - hold.wallBaseL, eR = _wFiltR - hold.wallBaseR;
      wallSeen = okL || okR;
      if (okL && okR) wallErr = 0.5f * (eL - eR);
      else if (okL) wallErr = eL;
      else if (okR) wallErr = -eR;
      wallErr = constrain(wallErr, -hold.wallErrClamp, hold.wallErrClamp);
    }
    float wallYawOff = 0.0f;
    uint8_t wallCombo = 0;
    if (hold.wallEnable) {
      wallCombo = ((leftReady && _wFiltL > 20.0f && _wFiltL < hold.wallValidMax) ? 1 : 0) |
                  ((rightReady && _wFiltR > 20.0f && _wFiltR < hold.wallValidMax) ? 2 : 0);
    }
    if (s < slNoWallMm) { wallErr = 0.0f; wallCombo = 0; }   // vừa lùi calib: chưa bám tường
    if (freshGyro) pidOut += constrain(-hold.yawKp * wrap180(_turnHeading - mpu6050.getYaw()), -yawMaxCorr, yawMaxCorr);
    if (!straightUseGyro) {
      // KHÔNG gyro: tường P (WALL_KP) + D theo góc chéo với tường (WALL_KD), cộng PWM. Gyro chỉ dùng khi cua.
      if (hold.wallEnable) {
        float wc = hold.wallKp * wallErr + wallAngleTerm(s, wallErr, wallCombo);
        if (handOver) wc *= constrain(max(0.0f, distMm - s) / SLALOM_WALL_FADE_MM, 0.0f, 1.0f);
        pidOut += constrain(wc, -hold.wallMaxCorr, hold.wallMaxCorr);
      }
      // Hãm xoay bằng gyro (tốc độ góc luôn đúng, không cần hướng chuẩn): tường kéo mạnh mà chỉ có 1 bên tường
      // thì không có khâu D -> xe xoay tới 140 độ/s lao vào tường (log sau cua (15,0))
      if (mpuReady && !freshGyro) pidOut -= constrain(WALL_GYRO_DAMP * turnLeftYawSign * mpu6050.getGyroRate(), -WALL_GYRO_DAMP_MAX, WALL_GYRO_DAMP_MAX);
    } else if (hold.wallEnable && hold.wallYawGain > 0.0f) wallYawOff = constrain(hold.wallYawGain * wallErr, -hold.wallYawMax, hold.wallYawMax);
    else if (hold.wallEnable) pidOut += constrain(hold.wallKp * wallErr, -hold.wallMaxCorr, hold.wallMaxCorr);
    // Sắp vào cua: dần bỏ phần lái lệch của bám tường để xe vào cua với hướng chuẩn
    if (handOver) wallYawOff *= constrain(max(0.0f, distMm - s) / SLALOM_WALL_FADE_MM, 0.0f, 1.0f);
    if (wallSeen && s - wallHistS >= 10.0f) {
      for (int k = 7; k > 0; k--) wallHist[k] = wallHist[k - 1];
      wallHist[0] = wallErr;
      wallHistS = s;
      if (wallHistN < 8) wallHistN++;
    } else if (!wallSeen) wallHistN = 0;
    // Reset yaw CHỈ KHI có tường hông, xe ở giữa hành lang VÀ đi song song tường (sai lệch tường gần như không đổi trong ~70 mm).
    // Log: reset lúc xe còn chéo 5° (đang trôi về giữa) -> hướng chuẩn sai 5° -> đoạn dài sau đó lao vào tường phải.
    // Không có tường hông: không reset (không có gì để biết xe đang thẳng)
    const bool parallel = wallHistN >= 8 && fabsf(wallHist[0] - wallHist[7]) <= SLALOM_YAW_RESET_MAX_DRIFT_MM;
    if (straightUseGyro && !yawResetDone && s >= SLALOM_YAW_RESET_MM && fabsf(mpu6050.getGyroRate()) < SLALOM_YAW_RESET_MAX_DPS &&
        wallSeen && fabsf(wallErr) <= SLALOM_YAW_RESET_MAX_WALL_MM && parallel) {
      yawResetDone = true;
      float before = wrap180(_turnHeading - mpu6050.getYaw());
      mpu6050.resetYaw();
      _absHeading = 0.0f;
      _turnHeading = 0.0f;
      _turnHeadingValid = true;
      _targetYaw = 0.0f;
      yawI = 0.0f;
      char rb[64];
      snprintf(rb, sizeof(rb), " [RESET YAW s=%d lech %.1f]", (int)s, before);
      trace += rb;
    }
    if (straightUseGyro) {
      float e = wrap180(_turnHeading + turnLeftYawSign * wallYawOff - mpu6050.getYaw());
      // Khâu I chỉ tích khi KHÔNG bám tường bằng PWM: nếu không, I tích dần đúng bằng phần tường kéo và triệt tiêu nó
      // (log: tường đòi +15 PWM mà pid chỉ còn 0..5, xe đi song song lệch 30 mm rồi quẹt tường)
      const bool wallPwmActive = hold.wallEnable && hold.wallYawGain <= 0.0f && wallSeen;
      if (hold.yawKi > 0.0f && !wallPwmActive) {
        yawI = constrain(yawI + e * dt, -hold.yawIMax / hold.yawKi, hold.yawIMax / hold.yawKi);
      }
      pidOut += constrain(-(hold.yawKp * e + hold.yawKi * yawI), -yawMaxCorr, yawMaxCorr);
    }
    // Tốc độ: PWM baseForwardSpeed tăng dần từ rampFrom; gần cuối đoạn hãm theo biên dạng quãng đường về tốc độ vào cua
    // (hoặc tốc độ dừng), đo bằng encoder - quá nhanh thì giảm PWM (âm = hãm đảo chiều)
    float basePwm = (float)baseForwardSpeed;
    if (accelRampMs > 0) {
      float t = (nowMs - tStart) / (float)accelRampMs;
      if (t < 1.0f) basePwm = rampFrom + (basePwm - rampFrom) * t;
    }
    float rem = max(0.0f, distMm - s);
    float vAllow = sqrtf(vExit * vExit + 2.0f * SLALOM_DECEL_MM_S2 * rem);
    float vMeas = 0.5f * (piL.meas + piR.meas);
    if (vMeas > vAllow) {
      float brakePwm = SLALOM_PWM_DEAD + SLALOM_PWM_PER_MMS * vAllow + SLALOM_STRAIGHT_BRAKE_KP * (vAllow - vMeas);
      basePwm = max(min(basePwm, brakePwm), -(float)SLALOM_PWM_REV_MAX);
    }
    // Hiệu chỉnh lái tỉ lệ theo tốc độ: cùng 15 PWM lệch ở PWM 50 (vừa xuất phát) bẻ lái gấp ~4 lần ở PWM 230
    // (log: vừa lùi calib xong, PWM 50 + tường +15 -> xe quẹo trái lao vào tường trái)
    pidOut *= constrain(fabsf(basePwm) / (float)max((int)baseForwardSpeed, 1), 0.25f, 1.0f);
    int pwmL = (int)lroundf((basePwm - pidOut) * scL);
    int pwmR = (int)lroundf((basePwm + pidOut) * scR);
    pwmL = constrain(pwmL, -SLALOM_PWM_REV_MAX, 255);
    pwmR = constrain(pwmR, -SLALOM_PWM_REV_MAX, 255);
    vCmd = vMeas;
    if (nowMs - tTrace >= 60 && trace.length() < 1500) {
      tTrace = nowMs;
      char tb[72];
      snprintf(tb, sizeof(tb), " %d:%.1f/%d/%d/%d/%d/%d/%d/%d/%d/%d", (int)s,
               straightUseGyro ? wrap180(_turnHeading - mpu6050.getYaw()) : _wallAngDeg, (int)mpu6050.getGyroRate(), (int)wallErr,
               (int)_rawL, (int)_rawR, (int)_rawF, (int)pidOut, pwmL, pwmR, (int)vMeas);
      trace += tb;
    }
    driveFwdRaw(pwmL, pwmR);
    currentLeftSpeed = max(pwmL, 0); currentRightSpeed = max(pwmR, 0);
    if (nowMs - tStart > tLimit) { reason = "TIMEOUT"; break; }
  }
  // In vết đoạn thẳng theo từng khúc (navPrintf giới hạn 220 ký tự/lần)
  if (trace.length() > 0) {
    navPrintf(straightUseGyro ? ">> [THANG] s:lechYaw/gyro/tuong/L/R/F/pid/pwmL/pwmR/v (lechYaw = chuan - thuc, do)\n"
                              : ">> [THANG] s:gocTuong/gyro/tuong/L/R/F/pid/pwmL/pwmR/v (KHONG GYRO: gocTuong = goc cheo uoc tu tuong, do)\n");
    for (unsigned i = 0; i < trace.length(); i += 200) navPrintf(">>  %s\n", trace.substring(i, min((unsigned)trace.length(), i + 200)).c_str());
  }
  // Chỉ dừng khi VL53 trước báo quá sát tường (hoặc mất MPU/I2C: không còn cảm biến để lái). Mọi lý do khác (hết giờ...) coi như hết đoạn, đi tiếp
  const bool fatal = strcmp(reason, "FRONT_EMERGENCY") == 0 || strcmp(reason, "FRONT_COVERED") == 0 || strcmp(reason, "MPU_FAIL") == 0 || strcmp(reason, "I2C_FAULT") == 0;
  const bool ok = !fatal;
  if (!(ok && handOver)) {
    brakeMotors();
    unsigned long sw = millis();
    while (millis() - sw < 150) { mpu6050.update(); delay(2); }
    stopMotors();
    currentLeftSpeed = currentRightSpeed = 0;
    mpu6050.setAutoBias(true);
  }
  // Có tường trước ở ô vừa tới không: đã biết (frontWall) hoặc, khi dò, VL53 thấy vật trong tầm tin cậy lúc kết thúc đoạn
  if (slExplore) slLegWall = strcmp(endBy, "TUONG") == 0 || (strcmp(reason, "SUCCESS") == 0 && frontReady && _rawF > 20 && (float)_rawF <= frontStopMaxMm);
  else slLegWall = frontWall;
  slLegMm = s; slLegEnd = (strcmp(reason, "SUCCESS") == 0) ? endBy : reason; slLegFront = (int)_rawF;
  if (strcmp(reason, "SUCCESS") != 0) Serial.printf("# SLALOM STRAIGHT: %s | s=%.1f/%.1f mm\n", reason, s, distMm);
  return ok;
}

bool RobotNav::slalomLogPending() const { return slPending; }

// In log đã lưu ra Serial (gọi khi cắm USB), rồi xóa cờ chờ in
void RobotNav::printSlalomLog() {
  if (!slPending) return;
  Serial.println("# SLALOM ENCODER + MPU");
  Serial.print(slText);
  slPending = false;
}

// Bắt đầu lần chạy mới: xóa log các lần cua cũ
void RobotNav::slalomLogReset() {
  slText = "";
  slText.reserve(38000);
  slTurnNo = 0;
  slPending = false;
}

// Log text của các lần cua (để lưu vào flash)
const String &RobotNav::slalomLogText() const { return slText; }
