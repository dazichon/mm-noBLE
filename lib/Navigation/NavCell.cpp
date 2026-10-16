// DI CHUYỂN TỪNG Ô + ĐỌC VÁCH Ô
#include "RobotNav.h"
#include "NavInternal.h"

// =========================================================================
// 4 HÀM CHUYỂN ĐỘNG NGUYÊN TỬ (ATOMIC MOTION - BƯỚC 3)
// =========================================================================

WallStatus RobotNav::senseCurrentWalls() {
  updateSensors();
  WallStatus walls;
  // Tường sát xe (smoothDF <= 180mm): vách trước của ô hiện tại
  walls.hasFrontNear = frontReady && smoothDF > 20.0f && smoothDF <= 180.0f;
  // Tường trước của ô tiếp theo (180mm < smoothDF <= frontWallDist): vách trước của Ô PHÍA TRƯỚC
  walls.hasFront = !walls.hasFrontNear && frontReady && smoothDF > 180.0f && smoothDF <= (float)frontWallDist;

  if (walls.hasFrontNear) {
    // KHI BỊ CHẮN TƯỜNG TRƯỚC SÁT XE:
    // Cảm biến 45° chĩa ra trước sẽ phản xạ từ vách trước ở cự ly ~ smoothDF * 1.414.
    // Cần loại trừ phản xạ này để không nhận diện nhầm cửa mở/khoảng trống thành tường!
    float frontDiag = smoothDF * 1.414f;
    bool leftIsFrontReflection = (fabs(smoothDL - frontDiag) < 32.0f && smoothDL > 125.0f);
    bool rightIsFrontReflection = (fabs(smoothDR - frontDiag) < 32.0f && smoothDR > 125.0f);

    walls.hasLeft  = leftReady && smoothDL > 20.0f && smoothDL < (float)wallThreshold && !leftIsFrontReflection;
    walls.hasRight = rightReady && smoothDR > 20.0f && smoothDR < (float)rightWallThreshold && !rightIsFrontReflection;
  } else {
    // 2 Cảm biến 45° chĩa ra trước nhìn 2 bên sườn ô phía trước:
    walls.hasLeft  = (leftReady && smoothDL > 20.0f && smoothDL < (float)wallThreshold);
    walls.hasRight = (rightReady && smoothDR > 20.0f && smoothDR < (float)rightWallThreshold);
  }
  return walls;
}

void RobotNav::reportCell(char act, const WallStatus &w, const WallStatus *pre, bool blocked) {
  // Gói JSON cho bản đồ (in ra Serial): act = F/L/R/B (đã di chuyển) hoặc S (chỉ đọc vách)
  // Cảm biến nhìn trước 1 ô: wf/wl/wr = vách của Ô PHÍA TRƯỚC xe;
  // pf/pl/pr (nếu có) = vách của chính ô vừa bước vào
  String j = "{\"type\":\"cell\",\"act\":\"" + String(act) + "\"" +
             ",\"wf\":" + String(w.hasFront ? 1 : 0) +
             ",\"wl\":" + String(w.hasLeft ? 1 : 0) +
             ",\"wr\":" + String(w.hasRight ? 1 : 0) +
             ",\"wn\":" + String(w.hasFrontNear ? 1 : 0);
  if (pre) {
    j += ",\"pf\":" + String(pre->hasFront ? 1 : 0) +
         ",\"pl\":" + String(pre->hasLeft ? 1 : 0) +
         ",\"pr\":" + String(pre->hasRight ? 1 : 0);
  }
  if (blocked) j += ",\"blk\":1"; // chỉ xoay, không tiến ô
  j += "}";
  serialLink.println(j);
}

WallStatus RobotNav::moveOneCell(char turn, int cells) {
  if (cells < 1) cells = 1;
  long skip = (long)(cells - 1) * pulsesPerCell; // xung của các ô đi qua, vách hông chỉ lấy mẫu ở ô cuối
  String startMsg = ">> [ATOMIC] BAT DAU TIEN " + String(cells) + " O (" + String(pulsesPerCell * cells) + " xung)...";
  Serial.println(startMsg);
  serialLink.println(startMsg);

  preWalls = senseCurrentWalls(); // cảm biến nhìn trước 1 ô: đây là vách của ô sắp bước vào
  startCellFresh = false;
  bool keepAuto = autoWallFollowActive; // lưu trạng thái tự động trước khi PID tự dừng cuối ô

  stepCell(cells); // Kích hoạt PID bám tường và đặt mục tiêu đúng `cells` ô (pulsesPerCell xung/ô)

  unsigned long startTime = millis();
  unsigned long maxMs = 6000UL * (unsigned long)cells;

  // Thu thập mẫu vách hông liên tục trong cửa sổ [28% .. 75%] của ô
  int leftOpenCount = 0;
  int leftWallCount = 0;
  int rightOpenCount = 0;
  int rightWallCount = 0;

  while (stepCellActive && (millis() - startTime < maxMs)) {
    updateSensors();
    if (mpuReady) {
      mpu6050.update();
    }

    // Cửa sổ nhận diện vách hông [28% .. 75%]:
    // - Đã vượt qua trụ góc và vách của ô cũ (>28%)
    // - Chưa bị ảnh hưởng bởi phản xạ tường trước (<75%)
    if (stepTraveledPulses >= skip + (long)(sideSampleFrac * pulsesPerCell) &&
        stepTraveledPulses <= skip + (long)(0.75f * pulsesPerCell)) {
      if (leftReady) {
        if (smoothDL >= (float)wallThreshold) {
          leftOpenCount++;
        } else if (smoothDL > 20.0f && smoothDL < (float)wallThreshold) {
          leftWallCount++;
        }
      }
      if (rightReady) {
        if (smoothDR >= (float)rightWallThreshold) {
          rightOpenCount++;
        } else if (smoothDR > 20.0f && smoothDR < (float)rightWallThreshold) {
          rightWallCount++;
        }
      }
    }

    // updatePIDLoop() tự động kiểm tra:
    // a) Chạm vách trước (dF <= frontStopDist)
    // b) Đạt đủ pulsesPerCell (pulsesPerCell xung)
    // và sẽ tự phanh dừng, tắt stepCellActive!
    updatePIDLoop();
    delay(2);
  }

  // Đảm bảo xe đã phanh dừng hoàn toàn dứt điểm tại tâm ô
  brakeMotors();
  stopPID();
  autoWallFollowActive = keepAuto; // stopPID() (kể cả trong updatePIDLoop) tắt cờ tự động -> khôi phục

  // Đã dừng hẳn ở ô mới: chờ ~250ms cho cảm biến đọc lại vài lần (bộ lọc EMA ổn định)
  // rồi mới đọc vách -> cập nhật lại tường của ô vừa tới bằng số đo lúc đứng yên.
  unsigned long settle = millis();
  while (millis() - settle < 250) {
    updateSensors();
    if (mpuReady) mpu6050.update();
    delay(5);
  }

  // Đã dừng ở tâm ô mới -> đọc vách của ô kế tiếp phía trước (look-ahead)
  WallStatus status = senseCurrentWalls();

  // Vách của chính ô hiện tại:
  // Ưu tiên kết quả thống kê liên tục trong cửa sổ [28% .. 75%] lúc xe đang lướt qua thân ô
  WallStatus curCellWalls;
  if (leftOpenCount + leftWallCount >= 2) {
    // Có khoảng trống nếu số lần đọc cửa mở >= 2 hoặc tỷ lệ mở >= tường
    curCellWalls.hasLeft = (leftOpenCount >= 2 && leftOpenCount >= leftWallCount) ? false : (leftWallCount > leftOpenCount);
  } else {
    curCellWalls.hasLeft = status.hasLeft;
  }

  if (rightOpenCount + rightWallCount >= 2) {
    curCellWalls.hasRight = (rightOpenCount >= 2 && rightOpenCount >= rightWallCount) ? false : (rightWallCount > rightOpenCount);
  } else {
    curCellWalls.hasRight = status.hasRight;
  }

  curCellWalls.hasFront = status.hasFrontNear;
  curCellWalls.hasFrontNear = status.hasFrontNear;
  curWalls = curCellWalls;

  // Bị tường chặn khi chưa đi được nửa ô -> xe vẫn ở ô cũ (thường do đọc nhầm cửa mở)
  lastMoveBlocked = stepTraveledPulses < pulsesPerCell / 2;
  lastMovedCells = lastMoveBlocked ? 0 : (int)min((long)cells, (stepTraveledPulses + pulsesPerCell / 2) / pulsesPerCell);
  if (lastMovedCells < 1 && !lastMoveBlocked) lastMovedCells = 1;
  if (lastMoveBlocked) {
    status.hasFrontNear = true;
    status.hasFront = false;
    serialLink.println(">> [ATOMIC] BI TUONG CHAN! Xe van o o cu (" + String(stepTraveledPulses) + " xung)");
  }

  String resMsg = ">> [ATOMIC] DA DEN TAM O MOI! O ke tiep: Truoc=" + String(status.hasFront ? "CO" : "TRONG") +
                  " | Trai=" + String(status.hasLeft ? "CO" : "TRONG") +
                  " | Phai=" + String(status.hasRight ? "CO" : "TRONG") +
                  " | Vach O nay: Trai=" + String(curCellWalls.hasLeft ? "CO" : "TRONG") +
                  " (Open:" + String(leftOpenCount) + "/Wall:" + String(leftWallCount) + ")" +
                  ", Phai=" + String(curCellWalls.hasRight ? "CO" : "TRONG") +
                  " (Open:" + String(rightOpenCount) + "/Wall:" + String(rightWallCount) + ")" +
                  ", Truoc=" + String(curCellWalls.hasFront ? "CO" : "TRONG") +
                  " | Xung: " + String(stepTraveledPulses);
  Serial.println(resMsg);
  serialLink.println(resMsg);
  reportCell(turn, status, lastMoveBlocked ? nullptr : &curCellWalls, lastMoveBlocked);

  return status;
}

WallStatus RobotNav::turnLeftAndStep(int cells) {
  serialLink.println(">> [ATOMIC] LENH: RE TRAI 90 DO & TIEN 1 O");
  turnLeft(90.0f);
  delay(60);
  resetEnc();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('L', cells);
}

WallStatus RobotNav::turnRightAndStep(int cells) {
  serialLink.println(">> [ATOMIC] LENH: RE PHAI 90 DO & TIEN 1 O");
  turnRight(90.0f);
  delay(60);
  resetEnc();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('R', cells);
}

WallStatus RobotNav::turnAroundAndStep(int cells) {
  serialLink.println(">> [ATOMIC] LENH: QUAY DAU 180 DO & TIEN 1 O");
  turnRight(180.0f);
  delay(60);
  resetEnc();
  if (mpuReady) {
    _targetYaw = mpu6050.getYaw();
  }
  return moveOneCell('B', cells);
}
