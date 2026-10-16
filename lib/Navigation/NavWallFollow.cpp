// CHẾ ĐỘ BÁM TƯỜNG TỰ ĐỘNG
#include "RobotNav.h"
#include "NavInternal.h"

// =========================================================================
// CHẾ ĐỘ 3: BÁM TƯỜNG TỰ ĐỘNG (AUTONOMOUS WALL FOLLOWER)
// =========================================================================

void RobotNav::startAutoWallFollow() {
  pidRunActive = false;
  stepCellActive = false;
  autoCellCount = 0;
  curWalls = senseCurrentWalls();
  if (startCellFresh) { // ô xuất phát luôn có tường hai bên
    curWalls.hasLeft = true;
    curWalls.hasRight = true;
  }
  lastAutoDecision = "KÍCH HOẠT";
  autoWallFollowActive = true;

  String msg = ">> [CHẾ ĐỘ 3] BẮT ĐẦU BÁM TƯỜNG TỰ ĐỘNG (" +
               String(followRightHand ? "LUẬT TAY PHẢI" : "LUẬT TAY TRÁI") +
               ") - GIỚI HẠN AN TOÀN " + String(autoMaxCells) + " Ô";
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stopAutoWallFollow() {
  autoWallFollowActive = false;
  cornerState = STRAIGHT;
  stopPID();
  brakeMotors();
  lastAutoDecision = "ĐÃ DỪNG";
  String msg = ">> [CHẾ ĐỘ 3] ĐÃ DỪNG TỰ ĐỘNG! Tổng số ô đã chạy: " + String(autoCellCount);
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stepAutoWallFollow() {
  if (!autoWallFollowActive) return;

  // 1. Kiểm tra an toàn giới hạn số ô để phòng chạy vòng tròn lặp vô tận
  if (autoCellCount >= autoMaxCells) {
    stopAutoWallFollow();
    String limMsg = ">> [CHẾ ĐỘ 3] ĐẠT MỐC AN TOÀN " + String(autoMaxCells) + " Ô! Tự động phanh dừng.";
    Serial.println(limMsg);
    serialLink.println(limMsg);
    return;
  }

  // 2. Vách ô hiện tại: hông = mẫu đã lưu khi vào ô, trước = đọc trực tiếp
  WallStatus walls = curWalls;
  walls.hasFront = senseCurrentWalls().hasFrontNear; // chỉ bị chắn khi tường sát xe

  // Chế độ đi thẳng: gặp tường trước thì quay đầu, còn lại đi thẳng
  if (patrolMode) {
    if (walls.hasFront) {
      lastAutoDecision = "GẶP TƯỜNG: QUAY ĐẦU 180°";
      turnAroundAndStep();
    } else {
      lastAutoDecision = "TIẾN THẲNG 1 Ô";
      moveOneCell();
    }
    autoCellCount++;
    delay(80);
    return;
  }

  // 3. Ra quyết định điều hướng tự động
  if (followRightHand) {
    // === QUY TẮC BÀN TAY PHẢI (RIGHT-HAND RULE) ===
    // Ưu tiên: 1. Rẽ Phải -> 2. Đi Thẳng -> 3. Rẽ Trái -> 4. Quay Đầu 180°
    if (!walls.hasRight) {
      lastAutoDecision = "RẼ PHẢI & TIẾN 1 Ô";
      turnRightAndStep();
    } else if (!walls.hasFront) {
      lastAutoDecision = "TIẾN THẲNG 1 Ô";
      moveOneCell();
    } else if (!walls.hasLeft) {
      lastAutoDecision = "RẼ TRÁI & TIẾN 1 Ô";
      turnLeftAndStep();
    } else {
      lastAutoDecision = "ĐƯỜNG CỤT: QUAY ĐẦU 180°";
      turnAroundAndStep();
    }
  } else {
    // === QUY TẮC BÀN TAY TRÁI (LEFT-HAND RULE) ===
    // Ưu tiên: 1. Rẽ Trái -> 2. Đi Thẳng -> 3. Rẽ Phải -> 4. Quay Đầu 180°
    if (!walls.hasLeft) {
      lastAutoDecision = "RẼ TRÁI & TIẾN 1 Ô";
      turnLeftAndStep();
    } else if (!walls.hasFront) {
      lastAutoDecision = "TIẾN THẲNG 1 Ô";
      moveOneCell();
    } else if (!walls.hasRight) {
      lastAutoDecision = "RẼ PHẢI & TIẾN 1 Ô";
      turnRightAndStep();
    } else {
      lastAutoDecision = "ĐƯỜNG CỤT: QUAY ĐẦU 180°";
      turnAroundAndStep();
    }
  }

  autoCellCount++;
  String statusMsg = ">> [CHẾ ĐỘ 3] Ô #" + String(autoCellCount) + ": " + lastAutoDecision;
  Serial.println(statusMsg);
  serialLink.println(statusMsg);

  // Cho xe nghỉ ổn định 80ms tại tâm ô trước khi sang ô tiếp theo
  delay(80);
}
