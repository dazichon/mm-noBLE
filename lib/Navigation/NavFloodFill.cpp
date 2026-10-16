// CHẾ ĐỘ FLOOD-FILL
#include "RobotNav.h"
#include "NavInternal.h"

// =========================================================================
// CHẾ ĐỘ 4: FLOOD-FILL (dùng maze_algorithm.h)
// =========================================================================

// sure = true: chắc chắn (xe vừa đi qua cửa này) -> ghi đè.
// sure = false: đọc cảm biến -> chỉ được THÊM tường, không xoá tường đã thấy
// (tránh đọc nhiễu 1 lần làm "phá tường" rồi xe lao vào tường).
static void ffSetWall(ParentMaze &m, int x, int y, int dir, bool wall, bool sure = false) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  int nx = x + DX[dir], ny = y + DY[dir];
  if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE)
    return; // tường biên luôn có sẵn
  Cell &c = m.cell(x, y);
  bool *self[4] = {&c.north_wall, &c.east_wall, &c.south_wall, &c.west_wall};
  if (!wall && !sure && *self[dir]) return;
  *self[dir] = wall;
  Cell &n = m.cell(nx, ny);
  bool *other[4] = {&n.north_wall, &n.east_wall, &n.south_wall, &n.west_wall};
  *other[(dir + 2) % 4] = wall; // đồng bộ tường với ô kề
}

// Cảm biến nhìn trước 1 ô: ws (trước/trái/phải) là vách của ô PHÍA TRƯỚC (x,y) theo hướng h
static void ffRecordAhead(ParentMaze &m, int x, int y, int h, const WallStatus &ws) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  ffSetWall(m, x, y, h, ws.hasFrontNear, true); // vách trước của ô đang đứng: đo lúc đứng yên -> ghi đè
  if (ws.hasFrontNear) return;            // bị chắn: không nhìn được ô phía trước
  int ax = x + DX[h], ay = y + DY[h];
  if (ax < 0 || ax >= MAZE_SIZE || ay < 0 || ay >= MAZE_SIZE) return;
  ffSetWall(m, ax, ay, (h + 3) % 4, ws.hasLeft);
  ffSetWall(m, ax, ay, (h + 1) % 4, ws.hasRight);
  ffSetWall(m, ax, ay, h, ws.hasFront);
  m.cell(ax, ay).known = true;
}

void RobotNav::resetFloodFill(int x, int y, int h) {
  ffMaze.clear_mem();
  startCellFresh = true; // ô xuất phát (bất kể toạ độ) luôn có tường trái, phải, sau theo hướng ban đầu
  x = constrain(x, 0, MAZE_SIZE - 1);
  y = constrain(y, 0, MAZE_SIZE - 1);
  ffX = x;
  ffY = y;
  ffH = ((h % 4) + 4) % 4;
  ffStart = {(int8_t)x, (int8_t)y};
  // Tường bao quanh mê cung thật ffCols x ffRows
  for (int i = 0; i < ffRows; i++) ffSetWall(ffMaze, ffCols - 1, i, 1, true, true);
  for (int i = 0; i < ffCols; i++) ffSetWall(ffMaze, i, ffRows - 1, 0, true, true);
}

void RobotNav::startFloodFill(uint8_t mode) {
  pidRunActive = false;
  stepCellActive = false;
  autoWallFollowActive = false;
  autoCellCount = 0;
  ffMode = mode;

  if (!ffMaze.cell(ffX, ffY).run_visited) {
    WallStatus ws = senseCurrentWalls(); // vách của ô phía trước
    if (startCellFresh) { // ô xuất phát: mặc định có tường trái, phải và SAU (theo hướng ban đầu ffH)
      ffSetWall(ffMaze, ffX, ffY, (ffH + 3) % 4, true);
      ffSetWall(ffMaze, ffX, ffY, (ffH + 1) % 4, true);
      ffSetWall(ffMaze, ffX, ffY, (ffH + 2) % 4, true);
      ffMaze.cell(ffX, ffY).known = true;
    }
    ffMaze.cell(ffX, ffY).run_visited = true;
    ffRecordAhead(ffMaze, ffX, ffY, ffH, ws);
    reportCell('S', ws);
  }

  lastAutoDecision = "FF KÍCH HOẠT";
  ffActive = true;
  String msg = ">> [CHẾ ĐỘ 4] FLOOD-FILL " +
               String(mode == 0 ? "TỚI ĐÍCH" : "KHÁM PHÁ TOÀN BỘ") + " | Xe (" +
               String(ffX) + "," + String(ffY) + ") hướng " + String(ffH) +
               " | Đích (" + String(ffGoal.x) + "," + String(ffGoal.y) + ") " +
               String(ffGoalSize) + "x" + String(ffGoalSize);
  Serial.println(msg);
  serialLink.println(msg);
}

void RobotNav::stopFloodFill(const char *reason) {
  ffActive = false;
  stopPID();
  brakeMotors();
  lastAutoDecision = reason;
  String msg = ">> [CHẾ ĐỘ 4] " + String(reason) + " | Số ô đã đi: " + String(autoCellCount);
  Serial.println(msg);
  serialLink.println(msg);
}

// Đếm số ô thẳng liên tiếp (tính từ ô `first`) theo hướng dir mà đường flood-fill hiện tại vẫn đi thẳng
// và các ô đi xuyên qua đã biết đủ vách. Dừng trước khi vào vùng đích / đổi hướng / ô chưa biết.
int RobotNav::ffRunLength(Point first, int dir) {
  static const int DX[4] = {0, 1, 0, -1};
  static const int DY[4] = {1, 0, -1, 0};
  int n = 1;
  Point p = first;
  while (n < ffMaxRun) {
    if (ffInGoal(p.x, p.y)) break;
    Cell &c = ffMaze.cell(p.x, p.y);
    bool wall[4] = {c.north_wall, c.east_wall, c.south_wall, c.west_wall};
    if (!c.known || wall[dir]) break;
    Point nx = ffMaze.get_next_move(p.x, p.y);
    if (nx.x != p.x + DX[dir] || nx.y != p.y + DY[dir]) break;
    p = nx;
    n++;
  }
  return n;
}

void RobotNav::stepFloodFill() {
  if (!ffActive) return;

  if (autoCellCount >= autoMaxCells) {
    stopFloodFill("DỪNG AN TOÀN (ĐỦ SỐ Ô TỐI ĐA)");
    return;
  }

  Point cur = {ffX, ffY};
  Point target = ffGoal;
  Point next = cur;

  if (ffMode == 1) {
    // KHÁM PHÁ HẾT: ưu tiên ô kề CHƯA KHÁM PHÁ theo thứ tự thẳng -> trái -> phải
    static const int DX[4] = {0, 1, 0, -1};
    static const int DY[4] = {1, 0, -1, 0};
    static const int PRIORITY[3] = {0, 3, 1}; // lệch so với hướng xe: thẳng, trái, phải
    Cell &c = ffMaze.cell(cur.x, cur.y);
    bool wall[4] = {c.north_wall, c.east_wall, c.south_wall, c.west_wall};
    bool picked = false;
    for (int k = 0; k < 3 && !picked; k++) {
      int d = (ffH + PRIORITY[k]) % 4;
      int nx = cur.x + DX[d], ny = cur.y + DY[d];
      if (wall[d] || nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE) continue;
      if (ffMaze.cell(nx, ny).run_visited || ffMaze.cell(nx, ny).known) continue; // đã biết đủ vách
      next = {(int8_t)nx, (int8_t)ny};
      picked = true;
    }
    if (!picked) {
      // Hết ô kề mới: quay lại (kể cả quay đầu) tới ô chưa đi gần nhất
      Point n = ffMaze.find_nearest_unvisited(cur);
      if (n.x < 0) {
        stopFloodFill("ĐÃ ĐI HẾT TẤT CẢ CÁC Ô");
        return;
      }
      target = n;
    } else {
      target = next;
    }
  }

  bool arrived = (ffMode == 0) ? ffInGoal(cur.x, cur.y)
                                : (cur.x == target.x && cur.y == target.y);
  if (arrived) {
    stopFloodFill("ĐÃ TỚI ĐÍCH");
    return;
  }

  if (ffMode == 0 || (next.x == cur.x && next.y == cur.y)) {
    ffMaze.floodfill_update(target.x, target.y, false, false, ffMode == 0 ? ffGoalSize : 1);
    next = ffMaze.get_next_move(cur.x, cur.y);
    if ((next.x == cur.x && next.y == cur.y) || ffMaze.cell(cur.x, cur.y).step >= 65535) {
      stopFloodFill("KHÔNG CÒN ĐƯỜNG ĐI");
      return;
    }
  }

  int dir;
  if (next.y > cur.y) dir = 0;
  else if (next.x > cur.x) dir = 1;
  else if (next.y < cur.y) dir = 2;
  else dir = 3;

  // Đi liên tục: gộp các ô thẳng đã biết vách thành 1 lần chạy (chỉ ở chế độ tới đích)
  int run = (ffContinuous && ffMode == 0) ? ffRunLength(next, dir) : 1;

  int diff = (dir - ffH + 4) % 4;
  WallStatus ws;
  if (diff == 0) {
    lastAutoDecision = "TIẾN THẲNG " + String(run) + " Ô";
    ws = moveOneCell('F', run);
  } else if (diff == 1) {
    lastAutoDecision = "RẼ PHẢI & TIẾN " + String(run) + " Ô";
    ws = turnRightAndStep(run);
  } else if (diff == 3) {
    lastAutoDecision = "RẼ TRÁI & TIẾN " + String(run) + " Ô";
    ws = turnLeftAndStep(run);
  } else {
    lastAutoDecision = "QUAY ĐẦU 180°";
    ws = turnAroundAndStep(run);
  }

  ffH = dir;
  if (lastMoveBlocked) {
    // Không đi được: xe vẫn ở ô cũ, hướng mới -> ghi chắc chắn tường trước rồi tính lại đường
    ffSetWall(ffMaze, ffX, ffY, ffH, true, true);
    String m = ">> [CHẾ ĐỘ 4] Bị tường chặn tại (" + String(ffX) + "," + String(ffY) + "), tính lại đường";
    Serial.println(m);
    serialLink.println(m);
    delay(80);
    return;
  }
  {
    static const int DX[4] = {0, 1, 0, -1};
    static const int DY[4] = {1, 0, -1, 0};
    int moved = max(1, lastMovedCells);
    // Các ô đi xuyên qua (không phải ô cuối): đánh dấu đã đi, cửa theo hướng chạy thông
    for (int i = 0; i < moved - 1; i++) {
      int mx = next.x + DX[dir] * i, my = next.y + DY[dir] * i;
      ffSetWall(ffMaze, mx, my, (dir + 2) % 4, false, true);
      ffSetWall(ffMaze, mx, my, dir, false, true);
      ffMaze.cell(mx, my).known = true;
      ffMaze.cell(mx, my).run_visited = true;
      autoCellCount++;
    }
    ffX = next.x + DX[dir] * (moved - 1);
    ffY = next.y + DY[dir] * (moved - 1);
  }
  // Cửa xe vừa bước qua luôn thông
  ffSetWall(ffMaze, ffX, ffY, (ffH + 2) % 4, false, true);
  // Đã tới ô này: CẬP NHẬT LẠI vách hông bằng số đo khi đi qua thân ô (ghi đè kết quả
  // nhìn trước 1 ô lúc còn ở ô cũ), không còn chỉ ghi khi ô chưa biết.
  ffSetWall(ffMaze, ffX, ffY, (ffH + 3) % 4, curWalls.hasLeft, true);
  ffSetWall(ffMaze, ffX, ffY, (ffH + 1) % 4, curWalls.hasRight, true);
  ffMaze.cell(ffX, ffY).known = true;
  ffMaze.cell(ffX, ffY).run_visited = true;
  ffRecordAhead(ffMaze, ffX, ffY, ffH, ws);
  autoCellCount++;

  String m = ">> [CHẾ ĐỘ 4] Ô #" + String(autoCellCount) + " -> (" + String(ffX) + "," +
             String(ffY) + ") " + lastAutoDecision;
  Serial.println(m);
  serialLink.println(m);
  delay(80);
}
