#include "Mapping.h"
#include "Motion.h"

MazeMapper mapper;

static const int8_t DX[4] = {0, 1, 0, -1};
static const int8_t DY[4] = {1, 0, -1, 0};
static const char *DIR_NAME[4] = {"BAC", "DONG", "NAM", "TAY"};

static bool &wallRef(Cell &c, int dir) {
  switch (dir) {
    case 0: return c.north_wall;
    case 1: return c.east_wall;
    case 2: return c.south_wall;
    default: return c.west_wall;
  }
}

static bool inMaze(int cx, int cy) { return cx >= 0 && cx < MAZE_SIZE && cy >= 0 && cy < MAZE_SIZE; }

// ================= BẢN ĐỒ =================

void MazeMapper::reset() {
  maze.clear_mem();
  memset(known, 0, sizeof(known));
  // Biên của mê cung thật
  for (int i = 0; i < cols; i++) { setWall(i, 0, 2, true, true); setWall(i, rows - 1, 0, true, true); }
  for (int i = 0; i < rows; i++) { setWall(0, i, 3, true, true); setWall(cols - 1, i, 1, true, true); }
  x = 0; y = 0; h = 0;
  maze.cell(0, 0).run_visited = true;
}

// sure = true: đọc ở ô đang đứng (vách trước) hoặc vừa đi qua -> ghi đè.
// sure = false: đọc nhìn-trước (cảm biến chéo) -> chỉ ghi khi chưa biết, tránh đọc nhiễu đè lên số đọc tốt.
void MazeMapper::setWall(int cx, int cy, int dir, bool wall, bool sure) {
  if (!inMaze(cx, cy)) return;
  if (isKnown(cx, cy, dir) && !sure) return;
  wallRef(maze.cell(cx, cy), dir) = wall;
  known[cx][cy] |= (1 << dir);
  int nx = cx + DX[dir], ny = cy + DY[dir];
  if (inMaze(nx, ny)) {
    int back = (dir + 2) % 4;
    wallRef(maze.cell(nx, ny), back) = wall;
    known[nx][ny] |= (1 << back);
  }
}

bool MazeMapper::hasWall(int cx, int cy, int dir) const {
  const Cell &c = maze.cell(cx, cy);
  switch (dir) {
    case 0: return c.north_wall;
    case 1: return c.east_wall;
    case 2: return c.south_wall;
    default: return c.west_wall;
  }
}

bool MazeMapper::isKnown(int cx, int cy, int dir) const { return known[cx][cy] & (1 << dir); }

String MazeMapper::draw() const {
  static const char *ROBOT[4] = {" ^ ", " > ", " v ", " < "};
  String s;
  s.reserve((cols * 4 + 3) * (rows * 2 + 1));
  for (int cy = rows - 1; cy >= 0; cy--) {
    // Vách Bắc của hàng
    for (int cx = 0; cx < cols; cx++) {
      s += '+';
      s += !isKnown(cx, cy, 0) ? " ? " : hasWall(cx, cy, 0) ? "---" : "   ";
    }
    s += "+\n";
    // Vách Tây + nội dung ô, cuối hàng là vách Đông
    for (int cx = 0; cx < cols; cx++) {
      s += !isKnown(cx, cy, 3) ? '?' : hasWall(cx, cy, 3) ? '|' : ' ';
      if (cx == x && cy == y) s += ROBOT[h];
      else if (inGoal(cx, cy)) s += " G ";
      else if (maze.cell(cx, cy).run_visited) s += " . ";
      else s += "   ";
    }
    s += !isKnown(cols - 1, cy, 1) ? '?' : hasWall(cols - 1, cy, 1) ? '|' : ' ';
    s += '\n';
  }
  for (int cx = 0; cx < cols; cx++) {
    s += '+';
    s += !isKnown(cx, 0, 2) ? " ? " : hasWall(cx, 0, 2) ? "---" : "   ";
  }
  s += "+\n";
  return s;
}

// ================= DÒ ĐƯỜNG =================

void MazeMapper::explore() {
  robotNav.navReport = "";
  robotNav.navReportPending = false;
  reset();
  datHuongGoc();

  // Ô xuất phát: có tường sau, 2 bên theo quy ước
  setWall(x, y, (h + 2) % 4, true, true);
  if (startSidesWall) { setWall(x, y, (h + 3) % 4, true, true); setWall(x, y, (h + 1) % 4, true, true); }

  robotNav.navPrintf(">> [MAP] BAT DAU | me cung %ux%u | dich (%u,%u) %ux%u\n", cols, rows, goalX, goalY, goalSize, goalSize);

  const char *endReason = "HET SO O TOI DA";
  uint16_t cells = 0;
  bool calibDone = false;
  for (uint16_t loop = 0; loop < maxCells * 4 + 8 && cells < maxCells; loop++) {
    if (inGoal(x, y)) { endReason = "TOI DICH"; break; }

    // 0. Biết CHẮC sau lưng có tường -> lùi kịch tường, calib gyro, tiến lại tâm ô (mỗi tư thế x,y,h chỉ 1 lần)
    int back = (h + 2) % 4;
    if (calibBack && !calibDone && isKnown(x, y, back) && hasWall(x, y, back)) {
      robotNav.navPrintf(">> [MAP] O (%d,%d): SAU LUNG CO TUONG -> LUI CALIB\n", x, y);
      if (!calibNgoCut()) { endReason = "CALIB LOI"; break; }
      calibDone = true;
    }

    // 1. Đọc tường (xe đứng yên ở tâm ô)
    WallState l, f, r;  // l, r không dùng (xem dưới)
    docTuong(l, f, r);
    if (f == WS_UNKNOWN) { endReason = "VL53 TRUOC KHONG DANG TIN"; break; }
    setWall(x, y, h, f == WS_WALL, true);
    if (f == WS_WALL && alignFront) canTuongTruoc();  // có tường trước: căn lại đúng tâm ô theo VL53 trước
    // Vách hông KHÔNG lấy lúc đứng yên: tia 45° ở tâm ô chiếu đúng vào cột góc giữa 2 ô nên hay đọc sai.
    // Vách hông được lấy trong lúc đi vào ô (xem bước 3).
    if (Serial) Serial.print(draw());

    // 2. Flood-fill (tường chưa biết coi như thông) -> chọn hướng có bước nhỏ nhất, ưu tiên đi thẳng
    maze.floodfill_update(goalX, goalY, false, false, goalSize);
    if (maze.cell(x, y).step >= 65535) { endReason = "KHONG CON DUONG TOI DICH"; break; }
    int best = -1, bestStep = 65535;
    for (int k = 0; k < 4; k++) {
      int d = (h + k) % 4;  // k = 0 trước tiên: hoà điểm thì đi thẳng
      int nx = x + DX[d], ny = y + DY[d];
      if (hasWall(x, y, d) || !inMaze(nx, ny)) continue;
      if (maze.cell(nx, ny).step < bestStep) { bestStep = maze.cell(nx, ny).step; best = d; }
    }
    if (best < 0) { endReason = "BI KET"; break; }

    // 3. Đi hoặc quay
    int rel = (best - h + 4) % 4;
    robotNav.navPrintf(">> [MAP] O (%d,%d) huong %s -> %s\n", x, y, DIR_NAME[h],
                       rel == 0 ? "DI THANG" : rel == 1 ? "QUAY PHAI" : rel == 3 ? "QUAY TRAI" : "QUAY DAU");
    bool ok = true;
    if (rel == 0) {
      diThang(1);
      setWall(x, y, h, false, true);  // vừa đi qua: chắc chắn thông
      x += DX[h]; y += DY[h];
      maze.cell(x, y).run_visited = true;
      // Vách hông của ô vừa vào: đếm mẫu VL53 trái/phải trong lúc chạy (giữa quãng đường ô)
      WallState sl = robotNav.movingSideWall(true), sr = robotNav.movingSideWall(false);
      robotNav.navPrintf(">> [MAP] VACH HONG O (%d,%d) khi chay: TRAI=%s (gan %u/xa %u) PHAI=%s (gan %u/xa %u)\n", x, y,
                         sl == WS_WALL ? "TUONG" : sl == WS_OPEN ? "THONG" : "?", robotNav.sideNearL, robotNav.sideFarL,
                         sr == WS_WALL ? "TUONG" : sr == WS_OPEN ? "THONG" : "?", robotNav.sideNearR, robotNav.sideFarR);
      if (sl != WS_UNKNOWN) setWall(x, y, (h + 3) % 4, sl == WS_WALL, true);
      if (sr != WS_UNKNOWN) setWall(x, y, (h + 1) % 4, sr == WS_WALL, true);
      cells++;
    } else if (rel == 1) {
      ok = quayPhai(); h = (h + 1) % 4;
    } else if (rel == 3) {
      ok = quayTrai(); h = (h + 3) % 4;
    } else {
      ok = quayDau(); h = (h + 2) % 4;
    }
    calibDone = false;  // đã đi/quay: tư thế mới, được calib lại nếu sau lưng có tường
    if (!ok) { endReason = "QUAY LOI"; break; }
  }

  phanh();
  report = draw();
  robotNav.navPrintf(">> [MAP] KET THUC: %s | o (%d,%d) | da di %u o\n", endReason, x, y, cells);
  if (Serial) Serial.print(report);
  reportPending = !(bool)Serial;  // chạy rời USB: in khi cắm lại
  robotNav.navReportPending = !(bool)Serial;
}
