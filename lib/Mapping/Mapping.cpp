#include "Mapping.h"
#include "Motion.h"
#include "PresetMaze.h"

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

void MazeMapper::senseWall(int cx, int cy, int dir, bool wall, bool sure) {
  if (presetLoaded && presetTrust) return;  // tin bản đồ nạp sẵn hơn số đọc khi đang chạy
  setWall(cx, cy, dir, wall, sure);
}

// Đọc PRESET_MAZE (vẽ nhìn từ trên, ô xuất phát góc dưới PHẢI, xe quay mặt TÂY) sang toạ độ xe:
// X_xe = nhãn hàng, Y_xe = nhãn cột; hướng xe 0(trước)=Tây, 1(phải)=Bắc, 2=Đông, 3=Nam -> hướng xe = (hướng ảnh + 1) % 4
bool MazeMapper::loadPreset() {
  const char *lines[40];
  int lens[40], nl = 0;
  const char *p = PRESET_MAZE;
  while (*p && nl < 40) {
    const char *e = p;
    while (*e && *e != '\n') e++;
    int len = e - p;
    if (len > 0 && p[len - 1] == '\r') len--;
    bool grid = len > 3 && (p[3] == '+' || p[3] == '|');  // bỏ dòng trống và dòng nhãn cột
    if (grid) { lines[nl] = p; lens[nl] = len; nl++; }
    p = *e ? e + 1 : e;
  }
  if (nl != 2 * MAZE_SIZE + 1) {
    robotNav.navPrintf(">> [MAP] BAN DO NAP SAN LOI: %d dong (can %d)\n", nl, 2 * MAZE_SIZE + 1);
    return false;
  }
  auto at = [&](int li, int col) -> char { int i = 3 + col; return i < lens[li] ? lines[li][i] : ' '; };
  for (int r = 0; r < MAZE_SIZE; r++) {
    int py = MAZE_SIZE - 1 - r;
    for (int c = 0; c < MAZE_SIZE; c++) {
      int px = MAZE_SIZE - 1 - c;
      bool w[4];  // hướng ảnh: 0 = lên, 1 = phải, 2 = xuống, 3 = trái
      w[0] = at(2 * r, 4 * c + 2) == '-';
      w[2] = at(2 * r + 2, 4 * c + 2) == '-';
      w[3] = at(2 * r + 1, 4 * c) == '|';
      w[1] = at(2 * r + 1, 4 * c + 4) == '|';
      for (int pd = 0; pd < 4; pd++) setWall(py, px, (pd + 1) % 4, w[pd], true);
    }
  }
  presetLoaded = true;
  robotNav.navPrintf(">> [MAP] DA NAP BAN DO SAN (PresetMaze.h)%s\n", presetTrust ? ", tin ban do hon so doc khi chay" : "");
  return true;
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

// Hướng tới ô kề có step flood-fill nhỏ nhất (gọi sau floodfill_update). Hoà điểm thì ưu tiên đi thẳng. -1: bị kẹt
int MazeMapper::chooseDir(int cx, int cy, int ch) {
  // Tâm vùng đích (vd đích 2x2 ở (7,7) -> tâm (7.5,7.5)): hoà step thì chọn ô gần tâm hơn theo đường chim bay,
  // tránh bị "ưu tiên đi thẳng" kéo chạy dọc mép mê cung. Gần như bằng nhau thì vẫn ưu tiên đi thẳng (ít quay)
  float gx = tX + (tS - 1) * 0.5f, gy = tY + (tS - 1) * 0.5f;
  int best = -1, bestStep = 65535;
  float bestDist = 1e9f;
  for (int k = 0; k < 4; k++) {
    int d = (ch + k) % 4;  // k = 0 trước tiên: đi thẳng được xét đầu
    int nx = cx + DX[d], ny = cy + DY[d];
    if (hasWall(cx, cy, d) || !inMaze(nx, ny)) continue;
    int st = maze.cell(nx, ny).step;
    float dist = (nx - gx) * (nx - gx) + (ny - gy) * (ny - gy);
    bool better = st < bestStep || (tieTowardCenter && st == bestStep && dist < bestDist - 0.01f);
    if (better) { bestStep = st; bestDist = dist; best = d; }
  }
  return best;
}

static const char *wsName(WallState w) { return w == WS_WALL ? "TUONG" : w == WS_OPEN ? "THONG" : "?"; }

// Chạy LIÊN TỤC qua các ô thẳng (SEQ_MODE 10), không dừng ở từng ô.
// Mỗi ô: khi đi được decideFrac quãng đường, lấy tường hông + tường trước của ô đang vào (đọc khi chạy),
// flood-fill từ ô đó: vẫn đi thẳng được -> chạy tiếp không phanh; cần quay / tới đích / tường trước không chắc
// -> giảm tốc, dừng ở tâm ô rồi trả về để vòng chính đứng yên đọc lại + quay. Trả về số ô đã đi.
int MazeMapper::runStraight(int maxRun) {
  int run = 0;
  bool keepGoing = true;
  String log;  // không in Serial khi đang chạy (USB chậm có thể chặn vòng PID) -> in sau khi dừng
  char buf[280];
  while (keepGoing && run < maxRun) {
    setWall(x, y, h, false, true);  // sắp đi qua: chắc chắn thông
    int nx = x + DX[h], ny = y + DY[h];
    robotNav.startCells(1, true);
    long decideAt = (long)(decideFrac * robotNav.pulsesPerCell);
    bool decided = false;
    keepGoing = false;
    // Ghi vách ô nx,ny + chọn chạy tiếp/dừng. moving = false: xe đã dừng trước decideAt (dừng theo tường trước / timeout)
    auto decide = [&](bool moving) {
      decided = true;
      WallState sl = robotNav.movingSideWall(true), sr = robotNav.movingSideWall(false);
      WallState fw = robotNav.frontLocked ? WS_WALL : robotNav.movingFrontWall();  // đã bắt được tường thật thì là tường
      if (sl != WS_UNKNOWN) senseWall(nx, ny, (h + 3) % 4, sl == WS_WALL, true);
      if (sr != WS_UNKNOWN) senseWall(nx, ny, (h + 1) % 4, sr == WS_WALL, true);
      if (fw != WS_UNKNOWN) senseWall(nx, ny, h, fw == WS_WALL, false);  // đứng yên đọc lại sẽ ghi đè
      if (moving && !inTarget(nx, ny) && fw == WS_OPEN && run + 1 < maxRun) {
        maze.floodfill_update(tX, tY, false, false, tS);
        keepGoing = (chooseDir(nx, ny, h) == h);
      }
      if (!keepGoing) robotNav.stepNoStop = false;  // giảm tốc bò + phanh ở tâm ô
      snprintf(buf, sizeof(buf), ">> [RUN] O (%d,%d): TRAI=%s(%u/%u) PHAI=%s(%u/%u) TRUOC=%s(%u/%u)%s%s -> %s\n", nx, ny,
               wsName(sl), robotNav.sideNearL, robotNav.sideFarL, wsName(sr), robotNav.sideNearR, robotNav.sideFarR,
               wsName(fw), robotNav.frontNearN, robotNav.frontFarN, robotNav.frontLocked ? " [BAT TUONG]" : "",
               (robotNav.sideEchoL || robotNav.sideEchoR) ? " [BO MAU HONG CHAM TUONG TRUOC]" : "",
               keepGoing ? "CHAY TIEP" : "DUNG");
      log += buf;
    };
    unsigned long t0 = millis();
    while (robotNav.pidRunActive && millis() - t0 < 6000) {
      robotNav.update();
      if (!decided && robotNav.stepTraveledPulses >= decideAt) decide(true);
    }
    bool timedOut = robotNav.pidRunActive;
    if (robotNav.pidRunActive) robotNav.stopPID();
    if (!decided) {
      if (timedOut) phanh();  // timeout: dừng cho an toàn
      decide(false);          // dừng trước decideAt: vẫn ghi vách đã đếm được, không chạy tiếp
    }
    if (!keepGoing && robotNav.frontStopAdj != 0) {
      snprintf(buf, sizeof(buf), ">> [RUN] DUNG THEO TUONG TRUOC: doi diem dung %+ld xung\n", robotNav.frontStopAdj);
      log += buf;
    }
    x = nx; y = ny;
    maze.cell(x, y).run_visited = true;
    run++;
  }
  robotNav.stepNoStop = false;
  if (keepGoing) phanh();  // hết maxRun khi đang chạy
  robotNav.settle(120);
  Serial.print(log);
  robotNav.navReport += log;
  return run;
}

void MazeMapper::begin() {
  robotNav.navReport = "";
  robotNav.navReportPending = false;
  reset();
  presetLoaded = false;
  if (usePreset) loadPreset();
  datHuongGoc();

  // Ô xuất phát: có tường sau, 2 bên theo quy ước
  setWall(x, y, (h + 2) % 4, true, true);
  if (startSidesWall) { setWall(x, y, (h + 3) % 4, true, true); setWall(x, y, (h + 1) % 4, true, true); }

  robotNav.navPrintf(">> [MAP] BAT DAU | me cung %ux%u | dich (%u,%u) %ux%u\n", cols, rows, goalX, goalY, goalSize, goalSize);
}

// Dò đường tới vùng đích (tx,ty) kích thước ts: vòng chính đứng yên đọc tường / quay / đi (continuous: chạy liên tục).
// Trả về lý do kết thúc ("TOI DICH" là thành công). cells: số ô đã đi trong lượt này
const char *MazeMapper::exploreTo(int tx, int ty, int ts, bool continuous, uint16_t &cells) {
  tX = tx; tY = ty; tS = ts;
  const char *endReason = "HET SO O TOI DA";
  cells = 0;
  bool calibDone = false;
  for (uint16_t loop = 0; loop < maxCells * 4 + 8 && cells < maxCells; loop++) {
    if (robotNav.i2cFault) { endReason = "LOI BUS I2C"; break; }
    if (inTarget(x, y)) { endReason = "TOI DICH"; break; }

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
    if (printMap && Serial) Serial.print(draw());

    // 2. Flood-fill (tường chưa biết coi như thông) -> chọn hướng có bước nhỏ nhất, ưu tiên đi thẳng
    maze.floodfill_update(tX, tY, false, false, tS);
    if (maze.cell(x, y).step >= 65535) { endReason = "KHONG CON DUONG TOI DICH"; break; }
    int best = chooseDir(x, y, h);
    if (best < 0) { endReason = "BI KET"; break; }

    // 3. Đi hoặc quay
    int rel = (best - h + 4) % 4;
    robotNav.navPrintf(">> [MAP] O (%d,%d) huong %s -> %s\n", x, y, DIR_NAME[h],
                       rel == 0 ? "DI THANG" : rel == 1 ? "QUAY PHAI" : rel == 3 ? "QUAY TRAI" : "QUAY DAU");
    bool ok = true;
    if (rel == 0 && continuous) {
      cells += runStraight(maxCells - cells);
    } else if (rel == 0) {
      diThang(1);
      setWall(x, y, h, false, true);  // vừa đi qua: chắc chắn thông
      x += DX[h]; y += DY[h];
      maze.cell(x, y).run_visited = true;
      // Vách hông của ô vừa vào: đếm mẫu VL53 trái/phải trong lúc chạy (giữa quãng đường ô)
      WallState sl = robotNav.movingSideWall(true), sr = robotNav.movingSideWall(false);
      robotNav.navPrintf(">> [MAP] VACH HONG O (%d,%d) khi chay: TRAI=%s (gan %u/xa %u) PHAI=%s (gan %u/xa %u)\n", x, y,
                         sl == WS_WALL ? "TUONG" : sl == WS_OPEN ? "THONG" : "?", robotNav.sideNearL, robotNav.sideFarL,
                         sr == WS_WALL ? "TUONG" : sr == WS_OPEN ? "THONG" : "?", robotNav.sideNearR, robotNav.sideFarR);
      if (sl != WS_UNKNOWN) senseWall(x, y, (h + 3) % 4, sl == WS_WALL, true);
      if (sr != WS_UNKNOWN) senseWall(x, y, (h + 1) % 4, sr == WS_WALL, true);
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

  return endReason;
}

void MazeMapper::finish(const char *reason, uint16_t cells) {
  phanh();
  report = draw();
  robotNav.navPrintf(">> [MAP] KET THUC: %s | o (%d,%d) | da di %u o\n", reason, x, y, cells);
  if (printMap && Serial) Serial.print(report);
  reportPending = printMap && !(bool)Serial;  // chạy rời USB: in khi cắm lại (luôn xem được bằng lệnh MAP)
  robotNav.navReportPending = true;  // xe dừng là tự in nhật ký (sau log chạy thẳng) để luôn nằm cuối terminal
}

void MazeMapper::explore(bool continuous) {
  begin();
  uint16_t cells = 0;
  const char *r = exploreTo(goalX, goalY, goalSize, continuous, cells);
  finish(r, cells);
}

// SEQ_MODE 11: dò tới đích -> dò ngược về ô xuất phát (khám phá thêm đường) -> speedrun đường ngắn nhất tới đích
void MazeMapper::fullRun() {
  begin();
  uint16_t c1 = 0, c2 = 0;
  const char *r = exploreTo(goalX, goalY, goalSize, true, c1);
  robotNav.navPrintf(">> [MAP] LUOT 1 (TOI DICH): %s | %u o\n", r, c1);
  if (strcmp(r, "TOI DICH") != 0) { finish(r, c1); return; }
  cho(300);
  r = exploreTo(0, 0, 1, true, c2);
  robotNav.navPrintf(">> [MAP] LUOT 2 (VE XUAT PHAT): %s | %u o\n", r, c2);
  if (strcmp(r, "TOI DICH") != 0) { finish(r, c1 + c2); return; }
  if (printMap && Serial) Serial.print(draw());
  cho(500);
  r = speedRun() ? "SPEEDRUN TOI DICH" : "SPEEDRUN LOI";
  finish(r, c1 + c2);
}

// ================= SPEEDRUN =================

// Chỉ đi qua tường ĐÃ BIẾT: tường chưa đọc coi như CÓ tường (an toàn). Gộp các ô thẳng thành 1 lần chạy liên tục,
// quay tại chỗ ở chỗ rẽ. Không đọc / vẽ tường nữa.
bool MazeMapper::speedRun() {
  static ParentMaze km;
  km.clear_mem();
  for (int cx = 0; cx < MAZE_SIZE; cx++)
    for (int cy = 0; cy < MAZE_SIZE; cy++)
      for (int d = 0; d < 4; d++)
        wallRef(km.cell(cx, cy), d) = !isKnown(cx, cy, d) || hasWall(cx, cy, d);
  // Tìm đường nhanh nhất (A*/Dijkstra trên trạng thái ô + HƯỚNG xe): chi phí = số ô + speedTurnCost ô cho mỗi lần
  // quay 90 độ (quay đầu = 2 lần). Cùng hướng đến 1 ô mới là cùng trạng thái -> phạt rẽ tính đúng.
  // speedTurnCost = 0: đường ít ô nhất (như flood-fill cũ). Chỉ đi qua tường ĐÃ BIẾT là thông.
  const int NS = MAZE_SIZE * MAZE_SIZE * 4;
  static uint16_t cost[NS];
  static int16_t prev[NS];
  static bool done[NS];
  const uint16_t INF = 65535;
  const int CELL_C = 10, TURN_C = (int)lroundf(speedTurnCost * 10.0f);
  auto sid = [](int cx, int cy, int d) { return (cx * MAZE_SIZE + cy) * 4 + d; };
  for (int i = 0; i < NS; i++) { cost[i] = INF; prev[i] = -1; done[i] = false; }
  cost[sid(x, y, h)] = 0;
  int goalState = -1;
  for (;;) {
    int cur = -1;
    uint16_t cmin = INF;
    for (int i = 0; i < NS; i++)
      if (!done[i] && cost[i] < cmin) { cmin = cost[i]; cur = i; }
    if (cur < 0) break;
    done[cur] = true;
    int cd = cur % 4, cy = (cur / 4) % MAZE_SIZE, cx = cur / 4 / MAZE_SIZE;
    if (inGoal(cx, cy)) { goalState = cur; break; }
    for (int d = 0; d < 4; d++) {
      int nx = cx + DX[d], ny = cy + DY[d];
      if (wallRef(km.cell(cx, cy), d) || !inMaze(nx, ny)) continue;
      int rel = (d - cd + 4) % 4;
      int turns = rel == 0 ? 0 : (rel == 2 ? 2 : 1);
      uint32_t nc = (uint32_t)cost[cur] + CELL_C + turns * TURN_C;
      int ns = sid(nx, ny, d);
      if (nc < cost[ns]) { cost[ns] = (uint16_t)min(nc, (uint32_t)(INF - 1)); prev[ns] = cur; }
    }
  }
  if (goalState < 0) {
    robotNav.navPrintf(">> [SPEED] KHONG CO DUONG DA BIET TOI DICH\n");
    return false;
  }

  // Lần ngược prev -> danh sách hướng
  uint8_t path[MAZE_SIZE * MAZE_SIZE];
  int n = 0;
  for (int st = goalState; prev[st] >= 0 && n < MAZE_SIZE * MAZE_SIZE; st = prev[st]) path[n++] = st % 4;
  for (int i = 0; i < n / 2; i++) { uint8_t t = path[i]; path[i] = path[n - 1 - i]; path[n - 1 - i] = t; }
  int nTurns = 0;
  for (int i = 0, ph = h; i < n; ph = path[i], i++) if (path[i] != ph) nTurns++;
  String s = ">> [SPEED] DUONG " + String(n) + " O, " + String(nTurns) + " LAN RE (phat re " + String(speedTurnCost, 1) + " o): ";
  for (int i = 0; i < n; i++) s += "NESW"[path[i]];
  robotNav.navPrintf("%s\n", s.c_str());

  uint8_t savedSpeed = robotNav.baseForwardSpeed;
  if (speedrunSpeed > 0) robotNav.baseForwardSpeed = speedrunSpeed;
  bool ok = true;
  for (int i = 0; i < n && ok && !robotNav.i2cFault;) {
    int d = path[i];
    int rel = (d - h + 4) % 4;
    // Trước khi quay: có tường trước (đã biết) thì căn khoảng cách cho đúng tâm ô
    if (rel != 0 && alignFront && isKnown(x, y, h) && hasWall(x, y, h)) canTuongTruoc();
    if (rel == 1) ok = quayPhai();
    else if (rel == 3) ok = quayTrai();
    else if (rel == 2) ok = quayDau();
    h = d;
    // Sau khi quay (và ở ô xuất phát): sau lưng có tường đã biết -> lùi kịch tường, calib tâm ô + gyro như lúc dò
    int back = (h + 2) % 4;
    if (ok && calibBack && (rel != 0 || i == 0) && isKnown(x, y, back) && hasWall(x, y, back)) {
      robotNav.navPrintf(">> [SPEED] O (%d,%d): LUI CALIB\n", x, y);
      ok = calibNgoCut();
    }
    int run = 0;
    while (i < n && path[i] == d) { run++; i++; }
    diThang(run);  // gộp cả đoạn thẳng: chạy liên tục, chỉ phanh ở ô cuối
    x += DX[d] * run; y += DY[d] * run;
  }
  robotNav.baseForwardSpeed = savedSpeed;
  return ok && !robotNav.i2cFault && inGoal(x, y);
}
