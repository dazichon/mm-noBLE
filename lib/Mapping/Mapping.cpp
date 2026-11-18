#include "Mapping.h"
#include "Motion.h"
#include "PresetMaze.h"
#include "MainParams.h"  // SLALOM_EXPLORE_* (mode 5)

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
  memset(openLock, 0, sizeof(openLock));
  memset(visits, 0, sizeof(visits));
  // Biên của mê cung thật
  for (int i = 0; i < cols; i++) { setWall(i, 0, 2, true, true); setWall(i, rows - 1, 0, true, true); }
  for (int i = 0; i < rows; i++) { setWall(0, i, 3, true, true); setWall(cols - 1, i, 1, true, true); }
  x = startX; y = startY; h = 0;
  maze.cell(startX, startY).run_visited = true;
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
  const char *p = presetApec2 ? PRESET_MAZE_APEC2 : presetApec ? PRESET_MAZE_APEC : presetQual ? PRESET_MAZE_QUAL : PRESET_MAZE;
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
  if (presetQual || presetApec || presetApec2) {  // vẽ thẳng hệ toạ độ xe (hàng trên = y 15, cột trái = x 0), hướng ảnh = hướng xe
    int inner = 0;  // số vách trong: 0 = bản đồ chưa vẽ -> không nạp (xe sẽ báo không có đường, không chạy bừa)
    for (int li = 1; li < 2 * MAZE_SIZE; li++)
      for (int c = 1; c < 4 * MAZE_SIZE; c++) { char ch = at(li, c); if (ch == '-' || ch == '|') inner++; }
    if (inner == 0) {
      robotNav.navPrintf(">> [MAP] BAN DO %s CHUA VE VACH TRONG -> KHONG NAP\n", presetApec2 ? "APEC 2" : presetApec ? "APEC" : "VONG LOAI");
      return false;
    }
    for (int r = 0; r < MAZE_SIZE; r++)
      for (int c = 0; c < MAZE_SIZE; c++) {
        int cy = MAZE_SIZE - 1 - r;
        setWall(c, cy, 0, at(2 * r, 4 * c + 2) == '-', true);
        setWall(c, cy, 2, at(2 * r + 2, 4 * c + 2) == '-', true);
        setWall(c, cy, 3, at(2 * r + 1, 4 * c) == '|', true);
        setWall(c, cy, 1, at(2 * r + 1, 4 * c + 4) == '|', true);
      }
    presetLoaded = true;
    robotNav.navPrintf(">> [MAP] DA NAP BAN DO %s 16x16\n", presetApec2 ? "APEC 2 (PRESET_MAZE_APEC2)" : presetApec ? "APEC (PRESET_MAZE_APEC)" : "VONG LOAI (PRESET_MAZE_QUAL)");
    return true;
  }
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
  robotNav.saveRunLog();  // lưu flash: chạy rời USB rồi cắm máy tính (kể cả bị reset khi mở serial) vẫn in được
  robotNav.navReportPending = true;  // xe dừng là tự in nhật ký (sau log chạy thẳng) để luôn nằm cuối terminal
}

void MazeMapper::explore(bool continuous) {
  begin();
  uint16_t cells = 0;
  const char *r = exploreTo(goalX, goalY, goalSize, continuous, cells);
  finish(r, cells);
}

// SEQ_MODE 1: dò tới đích -> dò ngược về ô xuất phát (khám phá thêm đường) -> speedrun đường ngắn nhất tới đích
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

// ================= MODE 5: DÒ ĐƯỜNG BẰNG CUA SLALOM =================
// Xe chạy liên tục. Tại "điểm quyết định" của mỗi ô (cách tâm ô F ~ 68 mm, chính là điểm bắt đầu cua) đã biết đủ vách của ô đó:
//  - vách hông: đếm mẫu VL53 trái/phải ở đoạn thẳng vừa chạy (tia 45 độ nhìn trước 1 ô)
//  - vách trước: VL53 trước đọc <= FRONT_ALIGN_MM + F (thấy tường) hoặc xa hơn tầm tin cậy (thông)
// rồi flood-fill chọn hướng: đi thẳng tiếp / cua Slalom trái-phải (chỉ khi vách phía đó ĐÃ BIẾT là thông) /
// còn lại (ngõ cụt, chưa chắc vách) thì dừng ở tâm ô, quay tại chỗ như mode 1.
static float headingErrDeg();

const char *MazeMapper::exploreToSlalom(int tx, int ty, int ts, uint16_t &cells) {
  tX = tx; tY = ty; tS = ts;
  cells = 0;
  const float F = RobotNav::slalomAdvanceMm(), CELL = robotNav.cellMm;
  bool atRest = true;        // true: xe đứng yên ở tâm ô (x,y). false: xe đang chạy, ở điểm quyết định của ô (x,y), cách tâm F
  bool frontWall = false;    // (đang chạy) đã thấy tường trước của ô (x,y)
  const char *endReason = "HET SO O TOI DA";

  // Chạy 1 đoạn thẳng DÒ tới điểm quyết định của ô kế (theo hướng h hiện tại), rồi cập nhật bản đồ:
  // ô vừa qua thông, vách hông + tường trước của ô mới
  auto runLeg = [&](float mm, bool moving, float p0) -> bool {
    robotNav.slExplore = true;
    robotNav.slPStartMm = p0;
    bool ok = robotNav.slalomStraight(mm, moving, true, false);
    robotNav.slExplore = false;
    robotNav.navPrintf(">> [DOAN] %.0f mm | di %.0f mm | ket thuc: %s | VL53 truoc %d mm\n", mm, robotNav.slLegMm, robotNav.slLegEnd, robotNav.slLegFront);
    if (!ok) return false;
    setWall(x, y, h, false, true);  // vừa đi qua: chắc chắn thông
    openLock[x][y] |= (1 << h);      // cạnh đã đi xuyên qua -> khóa thông, mẫu hông nhiễu sau đó không được ghi đè thành tường
    { int nx = x + DX[h], ny = y + DY[h];
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) openLock[nx][ny] |= (1 << ((h + 2) % 4)); }
    x += DX[h]; y += DY[h];
    maze.cell(x, y).run_visited = true;
    if (visits[x][y] < 255) visits[x][y]++;
    cells++;
    if (roam) {
      float yawErr = robotNav.mpu6050.getYaw() - robotNav._turnHeading;
      while (yawErr > 180.0f) yawErr -= 360.0f;
      while (yawErr < -180.0f) yawErr += 360.0f;
      robotNav.navPrintf(">> [LECH] o thu %u | %.1f s | yaw lech %.1f do | VL53 truoc %d mm | L/R luc cuoi %d/%d mm\n", cells, (millis() - roamT0) / 1000.0f, yawErr,
                         robotNav.slLegFront, (int)robotNav._rawL, (int)robotNav._rawR);
    }
    WallState sl = robotNav.movingSideWall(true), sr = robotNav.movingSideWall(false);
    robotNav.navPrintf(">> [MAP] O (%d,%d) vach hong: TRAI=%s PHAI=%s | tuong truoc=%s\n", x, y, sl == WS_WALL ? "TUONG" : sl == WS_OPEN ? "THONG" : "?",
                       sr == WS_WALL ? "TUONG" : sr == WS_OPEN ? "THONG" : "?", robotNav.slLegWall ? "CO" : "KHONG");
    robotNav.navPrintf(">> [MAP]   mau hong (gan/xa): TRAI %u/%u PHAI %u/%u\n", robotNav.sideNearL, robotNav.sideFarL, robotNav.sideNearR, robotNav.sideFarR);
    for (int off = 0; off < (int)robotNav.slSideTrace.length(); off += 150)   // navPrintf chỉ in được ~200 ký tự mỗi lần
      robotNav.navPrintf(">> [VET] p:L/R%s\n", robotNav.slSideTrace.substring(off, off + 150).c_str());
    // Quy tắc mode 5: xe chạy nhanh, mẫu "có tường" hay sai (tia chéo chiếu vào vách ô vừa rời) còn "thông" thì đáng tin.
    // Hễ có >= SLALOM_EXPLORE_OPEN_MIN_SAMPLES mẫu thấy THÔNG ở một bên -> ghi thông và KHÓA, mẫu có tường đến sau không được sửa.
    auto sideRule = [&](bool left, int dir) {
      uint8_t farN = left ? robotNav.sideFarL : robotNav.sideFarR;
      if (farN >= SLALOM_EXPLORE_OPEN_MIN_SAMPLES) {
        setWall(x, y, dir, false, true);
        openLock[x][y] |= (1 << dir);
        int nx = x + DX[dir], ny = y + DY[dir];
        if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) openLock[nx][ny] |= (1 << ((dir + 2) % 4));
      } else if ((left ? sl : sr) == WS_WALL && !(openLock[x][y] & (1 << dir))) {
        setWall(x, y, dir, true, true);
      }
    };
    sideRule(true, (h + 3) % 4);
    sideRule(false, (h + 1) % 4);
    frontWall = robotNav.slLegWall;
    atRest = false;
    return true;
  };

  for (uint16_t loop = 0; loop < maxCells * 4 + 8 && cells < maxCells; loop++) {
    if (robotNav.i2cFault) { endReason = "LOI BUS I2C"; break; }
    if (!roam && inTarget(x, y)) {
      if (!atRest) {   // tới điểm quyết định của ô đích: đi nốt tới tâm ô rồi dừng
        robotNav.slalomStraight(F, true, false, frontWall);
        if (frontWall && alignFront) canTuongTruoc();
      }
      endReason = "TOI DICH";
      break;
    }
    if (atRest) {
      WallState l, f, r;
      docTuong(l, f, r);
      if (f == WS_UNKNOWN) { endReason = "VL53 TRUOC KHONG DANG TIN"; break; }
      setWall(x, y, h, f == WS_WALL, true);
      frontWall = (f == WS_WALL);
      if (frontWall && alignFront) canTuongTruoc();
      if (printMap && Serial) Serial.print(draw());
    } else {
      setWall(x, y, h, frontWall, true);
    }

    int best;
    if (roam) {
      best = chooseRoam(x, y, h);
    } else {
      maze.floodfill_update(tX, tY, false, false, tS);
      if (maze.cell(x, y).step >= 65535) { endReason = "KHONG CON DUONG TOI DICH"; break; }
      best = chooseDir(x, y, h);
    }
    if (best < 0) { endReason = "BI KET"; break; }
    int rel = (best - h + 4) % 4;
    robotNav.navPrintf(">> [MAP] O (%d,%d) huong %s -> %s%s\n", x, y, DIR_NAME[h],
                       rel == 0 ? "DI THANG" : rel == 1 ? "CUA PHAI" : rel == 3 ? "CUA TRAI" : "QUAY DAU", atRest ? " (dung yen)" : " (dang chay)");
    bool ok = true;
    if (atRest) {
      if (rel == 0) ok = runLeg(CELL - F, false, 0.0f);          // từ tâm ô, đi tới điểm quyết định của ô kế
      else if (rel == 1) { ok = quayPhai(); h = (h + 1) % 4; }     // đứng yên: quay tại chỗ rồi đọc lại tường trước
      else if (rel == 3) { ok = quayTrai(); h = (h + 3) % 4; }
      else {
        ok = quayDau(); h = (h + 2) % 4;
        // Quay đầu xong: sau lưng là vách trước vừa đọc (đã biết có tường) -> lùi kịch tường căn lại tâm ô + đặt MPU về 0 độ
        int bk = (h + 2) % 4;
        if (ok && isKnown(x, y, bk) && hasWall(x, y, bk)) {
          robotNav.navPrintf(">> [MAP] O (%d,%d): QUAY DAU XONG -> LUI KICH TUONG CAN LAI, RESET MPU 0 DO\n", x, y);
          ok = calibNgoCut();
        }
      }
    } else if (rel == 0) {
      ok = runLeg(CELL, true, -F);                                // thẳng tiếp: điểm quyết định ô này -> điểm quyết định ô kế
    } else if ((rel == 1 || rel == 3) && !hasWall(x, y, best)) {   // có đường (đã biết thông hoặc chưa biết) -> cứ cua đi liên tục, không dừng
      ok = robotNav.slalomTurn(rel == 3, true);                   // vách phía rẽ đã biết thông: cua Slalom, không dừng
      h = best;
      // Như speedrun: cua xong lệch hướng > calibBackErrDeg mà sau lưng ô góc có tường -> dừng, lùi kịch tường calib, về tâm ô
      int bk = (h + 2) % 4;
      if (ok && !roam && calibBack && calibBackErrDeg > 0.0f && isKnown(x, y, bk) && hasWall(x, y, bk)) {
        float err = headingErrDeg();
        if (err > calibBackErrDeg) {
          robotNav.navPrintf(">> [MAP] O (%d,%d): CUA LECH %.1f do -> DUNG + LUI CALIB\n", x, y, err);
          phanh();
          ok = robotNav.calibCellFromBackWall(false);   // lùi kịch tường, yaw = 0, tiến về tâm ô
          atRest = true;
          if (!ok) { endReason = "LOI KHI CHAY"; break; }
          continue;                                        // vòng sau đọc tường ở tâm ô rồi đi tiếp
        }
      }
      if (ok) ok = runLeg(CELL - 2.0f * F, true, F);              // sau cua xe đã tiến F: chạy nốt tới điểm quyết định ô kế
    } else {
      // cụt, hoặc chưa chắc vách phía rẽ: dừng ở tâm ô, vòng sau đọc/quay tại chỗ như mode 1
      robotNav.navPrintf(">> [MAP] O (%d,%d): DUNG o tam o (%s)\n", x, y, rel == 2 ? "cut" : "chua chac vach ben re");
      robotNav.slalomStraight(F, true, false, frontWall);
      if (frontWall && alignFront) canTuongTruoc();
      atRest = true;
    }
    if (robotNav.frontEmergency) { endReason = "PHANH KHAN gan tuong truoc"; break; }
    if (!ok) { endReason = "LOI KHI CHAY"; break; }
  }
  return endReason;
}

// Chạy tự do (mode 5, không đích): tại mỗi ngã rẽ chọn ngẫu nhiên giữa các hướng KHÔNG có tường, ưu tiên ô đã qua ít lần nhất.
// Quay đầu chỉ khi cả 3 hướng trước/trái/phải đều bị chặn.
int MazeMapper::chooseRoam(int cx, int cy, int ch) {
  int cand[3], n = 0, minV = 1000;
  for (int k = 0; k < 4; k++) {
    int d = (ch + (k == 0 ? 0 : k == 1 ? 1 : k == 2 ? 3 : 2)) % 4;   // thẳng, phải, trái, rồi mới quay đầu
    if (k == 3) { if (n == 0 && !hasWall(cx, cy, d) && inMaze(cx + DX[d], cy + DY[d])) return d; break; }
    int nx = cx + DX[d], ny = cy + DY[d];
    if (hasWall(cx, cy, d) || !inMaze(nx, ny)) continue;
    int v = visits[nx][ny];
    if (v < minV) { minV = v; n = 0; }
    if (v == minV) cand[n++] = d;
  }
  if (n == 0) return -1;
  return cand[random(n)];
}

// ================= MODE 6: CHẠY TỰ DO nCells Ô =================
// Quy tắc cứng:
//  - Xe CHẠY LIÊN TỤC. Mọi ngã rẽ đều là cua Slalom khi đang chạy. KHÔNG quay 90 tại chỗ, KHÔNG căn tường trước, KHÔNG lùi kịch tường.
//  - Chỉ NGÕ CỤT (trước có tường, hai bên không chắc chắn thông) mới được dừng ở tâm ô và quay 180 tại chỗ, rồi chạy tiếp.
//  - Vách hông CHƯA chắc chắn thì coi là TƯỜNG (đi nhầm vào tường nguy hiểm hơn nhiều so với bỏ lỡ một đường thông).
//    "Thông" chỉ khi >= 2 mẫu liên tiếp trong cửa sổ đọc (xem NavSlalom.cpp) và mẫu thông nhiều hơn mẫu tường.
//  - Phanh khẩn / lỗi đoạn chạy = coi như ngõ cụt đột xuất: dừng, quay 180, chạy tiếp (chỉ bỏ cuộc nếu liên tiếp > 4 lần hoặc lỗi I2C).
void MazeMapper::roamSlalom(uint16_t nCells) {
  robotNav.slalomLogReset();
  begin();
  robotNav.frontGuard = true;
  robotNav.distUseMax = true;
  roam = true;
  roamT0 = millis();
  robotNav.slExtendMm = ROAM_FRONT_EXTEND_MM;
  const long saveCalibFwd = robotNav.calibFwdPulses;
  robotNav.calibFwdPulses = saveCalibFwd + ROAM_CALIB_EXTRA_PULSES;
  robotNav.frontCovered = false;
  const float F = RobotNav::slalomAdvanceMm(), CELL = robotNav.cellMm;
  uint16_t cells = 0, fails = 0;
  const char *endReason = "XONG";
  bool frontWall = false, lOpen = false, rOpen = false;   // kết quả đọc ở điểm quyết định của ô (x,y) hiện tại

  auto sideOpen = [&](bool left) -> bool {
    uint8_t farN = left ? robotNav.sideFarL : robotNav.sideFarR;
    uint8_t nearN = left ? robotNav.sideNearL : robotNav.sideNearR;
    (void)nearN;
    return farN >= 1;  // cua Slalom: chỉ cần 1 mẫu THÔNG là coi như không có tường
  };

  // Chạy 1 đoạn thẳng tới điểm quyết định của ô kế, cập nhật bản đồ + kết quả đọc
  auto runLeg = [&](float mm, bool moving, float p0) -> bool {
    robotNav.slExplore = true;
    robotNav.slPStartMm = p0;
    bool ok = robotNav.slalomStraight(mm, moving, true, false);
    robotNav.slExplore = false;
    robotNav.navPrintf(">> [DOAN] %.0f mm | di %.0f mm | ket thuc: %s | VL53 truoc luc dau %d -> luc cuoi %d mm\n", mm, robotNav.slLegMm, robotNav.slLegEnd, robotNav.slFront0, robotNav.slLegFront);
    if (!ok) return false;
    setWall(x, y, h, false, true);   // vừa đi qua: chắc chắn thông
    openLock[x][y] |= (1 << h);
    int nx = x + DX[h], ny = y + DY[h];
    if (inMaze(nx, ny)) openLock[nx][ny] |= (1 << ((h + 2) % 4));
    x = nx; y = ny;
    maze.cell(x, y).run_visited = true;
    if (visits[x][y] < 255) visits[x][y]++;
    cells++;
    float yawErr = robotNav.mpu6050.getYaw() - robotNav._turnHeading;
    while (yawErr > 180.0f) yawErr -= 360.0f;
    while (yawErr < -180.0f) yawErr += 360.0f;
    robotNav.navPrintf(">> [LECH] o thu %u | %.1f s | yaw lech %.1f do | VL53 truoc %d mm | L/R luc cuoi %d/%d mm\n", cells, (millis() - roamT0) / 1000.0f, yawErr,
                       robotNav.slLegFront, (int)robotNav._rawL, (int)robotNav._rawR);
    frontWall = robotNav.slLegWall;
    lOpen = sideOpen(true);
    rOpen = sideOpen(false);
    const int dl = (h + 3) % 4, dr = (h + 1) % 4;
    // Ghi bản đồ (chỉ để hiển thị / thống kê): thông xác nhận -> ghi thông + khóa; có nhiều mẫu tường -> ghi tường
    if (lOpen) { setWall(x, y, dl, false, true); openLock[x][y] |= (1 << dl); }
    else if (robotNav.sideNearL > 0 && !(openLock[x][y] & (1 << dl))) setWall(x, y, dl, true, true);
    if (rOpen) { setWall(x, y, dr, false, true); openLock[x][y] |= (1 << dr); }
    else if (robotNav.sideNearR > 0 && !(openLock[x][y] & (1 << dr))) setWall(x, y, dr, true, true);
    setWall(x, y, h, frontWall, true);
    robotNav.navPrintf(">> [MAP] O (%d,%d) h=%s: TRAI=%s PHAI=%s TRUOC=%s | mau hong TRAI %u/%u PHAI %u/%u\n", x, y, DIR_NAME[h],
                       lOpen ? "THONG" : "tuong", rOpen ? "THONG" : "tuong", frontWall ? "TUONG" : "THONG",
                       robotNav.sideNearL, robotNav.sideFarL, robotNav.sideNearR, robotNav.sideFarR);
    for (int off = 0; off < (int)robotNav.slSideTrace.length(); off += 150)
      robotNav.navPrintf(">> [VET] p:L/R%s\n", robotNav.slSideTrace.substring(off, off + 150).c_str());
    return true;
  };

  // Sự cố (phanh khẩn / lỗi đoạn): coi như ngõ cụt đột xuất -> dừng, quay 180 tại chỗ. false = bỏ cuộc
  auto recover = [&](const char *why) -> bool {
    if (robotNav.frontCovered) { robotNav.stopMotors(); return false; }   // che mắt trước: không phục hồi, dừng hẳn
    robotNav.navPrintf(">> [ROAM] %s o (%d,%d) h=%s -> QUAY 180 (lan %u)\n", why, x, y, DIR_NAME[h], fails + 1);
    robotNav.stopMotors();
    robotNav.frontEmergency = false;
    setWall(x, y, h, true, true);
    if (++fails > 4 || robotNav.i2cFault) return false;
    if (!quayDau()) return false;
    h = (h + 2) % 4;
    return calibNgoCut();   // sau lưng là tường vừa suýt đâm: lùi kịch tường căn lại tâm ô + reset gyro
  };

  bool atRest = true;   // đầu cuộc chạy: đứng yên ở tâm ô xuất phát
  while (cells < nCells) {
    if (robotNav.i2cFault) { endReason = "LOI BUS I2C"; break; }
    if (robotNav.frontCovered) { endReason = "CHE MAT TRUOC - DUNG KHAN CAP"; break; }
    if (atRest) {   // từ tâm ô đi tới điểm quyết định của ô kế
      atRest = false;
      if (!runLeg(CELL - F, false, 0.0f)) {
        if (!recover(robotNav.frontEmergency ? "PHANH KHAN" : "LOI DOAN")) { endReason = robotNav.frontCovered ? "CHE MAT TRUOC - DUNG KHAN CAP" : "KET CUNG"; break; }
        atRest = true;
      } else fails = 0;
      continue;
    }
    // Điểm quyết định: chọn trong các hướng ĐÃ XÁC NHẬN thông, ưu tiên ô ít đi nhất, hoà thì ngẫu nhiên
    int cand[3], n = 0, minV = 1000;
    const int dirs[3] = {h, (h + 1) % 4, (h + 3) % 4};
    const bool isOpen[3] = {!frontWall, rOpen, lOpen};
    for (int k = 0; k < 3; k++) {
      int nx = x + DX[dirs[k]], ny = y + DY[dirs[k]];
      if (!isOpen[k] || !inMaze(nx, ny)) continue;
      int v = visits[nx][ny];
      if (v < minV) { minV = v; n = 0; }
      if (v == minV) cand[n++] = dirs[k];
    }
    int best = n ? cand[random(n)] : -1;
    int rel = best < 0 ? 2 : (best - h + 4) % 4;
    robotNav.navPrintf(">> [MAP] O (%d,%d) huong %s -> %s\n", x, y, DIR_NAME[h], rel == 0 ? "DI THANG" : rel == 1 ? "CUA PHAI" : rel == 3 ? "CUA TRAI" : "NGO CUT: QUAY 180");
    bool ok = true;
    if (rel == 0) {
      ok = runLeg(CELL, true, -F);
    } else if (rel == 1 || rel == 3) {
      ok = robotNav.slalomTurn(rel == 3, true);
      if (ok) { h = best; ok = runLeg(CELL - 2.0f * F, true, F); }
    } else {
      // Ngõ cụt thật sự: đi nốt tới tâm ô, phanh dừng (không căn tường, không lùi), quay 180 tại chỗ, rồi chạy tiếp
      ok = robotNav.slalomStraight(F, true, false, true);
      if (ok) ok = quayDau();
      if (ok) {
        h = (h + 2) % 4;
        // Sau lưng giờ là vách ngõ cụt: lùi kịch tường căn lại tâm ô + calib gyro (yaw về 0), rồi mới chạy tiếp
        robotNav.navPrintf(">> [MAP] O (%d,%d): NGO CUT -> LUI KICH TUONG CAN LAI, RESET MPU 0 DO\n", x, y);
        ok = calibNgoCut();
        atRest = true;
      }
    }
    if (!ok) {
      if (!recover(robotNav.frontEmergency ? "PHANH KHAN" : "LOI DOAN")) { endReason = robotNav.frontCovered ? "CHE MAT TRUOC - DUNG KHAN CAP" : "KET CUNG"; break; }
      atRest = true;
    } else if (rel != 2) fails = 0;
  }
  roam = false;
  robotNav.calibFwdPulses = saveCalibFwd;
  robotNav.slExtendMm = SLALOM_FRONT_EXTEND_MM;
  robotNav.navPrintf(">> [MAP] CHAY TU DO XONG: %s | %u o | %.1f s\n", endReason, cells, (millis() - roamT0) / 1000.0f);
  robotNav.frontGuard = false;
  robotNav.distUseMax = false;
  finish(endReason, cells);
}

// SEQ_MODE 5: dò tới đích -> dò về xuất phát (đều bằng cua) -> speedrun đường ngắn nhất bằng cua
void MazeMapper::fullRunSlalom() {
  robotNav.slalomLogReset();
  begin();
  robotNav.frontGuard = true;       // VL53 trước phanh khẩn khi quá sát tường
  robotNav.distUseMax = true;
  const uint8_t exploreSaved = robotNav.baseForwardSpeed;
  if (SLALOM_EXPLORE_PWM > 0) robotNav.baseForwardSpeed = SLALOM_EXPLORE_PWM;  // mode 5: dò chậm để VL53 đọc tường chuẩn hơn
  // Như mode 6: hết encoder mà VL53 trước còn thấy tường của ô này -> đi tiếp tới ngưỡng cua (log: cap 55 mm cua sớm ~60 mm)
  robotNav.slExtendMm = ROAM_FRONT_EXTEND_MM;
  uint16_t c1 = 0, c2 = 0;
  const char *r = exploreToSlalom(goalX, goalY, goalSize, c1);
  robotNav.navPrintf(">> [MAP] LUOT 1 (TOI DICH): %s | %u o\n", r, c1);
  if (strcmp(r, "TOI DICH") != 0) { robotNav.slExtendMm = SLALOM_FRONT_EXTEND_MM; robotNav.baseForwardSpeed = exploreSaved; robotNav.frontGuard = false; robotNav.distUseMax = false; finish(r, c1); return; }
  cho(300);
  r = exploreToSlalom(startX, startY, 1, c2);
  robotNav.navPrintf(">> [MAP] LUOT 2 (VE XUAT PHAT): %s | %u o\n", r, c2);
  robotNav.baseForwardSpeed = exploreSaved;  // hết dò: trả PWM gốc (speedRun tự đặt MAP_SPEEDRUN_PWM)
  robotNav.slExtendMm = SLALOM_FRONT_EXTEND_MM;
  if (strcmp(r, "TOI DICH") != 0) { robotNav.baseForwardSpeed = exploreSaved; robotNav.frontGuard = false; robotNav.distUseMax = false; finish(r, c1 + c2); return; }
  if (printMap && Serial) Serial.print(draw());
  cho(500);
  r = speedRun() ? "SPEEDRUN TOI DICH" : "SPEEDRUN LOI";
  robotNav.frontGuard = false;
  robotNav.distUseMax = false;
  finish(r, c1 + c2);
}

// SEQ_MODE 2: BIẾT TRƯỚC toàn bộ map (PresetMaze.h) -> không dò, tính đường ngắn nhất rồi chạy
void MazeMapper::runKnownMap() {
  robotNav.slalomLogReset();
  begin();
  robotNav.distUseMax = true;  // encoder một bên mất xung (log thấy ENC_R đứng yên) thì lấy bên đi xa hơn
  robotNav.frontGuard = true;  // VL53 trước phanh khẩn khi quá sát tường
  const char *r = speedRun() ? "CHAY MAP BIET TOI DICH" : "CHAY MAP BIET LOI";
  robotNav.frontGuard = false;
  robotNav.distUseMax = false;
  finish(r, 0);
}

// ================= SPEEDRUN =================

// Chỉ đi qua tường ĐÃ BIẾT: tường chưa đọc coi như CÓ tường (an toàn). Gộp các ô thẳng thành 1 lần chạy liên tục,
// quay tại chỗ ở chỗ rẽ. Không đọc / vẽ tường nữa.
// Độ lệch (độ, trị tuyệt đối) giữa hướng MPU thực và hướng chuẩn lý tưởng sau các lần quay / cua
static float headingErrDeg() {
  if (!robotNav.mpuReady || !robotNav._turnHeadingValid) return 0.0f;
  robotNav.mpu6050.update();
  float e = (robotNav.straightUseGyro ? robotNav._turnHeading : robotNav._absHeading) - robotNav.mpu6050.getYaw();
  while (e > 180.0f) e -= 360.0f;
  while (e < -180.0f) e += 360.0f;
  return fabsf(e);
}

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
  long extraFwd = 0;
  int turnsSinceCalib = 0;   // số lần quay/cua từ lần lùi calib gần nhất (hoặc từ lúc xuất phát)
  // Có nên lùi calib không: lệch > SPEED_CALIB_FORCE_DEG -> lùi ngay; lệch > calibBackErrDeg chỉ khi đã quay/cua >= SPEED_CALIB_MIN_TURNS lần
  auto needCalib = [&](float err) -> bool {
    if (err > SPEED_CALIB_FORCE_DEG) return true;
    return err > calibBackErrDeg && turnsSinceCalib >= SPEED_CALIB_MIN_TURNS;
  };  // xung cộng vào đoạn kế tiếp sau khi lùi calib không tiến về tâm ô
  bool afterSlalom = false;  // vừa cua Slalom xong: xe đã tiến sẵn advPulses trên đoạn thẳng mới
  for (int i = 0; i < n && ok && !robotNav.i2cFault;) {
    int d = path[i];
    int rel = (d - h + 4) % 4;
    // Trước khi quay: có tường trước (đã biết) thì căn khoảng cách cho đúng tâm ô
    if (rel != 0 && alignFront && isKnown(x, y, h) && hasWall(x, y, h)) canTuongTruoc();
    if (rel == 1) ok = quayPhai();
    else if (rel == 3) ok = quayTrai();
    else if (rel == 2) ok = quayDau();
    if (rel != 0) turnsSinceCalib++;
    h = d;
    // Sau khi quay (và ở ô xuất phát): sau lưng có tường đã biết -> lùi kịch tường, calib tâm ô + gyro như lúc dò
    int back = (h + 2) % 4;
    if (ok && calibBack && (rel != 0 || i == 0) && isKnown(x, y, back) && hasWall(x, y, back)) {
      float err = calibBackErrDeg > 0.0f ? headingErrDeg() : 0.0f;
      if (calibBackErrDeg <= 0.0f || needCalib(err)) {
        robotNav.navPrintf(">> [SPEED] O (%d,%d): LUI CALIB (lech %.1f do, %d lan re tu lan calib truoc)\n", x, y, err, turnsSinceCalib);
        turnsSinceCalib = 0;
        if (calibBackErrDeg > 0.0f) { ok = robotNav.calibCellFromBackWall(true); extraFwd = robotNav.calibFwdPulses; }  // không dừng ở tâm ô
        else ok = calibNgoCut();
      } else robotNav.navPrintf(">> [SPEED] O (%d,%d): lech %.1f do, %d lan re -> bo qua calib\n", x, y, err, turnsSinceCalib);
    }
    int run = 0;
    while (i < n && path[i] == d) { run++; i++; }
    int ex = x + DX[d] * run, ey = y + DY[d] * run;  // ô cuối đoạn
    // Mode 2: cuối đoạn rẽ 90° -> cua Slalom khi đang chạy. Cua bắt đầu sớm advPulses trước tâm ô góc (xe tự tiến chừng đó theo
    // hướng cũ) và kết thúc khi đã tiến advPulses theo hướng mới, nên đoạn thẳng trước cua bị rút advPulses, đoạn sau cua cũng vậy
    int nextRel = i < n ? (path[i] - d + 4) % 4 : 0;
    bool intoTurn = slalomTurns && (nextRel == 1 || nextRel == 3);
    long advPulses = (long)lroundf(RobotNav::slalomAdvanceMm() * (float)robotNav.pulsesPerCell / robotNav.cellMm);
    robotNav.legTrimPulses = (afterSlalom ? advPulses : 0) + (intoTurn ? advPulses : 0) - extraFwd;  // vừa lùi calib: đi thêm vách -> tâm ô
    robotNav.frontEmergency = false;
    // Vừa lùi kịch tường calib: xuất phát lại MỀM (PWM thấp hơn, tăng tốc lâu hơn) để không vọt lên làm lệch hướng
    const float savedAccelPwm = robotNav.accelStartPwm;
    const uint16_t savedRampMs = robotNav.accelRampMs;
    robotNav.slNoWallMm = 0.0f;
    if (extraFwd > 0) {
      robotNav.accelStartPwm = CALIB_RESTART_PWM;
      robotNav.accelRampMs = CALIB_RESTART_RAMP_MS;
      robotNav.slNoWallMm = CALIB_RESTART_NO_WALL_MM;
    }
    if (slalomTurns) {
      // Mode 2: đoạn thẳng cũng chạy theo tốc độ encoder (SLALOM_SPEED_MM_S) để vào cua đúng tốc độ thiết kế; PWM chạy thẳng không điều khiển được tốc độ
      float legMm = (float)run * robotNav.cellMm - ((afterSlalom ? 1 : 0) + (intoTurn ? 1 : 0)) * RobotNav::slalomAdvanceMm();
      legMm += (float)extraFwd * robotNav.cellMm / (float)robotNav.pulsesPerCell;  // vừa lùi calib: đi thêm vách -> tâm ô
      if (legMm < 5.0f) legMm = 5.0f;
      robotNav.legTrimPulses = 0;  // không dùng ở nhánh này
      bool wallAhead = isKnown(ex, ey, d) && hasWall(ex, ey, d);   // bản đồ biết có tường trước ô cuối đoạn -> VL53 trước chốt điểm vào cua
      // Chốt điểm cua bằng mép tường hông: đoạn >= 2 ô, ô trước ô góc CÓ tường bên sắp rẽ, ô góc TRỐNG bên đó.
      // Đoạn 1 ô (lối rẽ ngay ô kế, mắt hông đã thấy trống từ đầu) -> cua theo encoder như bình thường
      robotNav.slEdgeSide = 0;
      if (intoTurn && run >= 2) {
        int td = path[i];
        int px = ex - DX[d], py = ey - DY[d];
        if (isKnown(px, py, td) && hasWall(px, py, td) && isKnown(ex, ey, td) && !hasWall(ex, ey, td)) {
          robotNav.slEdgeSide = (nextRel == 3) ? 1 : 2;
          robotNav.slEdgeMinS = max(0.0f, legMm - 1.5f * robotNav.cellMm);
        }
      }
      ok = robotNav.slalomStraight(legMm, afterSlalom, intoTurn, wallAhead);
      if (robotNav.slEdgeSide && robotNav.slEdgeAtMm < 0.0f) robotNav.navPrintf(">> [MEP] KHONG THAY MEP %s -> cua theo encoder\n", robotNav.slEdgeSide == 1 ? "TRAI" : "PHAI");
      robotNav.slEdgeSide = 0;
      robotNav.navPrintf(">> [DOAN] %d o | ke hoach %.0f mm | di %.0f mm | ket thuc: %s | VL53 truoc %d mm%s\n", run, legMm, robotNav.slLegMm,
                         robotNav.slLegEnd, robotNav.slLegFront, wallAhead ? "" : " (khong co tuong truoc)");
    } else {
      robotNav.slalomApproach = intoTurn;
      diThang(run, intoTurn);
      robotNav.slalomApproach = false;
      robotNav.navPrintf(">> [DOAN] %d o | muc tieu %ld xung | di duoc %ld xung | VL53 truoc %.0f mm\n", run,
                         robotNav.stepTargetPulses, robotNav.stepTraveledPulses, robotNav.smoothDF);
    }
    if (robotNav.frontEmergency) { robotNav.navPrintf(">> [SPEED] PHANH KHAN gan tuong truoc -> DUNG\n"); ok = false; }
    robotNav.slNoWallMm = 0.0f;
    robotNav.accelStartPwm = savedAccelPwm;   // trả lại tăng tốc bình thường
    robotNav.accelRampMs = savedRampMs;
    x = ex; y = ey;
    afterSlalom = false;
    extraFwd = 0;
    if (ok && intoTurn) {
      ok = robotNav.slalomTurn(nextRel == 3, true);   // rel 3 = trái, 1 = phải
      if (ok) {
        h = path[i];
        afterSlalom = true;
        turnsSinceCalib++;
        // Cua lệch hướng quá calibBackErrDeg mà sau lưng ô góc có tường -> dừng, lùi kịch tường calib (về tâm ô góc), rồi đi tiếp từ đứng yên
        int bk = (h + 2) % 4;
        if (calibBack && calibBackErrDeg > 0.0f && isKnown(x, y, bk) && hasWall(x, y, bk)) {
          float err = headingErrDeg();
          if (needCalib(err)) {
            robotNav.navPrintf(">> [SPEED] O (%d,%d): CUA LECH %.1f do (%d lan re) -> DUNG + LUI CALIB\n", x, y, err, turnsSinceCalib);
            turnsSinceCalib = 0;
            phanh();
            ok = robotNav.calibCellFromBackWall(true);  // chạm vách + yaw = 0, không tiến về tâm ô
            extraFwd = robotNav.calibFwdPulses;         // đoạn sau đi thêm quãng vách -> tâm ô, chạy liền một mạch
            afterSlalom = false;  // xuất phát từ vách ô góc -> đoạn sau đi đủ ô, không trừ phần cua
          }
        }
      }
      else robotNav.navPrintf(">> [SPEED] CUA SLALOM LOI -> DUNG\n");
    }
  }
  robotNav.baseForwardSpeed = savedSpeed;
  phanh();  // kết thúc (kể cả khi đoạn cuối không tiến): phanh dừng
  // Về đích: tường trước ở ô đích đã biết -> căn lại đúng tâm ô theo VL53 trước (F ~ FRONT_ALIGN_MM), bù cả đi thiếu lẫn đi quá sát
  if (ok && alignFront && isKnown(x, y, h) && hasWall(x, y, h)) canTuongTruoc();  // dùng chung mọi mode

  return ok && !robotNav.i2cFault && inGoal(x, y);
}
