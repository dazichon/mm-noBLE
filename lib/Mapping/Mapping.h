#pragma once
// ###########################################################################################################
// #  DÒ ĐƯỜNG + VẼ BẢN ĐỒ TƯỜNG                                                                              #
// #  Xe đi từng ô: dừng -> đọc tường -> ghi vào bản đồ -> flood-fill chọn hướng -> đi/quay -> lặp lại.       #
// #  Bản đồ in ra Serial dạng ASCII sau mỗi ô (nếu cắm USB) và lưu lại bản cuối để in khi cắm USB sau.       #
// ###########################################################################################################
//
// Toạ độ: (x, y), ô xuất phát (0,0) ở góc dưới trái, xe đặt quay mặt về BẮC (lên trên).
// Hướng: 0 = Bắc (y+1), 1 = Đông (x+1), 2 = Nam (y-1), 3 = Tây (x-1).
//
// Cách cảm biến thấy tường (xe đứng ở tâm ô):
//   - VL53 TRƯỚC  -> vách trước của CHÍNH ô đang đứng.
//   - VL53 TRÁI/PHẢI đặt chéo 45°: đọc KHI ĐANG CHẠY vào ô mới (đoạn MAP_SIDE_FROM..MAP_SIDE_TO quãng đường)
//     -> vách trái/phải của ô vừa vào. Đứng yên ở tâm ô thì tia 45° chạm cột góc nên không dùng.
//
// Ký hiệu bản đồ:  ---  |  tường      (trống) không có tường      ?  chưa biết
//                  ^ > v <  xe        G  đích       .  ô đã đi qua

#include <Arduino.h>
#include "maze_algorithm.h"

class MazeMapper {
public:
  // Cấu hình (gán trong App.cpp từ MainParams.h)
  uint8_t cols = 16, rows = 16;          // kích thước mê cung thật
  uint8_t goalX = 7, goalY = 7;          // góc dưới trái của vùng đích
  uint8_t startX = 0, startY = 0;        // ô xuất phát (mặc định góc dưới trái; mê cung ở góc dưới phải thì đặt startX = cols - 1)
  uint8_t goalSize = 2;                  // 1: đích 1 ô, 2: đích 2x2
  uint16_t maxCells = 60;                // số ô đi tối đa (an toàn)
  bool startSidesWall = true;            // ô xuất phát có tường trái/phải (quy ước micromouse)
  bool alignFront = true;                // trước mặt có tường -> căn khoảng cách tới tường (đúng tâm ô)
  bool calibBack = true;                // sau lưng chắc chắn có tường -> lùi kịch tường calib lại tâm ô + gyro
  float calibBackErrDeg = 0.0f;          // speedrun: > 0 -> chỉ lùi calib khi hướng MPU lệch hướng chuẩn quá chừng này độ (mode 7-10)
  bool slalomTurns = false;             // mode 2: rẽ 90° bằng cua Slalom khi đang chạy (chỉ khi biết trước map)

  // Trạng thái
  int8_t x = 0, y = 0, h = 0;            // vị trí + hướng hiện tại của xe
  ParentMaze maze;                       // tường đã biết (dùng cho flood-fill)
  uint8_t known[MAZE_SIZE][MAZE_SIZE];   // bit d = đã đọc được vách hướng d của ô
  unsigned long roamT0 = 0;
  bool roam = false;                     // mode 5: chạy tự do (không đích): mỗi ngã rẽ chọn ngẫu nhiên hướng thông, ưu tiên ô ít đi
  uint8_t visits[MAZE_SIZE][MAZE_SIZE];  // số lần đã qua mỗi ô (chỉ dùng cho roam)
  uint8_t openLock[MAZE_SIZE][MAZE_SIZE]; // mode 5: bit d = đã thấy vách hướng d THÔNG -> khóa, mẫu "có tường" sau đó không được sửa

  void reset();                          // xoá bản đồ, đặt xe về (0,0) hướng Bắc
  // CHẶN: dò tới đích (hoặc hết maxCells), vẽ bản đồ.
  // continuous = false: dừng ở từng ô (SEQ_MODE 8). true: chạy liên tục qua ô thẳng, chỉ dừng để quay (SEQ_MODE 10)
  void explore(bool continuous = false);
  void runKnownMap();                    // SEQ_MODE 2: map nạp sẵn -> đường ngắn nhất -> chạy
  void fullRun();                        // SEQ_MODE 1: dò tới đích -> dò về xuất phát -> speedrun
  float speedTurnCost = 1.0f;            // speedrun: mỗi lần quay 90 độ tính bằng chừng này ô (0 = chỉ tính số ô)
  uint8_t speedrunSpeed = 0;             // PWM chạy thẳng khi speedrun (0 = giữ tốc độ thường)
  // Bản đồ nạp sẵn (lib/Mapping/PresetMaze.h): lượt dò đầu đã biết tường -> đi đường ngắn nhất ngay.
  // presetTrust = true: vách đọc KHI ĐANG CHẠY không ghi đè vách của bản đồ nạp sẵn (đứng yên đọc tường trước vẫn ghi đè)
  bool usePreset = false;
  bool presetTrust = true;
  bool presetLoaded = false;
  bool presetQual = false;               // SEQ_MODE 7/8: nạp PRESET_MAZE_QUAL (map vòng loại 16x16, không xoay)
  bool presetApec2 = false;              // SEQ_MODE 11: nạp PRESET_MAZE_APEC2 (map APEC 2 16x16, không xoay)
  bool presetApec = false;               // SEQ_MODE 9/10: nạp PRESET_MAZE_APEC (map APEC 16x16, không xoay)
  bool loadPreset();
  void senseWall(int cx, int cy, int dir, bool wall, bool sure);  // ghi vách đọc khi chạy (tôn trọng presetTrust)
  bool tieTowardCenter = true;           // hoà step: chọn ô gần tâm đích hơn (false: ưu tiên đi thẳng như cũ)
  float decideFrac = 0.80f;              // chạy liên tục: quyết định chạy tiếp/dừng khi đi được phần này của ô

  void setWall(int cx, int cy, int dir, bool wall, bool sure);
  bool hasWall(int cx, int cy, int dir) const;
  bool isKnown(int cx, int cy, int dir) const;
  int chooseDir(int cx, int cy, int ch);
  int runStraight(int maxRun);
  void begin();
  const char *exploreTo(int tx, int ty, int ts, bool continuous, uint16_t &cells);
  const char *exploreToSlalom(int tx, int ty, int ts, uint16_t &cells);  // SEQ_MODE 5: dò tới vùng đích, rẽ bằng cua Slalom khi đang chạy
  void roamSlalom(uint16_t nCells);     // SEQ_MODE 5: chạy tự do nCells ô bằng cua Slalom, in độ lệch theo thời gian
  int chooseRoam(int cx, int cy, int ch);
  void fullRunSlalom();                  // SEQ_MODE 5: dò tới đích -> dò về xuất phát (đều bằng cua) -> speedrun bằng cua
  void finish(const char *reason, uint16_t cells);
  bool speedRun();
  int8_t tX = 0, tY = 0, tS = 1;         // vùng đích của lượt dò hiện tại
  bool inTarget(int cx, int cy) const { return cx >= tX && cx < tX + tS && cy >= tY && cy < tY + tS; }
  bool inGoal(int cx, int cy) const {
    return cx >= goalX && cx < goalX + goalSize && cy >= goalY && cy < goalY + goalSize;
  }

  String draw() const;                   // bản đồ ASCII
  String report;                         // bản đồ lần chạy gần nhất (lệnh MAP để in)
  bool printMap = false;                 // true: tự in bản đồ ở mỗi ô và khi kết thúc
  bool reportPending = false;            // chờ cắm USB để in
};

extern MazeMapper mapper;
