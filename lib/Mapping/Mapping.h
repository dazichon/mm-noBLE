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
  uint8_t goalSize = 2;                  // 1: đích 1 ô, 2: đích 2x2
  uint16_t maxCells = 60;                // số ô đi tối đa (an toàn)
  bool startSidesWall = true;
  bool alignFront = true;                // trước mặt có tường -> căn khoảng cách tới tường (đúng tâm ô)
  bool calibBack = true;                // sau lưng chắc chắn có tường -> lùi kịch tường calib lại tâm ô + gyro            // ô xuất phát có tường trái/phải (quy ước micromouse)

  // Trạng thái
  int8_t x = 0, y = 0, h = 0;            // vị trí + hướng hiện tại của xe
  ParentMaze maze;                       // tường đã biết (dùng cho flood-fill)
  uint8_t known[MAZE_SIZE][MAZE_SIZE];   // bit d = đã đọc được vách hướng d của ô

  void reset();                          // xoá bản đồ, đặt xe về (0,0) hướng Bắc
  void explore();                        // CHẶN: dò tới đích (hoặc hết maxCells), vẽ bản đồ

  void setWall(int cx, int cy, int dir, bool wall, bool sure);
  bool hasWall(int cx, int cy, int dir) const;
  bool isKnown(int cx, int cy, int dir) const;
  bool inGoal(int cx, int cy) const {
    return cx >= goalX && cx < goalX + goalSize && cy >= goalY && cy < goalY + goalSize;
  }

  String draw() const;                   // bản đồ ASCII
  String report;                         // bản đồ lần chạy gần nhất
  bool reportPending = false;            // chờ cắm USB để in
};

extern MazeMapper mapper;
