# So sánh cách xác định tường và căn tường trước: Mode 1 và Mode 5

Ghi 2026-11-07. Mode 5 hiện gọi `mapper.fullRunSlalom()`: dò tới đích, dò về, speedrun, rẽ bằng cua Slalom. Mode 1 gọi `mapper.fullRun()`: cùng luồng nhưng quay tại chỗ.

Giá trị lấy từ `MainParams.h`, `SlalomParams.h`, `RobotNav.h`, `NavStraight.cpp`, `NavSlalom.cpp`, `Mapping.cpp`, `App.cpp`. Số "tham số gốc" là giá trị `#define`. Nếu `App.cpp` có ghi đè thì ghi chú riêng.

## 1. Tường TRƯỚC (xe đứng yên ở tâm ô)

| Hạng mục | Mode 1 | Mode 5 |
|---|---|---|
| Khi nào đọc | Mỗi vòng lặp, xe đứng yên, `docTuong()` | Chỉ khi `atRest`, tức vừa dừng ở tâm ô. Lúc đang chạy thì dùng mục 2 |
| Hàm | `senseWallsSafe` → `classifyWall` | Như mode 1 |
| Lấy mẫu | 250 ms, tối đa 8 mẫu, cần ≥ 3 mẫu | Như mode 1 |
| Ngưỡng có tường | `NAV_FRONT_WALL_MM` = **150** mm (ghi chú: tường ngay cuối ô đo ~113-153, cách 1 ô ~200-260) | Như mode 1 |
| Quy tắc | Quá nửa số mẫu < ngưỡng = TƯỜNG. Quá nửa số mẫu ≥ ngưỡng hoặc -3 = THÔNG. Còn lại = không đáng tin, dừng | Như mode 1 |

## 2. Tường TRƯỚC (xe đang chạy, chưa dừng)

| Hạng mục | Mode 1 (`runStraight`, `NavStraight.cpp`) | Mode 5 (`slalomStraight` với `slExplore`, `NavSlalom.cpp`) |
|---|---|---|
| Cách phát hiện | Lấy mẫu VL53 trước trong cửa sổ quãng đường ô `frontWinFrom`..`frontWinTo` = **0.55..0.78** | Không có cửa sổ. Đoạn thẳng kết thúc khi VL53 trước ≤ `trigMm` (2 mẫu liên tiếp) |
| Ngưỡng | `NAV_FRONT_WALL_MM` + (1 − frac) × ô × `frontMoveGain` (0.5). Tường ở xa hơn thì ngưỡng lớn hơn | `trigMm` = `FRONT_ALIGN_MM` + quãng cua sớm F (~68) + `FRONT_TRIG_BIAS` (−8) ≈ **160 mm** nếu đoạn kết thúc để cua. Nếu đoạn kết thúc để dừng, F được thay bằng `STOP_LEAD` 12 |
| Kết luận | `movingFrontWall`: ≥ 50% mẫu gần = TƯỜNG. ≥ 75% mẫu xa = THÔNG. Còn lại = không biết | `slLegWall` = true khi đoạn kết thúc vì `TUONG`, hoặc kết thúc `SUCCESS` mà VL53 trước ≤ **300** mm (`FRONT_STOP_MAX_MM`) |
| Mẫu hợp lệ | VL53 trước đọc ≤ `FRONT_STOP_MAX_MM` = 300 | Như mode 5 bên trái, cùng giới hạn 300 |
| Phanh khẩn | Chỉ khi `frontGuard` bật | `frontGuard` luôn bật trong `fullRunSlalom`, ngưỡng `FRONT_EMERGENCY_MM` = **75** mm |

## 3. Vách HÔNG (VL53 trái/phải đặt chéo 45°, nhìn trước ~1 ô)

| Hạng mục | Mode 1 | Mode 5 (dò) |
|---|---|---|
| Lấy mẫu ở đâu | Cửa sổ quãng đường ô `MAP_SIDE_FROM`..`MAP_SIDE_TO` = **0.00..0.30** | Cửa sổ vị trí `SLALOM_EXPLORE_SIDE_FROM`..`TO` = **40..112 mm** tính từ tâm ô trước. Mẫu "thông" thì lấy ở bất kỳ vị trí nào trong đoạn |
| Ngưỡng "có tường" | `NAV_SIDE_WALL_MM` = **220**. Đọc < 220 là gần | Cùng 220 (`navSideWallMm`), nhưng chỉ đếm khi nằm trong cửa sổ |
| Ngưỡng "thông" | Đọc ≥ 220 hoặc -3 | **Trái ≥ 205**, **phải ≥ 185**, hoặc -3 (ngưỡng riêng từng bên) |
| Số mẫu để kết luận | `movingSideWall`: tổng gần + xa ≥ 3, rồi ≥ 75% gần = TƯỜNG, ≥ 75% xa = THÔNG, lẫn lộn = không biết | THÔNG: trái cần **2** mẫu liên tiếp, phải cần **1** mẫu. TƯỜNG: `movingSideWall` như mode 1 |
| Bổ sung cho mắt phải | Không | Trong đoạn dò, max − min ≥ **55** mm (cần ≥ 5 mẫu) cũng tính là THÔNG |
| Khóa kết quả | Không. `senseWall` ghi theo kết quả mỗi ô | **Có** (`openLock`). Đã thấy thông thì khóa. Mẫu "có tường" đến sau không được ghi đè. Cạnh xe đã đi xuyên qua cũng bị khóa thông |
| Lọc tia chéo chạm tường trước | `SIDE_ECHO`: bỏ mẫu hông khi VL53 trước ≤ 260 và \|đọc − 1.3 × VL53 trước\| ≤ 25 | Như mode 1 (`sideIsFrontEcho`) |
| Lý do khác biệt | Xe dừng ở từng ô, chạy chậm, mẫu "có tường" đủ tin | Xe chạy nhanh. Mẫu "có tường" hay sai vì tia chéo chiếu vào vách ô vừa rời. Mẫu "thông" thì đáng tin hơn |

## 4. Căn tường trước (`alignFrontWall` → `canTuongTruoc`)

Hai mode dùng chung một hàm. Chỉ khác lúc nó được gọi.

| Hạng mục | Giá trị | Ghi chú |
|---|---|---|
| Khoảng cách đích `FRONT_ALIGN_MM` | **100** mm | VL53 trước đọc khi xe đúng tâm ô có tường trước. Memory ghi F≈111 là tư thế thực đo |
| Bỏ qua nếu lệch ≤ `FRONT_ALIGN_SKIP` | **10** mm | Chỉ xét ở lần đọc đầu |
| Dung sai dừng `FRONT_ALIGN_TOL` | **6** mm | Phải đứng yên trong dung sai > 100 ms |
| Lọc | Trung vị 3 mẫu, mẫu hợp lệ 0 < đọc < 400 | |
| PWM | Kp 0.2 PWM/mm, tối thiểu 40, tối đa 55 | Đứng im thì tăng dần theo thời gian |
| Thời gian tối đa | 800 ms | |
| Bật/tắt | `MAP_ALIGN_FRONT` = 1 | |

| Thời điểm gọi căn | Mode 1 | Mode 5 |
|---|---|---|
| Sau khi đọc tường trước lúc đứng yên | Có (`Mapping.cpp:261`) nếu thấy tường trước | Có (`Mapping.cpp:420`) khi `atRest` |
| Sau đoạn thẳng cuối trước khi dừng ở ngõ cụt hoặc phía rẽ chưa chắc | Không có tình huống này | Có (`Mapping.cpp:462`): `slalomStraight(F)` rồi căn |
| Tới ô đích | Căn nếu tường trước đã biết (`Mapping.cpp:657`) | Căn (`Mapping.cpp:409`) nếu `frontWall` |
| Trước khi quay trong speedrun | Có (`Mapping.cpp:608`) | Như mode 1 |
| Đang cua Slalom liên tục | Không áp dụng | **Không căn**. Xe chỉ chốt vị trí bằng VL53 trước ở đoạn thẳng trước cua (`trigMm`) |

## 5. Các khác biệt khác ảnh hưởng tới việc xác định tường

| Hạng mục | Mode 1 | Mode 5 |
|---|---|---|
| Bản đồ nạp sẵn (`usePreset`) | Dùng nếu `MAP_PRESET` = 1 (hiện bằng 1) | **Không** (`App.cpp:81` loại mode 5). Dò từ bản đồ trống |
| Lùi kịch tường calib sau lưng (`calibBack`) | Bật nếu `MAP_CALIB_BACK` = 1 | **Tắt** (`App.cpp:77`). Chỉ lùi kịch tường sau khi quay đầu xong mà đã biết sau lưng có tường |
| Phanh khẩn | Tùy `frontGuard` | Luôn bật, 75 mm |
| Khi cần rẽ mà chưa chắc vách | Luôn quay tại chỗ | Dừng ở tâm ô rồi quay tại chỗ như mode 1. Chỉ cua khi phía rẽ không có tường đã biết |
| Tốc độ speedrun | `MAP_SPEEDRUN_PWM` = 210 | `SLALOM_RUN_PWM` = 70 |

## 6. Bảng ngưỡng đọc: CÓ TƯỜNG / KHÔNG TƯỜNG (số mm VL53 đọc được)

`-3` = ngoài tầm đo, luôn tính là KHÔNG TƯỜNG ở mọi ô trong bảng.

| Cảm biến, tình huống | Mode 1: CÓ TƯỜNG | Mode 1: KHÔNG TƯỜNG | Mode 5: CÓ TƯỜNG | Mode 5: KHÔNG TƯỜNG |
|---|---|---|---|---|
| **Trái** (hông, khi chạy) | < **220** | ≥ 220 | < **205** | ≥ **205** |
| **Phải** (hông, khi chạy) | < **220** | ≥ 220 | < **185** | ≥ **185** |
| **Trước**, đứng yên ở tâm ô | < **150** | ≥ 150 | < **150** | ≥ 150 |
| **Trước**, đang chạy | < 150 + (1 − frac) × ô × 0.5, nên ngưỡng nới ra khi còn xa tường | Từ ngưỡng đó trở lên | ≤ ~**160** (`trigMm`), hoặc ≤ 300 khi đoạn kết thúc `SUCCESS` | > 160 và > 300 |
| Phanh khẩn (trước) | ≤ 75 nếu `frontGuard` bật | | ≤ **75** | |

Cách đọc bảng:
- **Mode 1 hông:** ngưỡng 220 là một ranh giới duy nhất. Dưới 220 là gần, từ 220 trở lên là xa. Kết luận cuối vẫn cần ≥ 75% số mẫu cùng loại, nếu không thì "không biết".
- **Mode 5 hông:** thực tế "có tường" chỉ chắc khi mọi mẫu đều dưới ngưỡng mới (trái 205, phải 185). Mẫu trong vùng 205 đến 220 (trái) hoặc 185 đến 220 (phải) vẫn được đếm là gần nếu nằm trong cửa sổ 40-112 mm. Nhưng chỉ cần đủ số mẫu "thông" (trái 2, phải 1) là kết quả khóa thành THÔNG, nên vùng chồng này nghiêng về THÔNG.
- **Mắt phải mode 5:** ngoài ngưỡng 185, nếu max − min trong đoạn ≥ 55 mm (ít nhất 5 mẫu) cũng kết luận THÔNG dù từng mẫu đều dưới 185.
- **Trước, đang chạy mode 1:** `frac` là phần quãng đường ô đã đi, chỉ lấy mẫu trong frac từ 0.55 đến 0.78. Giá trị `cellMm` tôi chưa đọc, nên bảng để dạng công thức.

### 6b. Vì sao ngưỡng hai mode lệch nhau

| Yếu tố | Mode 1 | Mode 5 |
|---|---|---|
| Chỗ rẽ / gặp tường trước | Dừng, đọc tường trước lúc đứng yên (250 ms, ≤ 8 mẫu), căn tường, quay 90° tại chỗ rồi đọc lại | Cua liên tục, không có lúc đứng yên. Chỉ dừng khi ngõ cụt hoặc chưa chắc vách phía rẽ |
| Thời gian đọc mỗi quyết định | Dài, đủ mẫu để lấy đa số (≥ 75%) | Ngắn: cửa sổ hông ~70 mm đường, ~7-9 mm mỗi mẫu VL53 ở 190-240 mm/s |
| Đoạn thẳng liền nhau | Vẫn chạy liên tục (`continuous = true`), vách hông vẫn đọc khi đang chạy | Như mode 1 |
| Độ tin cậy mẫu "có tường" | Khá tốt vì xe chậm và hay dừng | Kém: tia chéo hay chiếu vào vách ô vừa rời, cột góc, tường trước |
| Độ tin cậy mẫu "thông" | Bình thường | Đáng tin hơn mẫu "có tường" |
| Cách bù | Luật 75% số mẫu, ngưỡng 220 chung hai bên | Khóa thông (`openLock`), ngưỡng thông riêng từng bên (205 / 185), độ phân tán mắt phải, cửa sổ lấy mẫu dời ra 40-112 mm |
| Reset hướng và vị trí | Sau mỗi lần quay và căn tường | Chỉ sau khi dừng ở tâm ô |

Hệ quả: phần cứng không đảm bảo, cảm biến nhiễu nhiều. Mode 1 bù bằng thời gian đứng yên đọc. Mode 5 không có thời gian đó nên phải bù bằng luật ghi bản đồ lệch về phía "thông" và ngưỡng chỉnh theo log, vì vậy cần chạy thử và chỉnh lại.

## 7. Điểm cần lưu ý

- Hai bên VL53 hông có ngưỡng "thông" lệch nhau ở mode 5 (trái 205, phải 185). Cả hai chỉ cách ngưỡng "có tường" 220 của mode 1 từ 15 đến 35 mm. Biên rất hẹp, nên nếu báo thông giả thì tăng số mẫu hoặc ngưỡng, nếu bỏ sót thì hạ ngưỡng.
- `SLALOM_EXPLORE_OPEN_MM_OLD` (190) còn trong `SlalomParams.h` nhưng không còn được dùng.
- Mode 5 chưa chạy thử trên xe sau khi đổi sang `fullRunSlalom`. Các số trong bảng mục 3 được tune khi mode 5 còn là chạy tự do.
- Tôi chưa kiểm tra `NAV_FRONT_WALL_MM` có bị ghi đè ở nơi khác ngoài `MainParams.h` hay không. Giá trị 150 lấy từ `#define`.
