# Task 1–10 完整技術報告

本文件集中說明雙 Berxel iHawk100 RGB-D 相機與 RoArm M3 從硬體確認、深度測試、雙相機外參、ArUco 三維定位、座標融合、機械臂座標校正，到監督式 Marker Approach 的完整過程。

> **安全結論**：系統的靜止融合重複性可以低於 1 mm，但目前相對 RoArm 的絕對座標誤差約為 10–12 mm，而且尚未涵蓋完整工作空間。因此只能用於有人監看的研究測試，不能用於自主接觸桌面、低間隙運動或精密抓取。

## 1. 系統架構與座標流程

```text
iHawk1 RGB ─┐                         ┌─ Camera 1 depth noise
            ├─ ArUco ID 4 center ────┤
iHawk1 Depth┘                         └─ Camera 1 3-D point
                                                │
                                                ▼
                                         base_link transform
                                                │
                                                ▼
                                      Camera 1 empirical correction
                                                │
                                                ├──────────────┐
                                                │              │
iHawk2 RGB + Depth ─► Camera 2 3-D point ─► base_link          │
                                                │              │
                                                └─ weighted fusion
                                                         │
                                                         ▼
                                                fused base point
                                                         │
                                                         ▼
                                          fused-to-RoArm rigid correction
                                                         │
                                                         ▼
                                                RoArm target point
```

主要 ROS frame：

- `base_link`：本專案使用的共同基準 frame。
- `ihawk1/berxel_camera_color_optical_frame`：Camera 1 彩色光學 frame。
- `ihawk1/berxel_camera_depth_optical_frame`：Camera 1 深度光學 frame。
- `ihawk2/berxel_camera_color_optical_frame`：Camera 2 彩色光學 frame。
- `ihawk2/berxel_camera_depth_optical_frame`：Camera 2 深度光學 frame。

光學 frame 遵守 ROS 相機慣例：Z 朝鏡頭前方、X 朝影像右方、Y 朝影像下方。RoArm SDK 回傳的 XYZ 則以機械臂底座座標定義。兩者不能直接比較，必須經過 TF 與後續剛體校正。

## 2. Task 1 — 雙相機辨識、啟動與基礎測試

### 目的

確認 VM 能同時辨識兩台 iHawk100，並且每次啟動都能讓固定序號對應固定 namespace，避免 Camera 1、Camera 2 交換。

### 執行過程

1. 在沒有相機、只有 Camera 1、兩台相機的情況下分別保存 `lsusb` 與 USB tree。
2. 以序號建立固定對應：
   - Camera 1：`HK100QB6513M2B479`
   - Camera 2：`HK100QB5311M2B242`
3. 建立 `dual_ihawk_bringup.launch.py`，啟動 `/ihawk1` 與 `/ihawk2`。
4. 關閉 point cloud 以降低 USB 與 CPU 負載。
5. 保存 topic、node、CameraInfo、TF graph、RGB/Depth rate 與 timestamp。

### 得到的資料

RGB timestamp 每台各收集 872 筆：

- 平均偏差：11.230 ms
- 中位數偏差：12.486 ms
- P95：18.509 ms
- 最大偏差：21.791 ms

Camera 1 內參：

```text
fx = 401.7702, fy = 401.9192
cx = 322.1314, cy = 202.5423
image = 640 × 400
```

Camera 2 內參：

```text
fx = 391.3235, fy = 391.3235
cx = 320.8533, cy = 202.9705
image = 640 × 400
```

### 遇到的問題與解法

- `uvc_open failed ret=-6`：USB device 已被舊相機 process 使用。用 `ps` 和 `sudo fuser -v /dev/bus/usb/...` 找出 process，關閉後再啟動。
- RViz 顯示 QoS incompatible：Publisher 與 Subscriber reliability 不相容。最後統一使用與 Berxel publisher 相容的 RELIABLE QoS。
- ROS topic 存在但收不到資料：不同 terminal 使用不同 `ROS_LOCALHOST_ONLY` 或 RMW。所有 terminal 統一 CycloneDDS 與 localhost 設定，再重啟 ROS daemon。
- `Failed to find a free participant index`：短時間開太多 CLI node。關閉不用的 terminal，設定 `MaxAutoParticipantIndex=50`。

## 3. Task 2 — Depth ROI 穩定度與雙相機干擾

### 目的

比較單獨開 Camera 1、單獨開 Camera 2，以及兩台一起開時的深度品質，確認兩台紅外深度相機是否互相干擾。

### 執行過程

`task2_depth_roi_stats.py` 在指定中心擷取 ROI，收集多幀深度。最初使用 80 × 80 ROI，但範圍包含不同物體與背景，導致標準差很大；最後改為 20 × 20 ROI。

對有效深度值 \(d_i\)：

\[
r_{invalid}=\frac{N_{zero}}{N_{all}}
\]

\[
\bar d=\frac{1}{N}\sum_i d_i,
\quad
\sigma=\sqrt{\frac{1}{N}\sum_i(d_i-\bar d)^2}
\]

\[
MAD=median(|d_i-median(d)|)
\]

Median 與 MAD 對零值、邊界點和離群深度比單點或平均值更穩定。

### 結果

| 模式 | Median | Overall STD | Invalid ratio |
| --- | ---: | ---: | ---: |
| Camera 1 only | 491 mm | 8.625 mm | 0 |
| Camera 2 only | 487 mm | 8.043 mm | 0 |
| Dual, Camera 1 | 492 mm | 8.756 mm | 0 |
| Dual, Camera 2 | 485 mm | 7.964 mm | 0 |

兩台同時啟動後沒有出現明顯的 ROI 深度惡化，因此在此場景沒有觀察到嚴重互相干擾。

### 問題與解法

- Collector 停在 `Collected 1/100`：Subscriber QoS 不相容；改為 RELIABLE。
- 80 × 80 ROI 數據很差：不是相機一定不穩，而是 ROI 跨越不同深度表面；縮小為 20 × 20。
- `--size` expected one argument：命令最後漏掉數值，補上 `--size 20`。

## 4. Task 3 — Checkerboard 與 ArUco 偵測

### 標定板定義

- 9 × 7 個棋盤格
- 實體總尺寸：225 × 175 mm
- 單格邊長：25 mm
- 內角點：8 × 6 = 48 點
- 周圍四張 `DICT_4X4_50` ArUco：ID 0、1、2、3
- 每張 Marker 黑框實測 50 mm

棋盤與四張 Marker 固定在同一塊平整硬板上，移動時相對位置不得改變。

### 執行與結果

`check_dual_checkerboard.py` 同時讀取兩台相機，兩台都成功找到 48 個內角點。Marker viewer 最終也讓兩台相機都辨識 ID `[0,1,2,3]`。

### 問題與解法

- Camera 1 只能看到三張 Marker：第四張距離影像邊界太近、透視角大或被機械臂遮擋。調整相機視角並重新擷取 live frame。
- 使用舊 PNG 反覆檢測：相機移動後舊圖片不會更新。必須先重新執行 checker，再對新保存圖片做 ArUco 偵測。
- `checkerboard=0/48` 但畫面有彩色角點：顯示邏輯使用了不同偵測結果；後續 pose detector 統一以 refined corners 的數量更新 status。

## 5. Task 4 — Checkerboard Pose、方向與重投影誤差

### 計算方法

棋盤物體點定義為：

\[
P_{ij}=[0.025i,0.025j,0]^T \text{ metre}
\]

OpenCV `solvePnP` 求解旋轉 \(R\) 與平移 \(t\)，使棋盤三維點投影到影像後接近偵測角點。重投影 RMSE：

\[
e_{px}=\sqrt{\frac{1}{N}\sum_i\|p_i-\pi(K(RP_i+t))\|^2}
\]

四個 ArUco ID 用來決定棋盤方向，解決棋盤格本身可能產生的 180° 對稱翻轉。

### 結果

- Camera 1 representative reprojection error：0.262 px
- Camera 2 representative reprojection error：0.188 px
- Acceptance threshold：小於 1 px

100-frame 靜止 Pose 測試：

| Camera | Position 3-D RMSE | Position max | Rotation max |
| --- | ---: | ---: | ---: |
| iHawk1 | 0.195 mm | 0.486 mm | 0.075° |
| iHawk2 | 0.047 mm | 0.102 mm | 0.026° |

這證明 detector 在固定板面時具有很好的重複性，但尚未證明相對機械臂的絕對準確度。

## 6. Task 5 — 雙相機外參

### 為什麼需要 30 個不同姿勢

只用一個固定姿勢會讓雜訊或局部幾何誤差直接進入結果，也無法充分約束旋轉。每次採樣都讓硬板平移至少 20 mm 或旋轉至少 5°，並確保兩台相機同時看到完整目標。

對每組樣本：

\[
{}^{C_1}T_{C_2,k}={}^{C_1}T_{Board,k}
({}^{C_2}T_{Board,k})^{-1}
\]

每組都得到 Camera 2 相對 Camera 1 的 transform。接著排除 translation/rotation residual 過大的 outlier，再平均平移與旋轉。

### 最終外參

28/30 組被保留：

```text
translation (m): [-0.3293606, -0.1212040, 0.2392759]
quaternion xyzw: [0.0021612, 0.4124386, 0.2140794, 0.8854715]
translation RMSE: 2.081 mm
translation max: 4.039 mm
rotation RMSE: 0.202 deg
rotation max: 0.368 deg
mean timestamp skew: 4.589 ms
max timestamp skew: 16.704 ms
```

三個獨立位置的平均 3-D validation error 為 0.778、1.789、1.963 mm。

### 失敗經驗

第一個錯誤校正驗證時出現 123 mm 誤差。原因是兩台 detector 對棋盤方向的定義不一致（Camera 1 rotated_180、Camera 2 normal）或使用了錯誤版本的 transform。失敗 YAML 被保留並標記為 `failed_123mm`，不可使用。

## 7. Task 6 — Camera 到 Robot Base 的 TF

### TF 的意義

相機只知道目標在 camera optical frame 的座標；RoArm 需要的是目標相對 robot base 的座標。因此需要：

\[
{}^{Base}P={}^{Base}R_{Camera}\,{}^{Camera}P+{}^{Base}t_{Camera}
\]

以齊次矩陣表示：

\[
{}^{Base}T_{Camera}=
\begin{bmatrix}R&t\\0&1\end{bmatrix}
\]

### 過程

一開始沿用 partner 的 board-to-robot 資料，條件是棋盤、robot base 和 camera 都回到完全相同的位置。因為這個條件難以精確重現，所以先標記為 provisional。之後使用三個已知板面點修正，第四點獨立檢查。

`base_camera_extrinsics_corrected.yaml` 結果：

- Fit RMSE：4.327 mm
- Validation 3-D error：5.573 mm
- 狀態：`three_point_corrected_one_point_validated`

### TF tree 問題

相機 driver 只發布 camera 內部 TF，沒有 `base_link`，所以曾看到：

```text
Tf has two or more unconnected trees
```

解法是啟動保存的 static transform publisher，把 `base_link` 連到兩台相機的 base link。Estimator 也保留 `corrected_calibration_fallback`，在 ROS TF 尚未可用時使用同一組明確標記的校正矩陣。

> 若相機、機械臂底座或支架移動，這些 TF 必須重做。

## 8. Task 7 — ArUco ID 4 RGB-D 三維座標

### 執行流程

1. 在 RGB 找到 ID 4 四個角。
2. 以四角平均得到中心像素 \((u,v)\)。
3. 從深度影像中心附近取 9 × 9 ROI，最多 81 點。
4. 排除深度 0，使用 median depth \(Z\)。
5. 以 CameraInfo 反投影為相機三維座標。
6. 經 TF 轉成 `base_link`。

反投影公式：

\[
X=\frac{(u-c_x)Z}{f_x},\quad
Y=\frac{(v-c_y)Z}{f_y},\quad
P_c=[X,Y,Z]^T
\]

### 時間同步修正

最初只保存「最後一張 depth」，造成 RGB/Depth skew 達 165–500 ms。改為保存 depth deque，對每張 RGB 選擇時間戳最近的 depth：

\[
d^*=\arg\min_d |t_{RGB}-t_d|
\]

### Camera 1 empirical correction

兩台相機雖然都轉到 `base_link`，四個位置仍有約 21.5–25.5 mm 的系統性差異。因此用 Position 1–3 的 paired points 計算 Camera 1 到 Camera 2 結果的剛體修正：

```text
R = [[ 0.99847761,  0.00555582,  0.05487792],
     [-0.00447147,  0.99979272, -0.01986240],
     [-0.05497689,  0.01958677,  0.99829550]]
t = [-0.01463351, 0.00049722, 0.00335271] m
```

訓練 RMSE 2.768 mm，最大 3.618 mm；獨立 Position 4 從 21.498 mm 降至 4.826 mm。

這只代表兩台 camera agreement 變好，不代表 robot absolute accuracy 同樣是 4.826 mm。

## 9. Task 8 — 雙相機融合與 Robot Correction

### 加權融合

兩台相機的深度標準差為 \(\sigma_1,\sigma_2\)，使用 inverse-variance weighting：

\[
w_i=\frac{1/\sigma_i^2}{1/\sigma_1^2+1/\sigma_2^2}
\]

\[
P_f=w_1P_1+w_2P_2
\]

深度越穩定的相機權重越高。Fusion 只有在 timestamp skew 與兩相機空間差異通過門檻時才發布。

### 融合重複性

Position 1 的 100 組融合結果：

```text
Mean: [221.637, -82.455, -89.358] mm
STD XYZ: [0.305, 0.091, 0.221] mm
3-D repeatability RMSE: 0.387 mm
P95 deviation: 0.510 mm
Maximum deviation: 0.548 mm
```

### 六點 fused-to-robot 校正

P1–P6 使用 camera fused mean 作為 source，RoArm 接觸點作為 target。使用 Kabsch/SVD 求剛體 transform。

先計算 centroid：

\[
\bar p=\frac1N\sum p_i,\quad \bar q=\frac1N\sum q_i
\]

Covariance：

\[
H=\sum_i(p_i-\bar p)(q_i-\bar q)^T
\]

SVD：

\[
H=U\Sigma V^T
\]

避免 reflection 的 rotation：

\[
R=V\,diag(1,1,det(VU^T))\,U^T
\]

Translation：

\[
t=\bar q-R\bar p
\]

最終公式：

\[
P_{robot}=R_{rf}P_{fused}+t_{rf}
\]

```text
R = [[ 0.98134014, -0.10336585, -0.16213275],
     [ 0.10078440,  0.99461662, -0.02408898],
     [ 0.16374991,  0.00729903,  0.98647488]]

t = [-10.25263208, 0.09260867, -22.17450662] mm
```

- 六點 training RMSE：6.322 mm
- Training maximum residual：9.480 mm
- P7 independent validation：約 9.6 mm

曾測試 affine transformation，但點位接近平面且設計矩陣條件不佳；Affine 會加入 scale/shear，不符合兩個剛體座標 frame 的物理關係，因此最後保留 SE(3) rigid transformation。

## 10. Task 9 — Robot-referenced Accuracy Validation

### Accuracy 與 Repeatability 的差別

重複性：量測是否集中在自己的平均值附近。

\[
RMSE_{repeat}=\sqrt{\frac1N\sum_i\|p_i-\bar p\|^2}
\]

Accuracy：量測是否接近 robot ground truth \(g\)。

\[
e_i=p_i-g
\]

\[
RMSE_{accuracy}=\sqrt{\frac1N\sum_i\|e_i\|^2}
\]

因此融合點可以非常穩定，但仍可能整體偏移 10 mm 以上。

### Position 1 accepted result

三次 RoArm touch 的 median ground truth：

```text
[240.614, -3.322, -77.417] mm
```

Robot-corrected prediction mean：

```text
[239.149, 8.600, -76.749] mm
```

```text
Mean error XYZ: [-1.465, 11.922, 0.668] mm
3-D RMSE: 12.034 mm
P95: 12.426 mm
Maximum: 12.529 mm
```

### Position 2 excluded result

Position 2 RMSE 18.758 mm、maximum 18.950 mm，因此沒有被當作 accepted validation；但原始資料仍保留，避免只留下成功結果。

### 主要誤差來源

- RoArm TCP 接觸點不是機械固定尖端，人工接觸會變動。
- Tilt/roll 每次不一致，末端實際接觸點會改變。
- 標定點大多接近平面，對部分旋轉/深度方向約束較弱。
- Marker center 的深度不一定等於 robot 實際接觸位置。
- Camera/base TF、Camera 1 empirical correction 與 robot correction 的殘差累積。

增加同一位置的數據只能降低 random noise，無法消除上述 systematic bias。要進一步降低誤差，必須增加分布於三維空間的校正點、固定 TCP、固定 tilt/roll，並保留獨立 validation points。

## 11. Task 10 — Supervised Marker Approach

### 最終需求流程

```text
RoArm 回到 home
→ 使用者確認
→ Viewer 確認兩台相機看到 ID 4
→ 讀取 /fused_target_pose_robot_corrected
→ 再次確認目標
→ 打開 gripper
→ 經高位 waypoint 接近
→ 停在 Marker 上方 10 mm
```

保存的版本：

- `task10_guarded_approach.py`：原始 guarded approach。
- `task10_guarded_approach_safe_backup.py`：安全備份。
- `task10_marker_approach_20mm_7step.py`：七段 20 mm 版本。
- `task10_marker_approach_20mm.py`：較快 20 mm 版本。
- `task10_marker_approach_10mm( Final ).py`：最後測試的 10 mm 版本。

### 為什麼最初分七步

RoArm SDK 的 Cartesian pose command 不保證末端沿直線運動。大幅度單步命令可能因 IK 分支、關節限制或追蹤誤差偏離目標。多段 waypoint 可以：

- 先在高處完成水平移動。
- 再垂直下降，避免掃過桌面或障礙物。
- 每段檢查實際 pose。
- 在偏差變大時提早停止。

等待時間是讓 servo 完成動作並避免下一指令過早覆蓋前一指令。後續改成三段可以更快，但安全檢查與追蹤能力也會減少。

### Guard abort 經驗

一次 Approach 3/6 的實際 tracking error 為 53.2 mm，超過原本 45 mm guard，因此自動停止。這個 guard 是防止 IK 或控制誤差使機械臂繼續向下。後續測試採 warning/監督方式，但這不代表移除 guard 後系統更準確。

Guarded pre-approach：

```text
Planned:  [280.733, -0.851, -25.443] mm
Measured: [279.284, -11.574, -25.481] mm
3-D difference: about 10.82 mm
```

目前約 10–12 mm absolute error 與 10 mm clearance 同一量級，因此 10 mm approach 不能被視為具安全保證的自主動作。

## 12. 常用啟動環境與檢查

每個 ROS terminal 使用相同設定：

```bash
source /opt/ros/humble/setup.bash
source ~/berxel_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

主要輸出：

```text
/ihawk1/target_quality
/ihawk2/target_quality
/ihawk1/target_point_base
/ihawk2/target_point_base
/dual_camera/fusion_status
/fused_target_pose
/fused_target_pose_robot_corrected
```

確認兩台相機的 quality 都是 `accepted=true`，再確認 fusion status 出現 `accepted=true`。單次 `timestamp_skew` reject 不代表整個系統壞掉；fusion 會等待下一組時間較接近的 pair。

## 13. 訓練與驗證資料連結

### Task 1–3 基礎證據

- [Task 1 相機、USB、timestamp、CameraInfo 與 TF 資料](../../experiment/dual_berxel_roarm/results/task1_results)
- [Task 2 Depth ROI 資料](../../experiment/dual_berxel_roarm/results/task2_results)
- [Task 3 Checkerboard/ArUco 偵測圖片](../../experiment/dual_berxel_roarm/results/task3_results)

### Camera 1 empirical correction

- [Task 7 四個位置的兩相機座標](../../experiment/dual_berxel_roarm/results/task7_results)
- [Camera 1 empirical correction YAML](../../src/dual_berxel_calibration/config/camera1_empirical_correction.yaml)

Position 1–3 是 correction training，Position 4 是獨立驗證。

### Fused-to-RoArm 六點訓練資料

- [六點 paired calibration points](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/calibration_points.csv)
- [P1 fused samples](../../experiment/dual_berxel_roarm/results/task8_results/fused_position1_100.csv)
- [P2 fused samples](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/p2_fused_30.csv)
- [P3 fused samples](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/p3_fused_30.csv)
- [P4 fused samples](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/p4_fused_30.csv)
- [P5 fused samples](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/p5_fused_30.csv)
- [P6 fused samples](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/p6_fused_30.csv)
- [P7 independent validation](../../experiment/dual_berxel_roarm/results/task8_results/robot_calibration/p7_fused_30.csv)
- [最終 fused-to-robot rigid correction](../../src/dual_berxel_calibration/config/fused_to_robot_correction.yaml)

P1–P6 用於 fit；P7 沒有參與 fit，只用於驗證泛化誤差。

### Task 9 Robot Accuracy

- [Position 1 accepted camera observations](../../experiment/dual_berxel_roarm/results/task9_results/raw/position1_redo.csv)
- [Position 1 three robot touches](../../experiment/dual_berxel_roarm/results/task9_results/raw/position1_redo_robot_touches.csv)
- [Position 1 summary](../../experiment/dual_berxel_roarm/results/task9_results/summary/position1_redo_summary.json)
- [Position 2 excluded data](../../experiment/dual_berxel_roarm/results/task9_results/raw/excluded_position2)
- [Incomplete/rejected attempts](../../experiment/dual_berxel_roarm/results/task9_results/raw/incomplete)

### Task 10

- [Approach test record](../../experiment/dual_berxel_roarm/results/task10_results/safe_approach_test1.txt)
- [Final 10 mm approach code](<../../tools/dual_berxel_roarm/task10_marker_approach_10mm( Final ).py>)
- [ArUco marker viewer](../../tools/dual_berxel_roarm/aruco_marker_viewer.py)

## 14. 最後結論

本專案已完成從雙相機 RGB-D 輸入、棋盤外參、ArUco 3-D、TF、相機一致性修正、uncertainty-weighted fusion、RoArm rigid correction 到監督式 approach 的完整 pipeline。

目前最強的結果是：相機融合的靜止重複性低於 1 mm；目前最重要的限制是：robot-referenced absolute accuracy 仍約 10–12 mm。下一步應重新設計具有固定 TCP、固定姿態、更多非共面點和 held-out validation 的 robot calibration，而不是只在單一位置收集更多重複資料。
