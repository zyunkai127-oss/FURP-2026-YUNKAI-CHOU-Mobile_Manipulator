# Task 1–10 execution record

## Task 1 — Bring up and identify two cameras

The cameras were enumerated with `lsusb`, compared one-at-a-time, and bound to
stable namespaces by serial number. Camera 1 is `HK100QB6513M2B479`; Camera 2
is `HK100QB5311M2B242`. A dual launch file starts `/ihawk1` and `/ihawk2` and
disables point clouds to reduce USB load. `view_frames`, CameraInfo, topic lists,
USB topology, and rates were saved in `task1_results`.

Measured RGB timestamp skew: mean 11.230 ms, median 12.486 ms, P95 18.509 ms,
maximum 21.791 ms (872 samples per camera). Camera 1 intrinsics are
`fx=401.7702, fy=401.9192, cx=322.1314, cy=202.5423`; Camera 2 intrinsics are
`fx=fy=391.3235, cx=320.8533, cy=202.9705`.

## Task 2 — Depth ROI quality and mutual interference

`task2_depth_roi_stats.py` collects a square depth ROI and reports invalid
ratio, median, standard deviation, MAD, minimum, and maximum. A 20 × 20 ROI
was selected because the earlier 80 × 80 ROI crossed objects at different
depths. In the restarted fixed scene, Camera 1 alone had median 491 mm and
8.625 mm overall standard deviation; Camera 2 alone had median 487 mm and
8.043 mm standard deviation. With both cameras on, the corresponding values
were 492/8.756 mm and 485/7.964 mm, with zero invalid ratio. No material
dual-camera depth interference was observed in this ROI.

## Task 3 — Calibration target detection

The existing rigid target contains a 9 × 7 square checkerboard (225 × 175 mm,
25 mm squares) and four independent 50 mm ArUco markers, IDs 0–3. The checker
detects 8 × 6 = 48 inner corners. Both cameras detected all 48 corners and all
four IDs after the view was adjusted. The checkerboard and markers must remain
on the same flat rigid board.

## Task 4 — Checkerboard pose and orientation

`checkerboard_pose_detector.py` refines the 48 sub-pixel corners, uses the
four marker IDs to resolve normal versus 180-degree board orientation, runs
PnP, and publishes pose/status/debug images. Representative reprojection error
was 0.262 px for iHawk1 and 0.188 px for iHawk2; both passed the 1 px limit.

A 100-frame stationary test gave iHawk1 3-D position RMSE 0.195 mm, maximum
0.486 mm, and maximum rotation error 0.075°. iHawk2 gave 0.047 mm RMSE,
0.102 mm maximum, and 0.026° maximum rotation error.

## Task 5 — Dual-camera extrinsic calibration

Thirty board poses were captured while the two cameras remained fixed. Each
new pose moved at least 20 mm or rotated at least 5° so the fit was not made
from duplicate views. The final solution retained 28/30 samples:

- iHawk1-to-iHawk2 translation: `[-0.329361, -0.121204, 0.239276] m`
- quaternion xyzw: `[0.002161, 0.412439, 0.214079, 0.885471]`
- translation RMSE/max: 2.081/4.039 mm
- rotation RMSE/max: 0.202/0.368°
- timestamp skew mean/max: 4.589/16.704 ms

Independent board-position validation produced 0.778, 1.789, and 1.963 mm
mean 3-D error. The older 123 mm failed calibration is preserved intentionally
as negative evidence and must not be used.

## Task 6 — Camera frames in the robot base

Partner board-to-robot measurements supplied an initial estimate. Because that
estimate depended on restoring the same physical board pose, it was marked
provisional. Three robot/board points and one independent check were then used
to create `base_camera_extrinsics_corrected.yaml`. Its fit RMSE was 4.327 mm
and independent validation error was 5.573 mm. The resulting transforms are
used only while both cameras and the robot base remain unchanged.

## Task 7 — ArUco depth point in `base_link`

`aruco_depth_point_estimator.py` detects ID 4 in RGB, samples the median of a
9 × 9 depth neighbourhood (up to 81 mono16 millimetre values), back-projects
the centre into 3-D, and transforms it to `base_link`. A nearest-depth-frame
buffer avoids pairing RGB with a stale depth frame.

Across four locations, the raw camera disagreement was approximately
21.5–25.5 mm. A three-location rigid Camera-1 correction reduced training RMSE
to 2.768 mm and maximum error to 3.618 mm. Independent Position 4 error fell
from 21.498 to 4.826 mm. This correction improves agreement between cameras;
it does not prove absolute robot accuracy.

## Task 8 — Weighted fusion and fused-to-robot correction

`dual_point_fusion.py` pairs buffered points, rejects excessive timestamp skew
or inter-camera distance, applies the Camera-1 correction, and weights each
camera inversely by depth variance. A static fused point had XYZ standard
deviation `[0.305, 0.091, 0.221] mm`, 3-D repeatability RMSE 0.387 mm, P95
0.510 mm, and maximum 0.548 mm.

Six fused/robot point pairs were fitted with a rigid Kabsch transform. Training
RMSE was 6.322 mm and maximum residual 9.480 mm. P7 was kept as independent
validation (approximately 9.6 mm). An unrestricted affine fit was tested but
rejected because the geometry was poorly conditioned and the non-rigid matrix
was not physically appropriate for coordinate frames.

## Task 9 — Robot-referenced accuracy validation

`accuracy_recorder.py` records accepted synchronized fusion diagnostics and
all intermediate estimates. At Position 1, three robot touches gave median
ground truth `[240.614, -3.322, -77.417] mm`. The robot-corrected camera mean
was `[239.149, 8.600, -76.749] mm`, yielding error
`[-1.465, 11.922, 0.668] mm`, 12.034 mm RMSE, 12.426 mm P95, and 12.529 mm
maximum.

Position 2 produced 18.758 mm RMSE and was excluded from acceptance, but is
retained under the results directory for traceability. The current result is
therefore a provisional local validation, not a full-workspace calibration.

## Task 10 — Supervised guarded marker approach

The approach workflow homes the arm, waits for user confirmation, reads the
robot-corrected marker target, opens the gripper, and follows segmented
waypoints. Seven-step, three-step, 20 mm, and 10 mm variants are preserved so
the changes are auditable. The viewer overlays detected marker information.

A guarded pre-approach test used planned `[280.733, -0.851, -25.443] mm` and
measured `[279.284, -11.574, -25.481] mm`, approximately 10.82 mm apart. The
10 mm-above-marker variant must therefore be treated as supervised research
code only: the known coordinate error can be as large as the commanded
clearance. Do not use it for autonomous contact or collision-sensitive grasping.
