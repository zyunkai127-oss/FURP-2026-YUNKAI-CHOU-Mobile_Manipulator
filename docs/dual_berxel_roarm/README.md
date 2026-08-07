# Dual Berxel iHawk100 + RoArm M3 integration

## Purpose

The system detects ArUco marker ID 4 with two RGB-D cameras, converts each
depth observation into `base_link`, corrects the systematic disagreement
between the cameras, fuses the two estimates, maps the fused point into the
RoArm coordinate convention, and performs a supervised guarded approach.

For the complete Chinese Task 1–10 narrative, equations, TF explanation,
problems, solutions, final matrices, and direct training-data links, see
[Task 1–10 完整技術報告](COMPLETE_TASK1_10_REPORT.md).

```text
iHawk RGB + depth
        │
        ▼
ArUco centre + median depth ──► camera-frame 3-D point
        │
        ▼
base/camera TF ──► empirical camera correction ──► weighted fusion
        │
        ▼
fused-to-RoArm rigid correction ──► supervised approach waypoint
```

## Hardware and coordinate assumptions

- Camera 1 serial: `HK100QB6513M2B479`
- Camera 2 serial: `HK100QB5311M2B242`
- Depth/RGB image size: 640 × 400
- Target: `DICT_4X4_50`, marker ID 4
- Calibration board: 9 × 7 squares, 25 mm square side, 8 × 6 = 48 inner corners
- The cameras and robot base must remain fixed after calibration.

## Common ROS environment

Run this in every ROS terminal before starting a node:

```bash
source /opt/ros/humble/setup.bash
source ~/berxel_ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

Start the components in separate terminals:

1. `ros2 launch berxel_camera_ros2 dual_ihawk_bringup.launch.py`
2. Publish the two saved `base_link` camera transforms.
3. `ros2 run dual_berxel_calibration aruco_depth_point_estimator --ros-args -p target_marker_id:=4`
4. `ros2 run dual_berxel_calibration dual_point_fusion`
5. Observe `/fused_target_pose_robot_corrected` and
   `/dual_camera/fusion_status`.

Use `tools/dual_berxel_roarm/aruco_marker_viewer.py` to confirm that ID 4 is
visible before diagnosing ROS topics.

## Current evidence and limitation

Dual-camera fusion repeatability at a static point was 0.387 mm RMSE, but this
does **not** equal absolute robot accuracy. The six-point fused-to-robot rigid
fit had 6.322 mm training RMSE and 9.480 mm maximum residual. An independent
point was approximately 9.6 mm away, while Task 9 Position 1 produced about
12.0 mm robot-referenced error. Therefore motion must remain supervised.
