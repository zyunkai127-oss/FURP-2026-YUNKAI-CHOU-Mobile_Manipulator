# Dual Berxel / RoArm standalone tools

Run after sourcing the ROS environment described in
[`docs/dual_berxel_roarm/README.md`](../../docs/dual_berxel_roarm/README.md).

| Script | Task | Purpose |
| --- | --- | --- |
| `task2_depth_roi_stats.py` | 2 | Depth ROI statistics and invalid-pixel check |
| `check_dual_checkerboard.py` | 3 | Confirm both cameras detect all 48 checkerboard corners |
| `validate_dual_camera_extrinsics.py` | 4–5 | Validate the dual-camera extrinsic calibration |
| `aruco_marker_viewer.py` | 6–9 | Live annotated view of ArUco ID 4 (occlusion vs ROS failure) |
| `task10_guarded_approach.py` | 10 | Supervised guarded approach (current version) |
| `task10_marker_approach_10mm.py` | 10 | **Final** marker approach, stops 10 mm above target |

`archive/` keeps earlier Task 10 variants (20 mm clearance, 7-step, pre-home and
safe-backup versions) for reference only.
