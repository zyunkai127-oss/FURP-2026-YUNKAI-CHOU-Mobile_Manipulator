# Code and data inventory

## Project code

| Path | Purpose |
| --- | --- |
| `src/berxel_camera_ros2` | Berxel ROS 2 driver modifications, configuration, dual launch, RViz config |
| `src/dual_berxel_calibration` | Checkerboard pose, extrinsic calibration, ArUco depth, fusion, accuracy recording |
| `src/roarm_driver` | Minimal RoArm ROS driver package used by this project |
| `tools/dual_berxel_roarm` | Standalone checkers, viewers, ROI analysis, validation and approach scripts |

The full Waveshare SDK and MoveIt distribution are external dependencies and
are intentionally not vendored. Development used `~/waveshare_roarm_sdk` and
`~/roarm_ws`; the approach scripts import `roarm_sdk.roarm` and communicate
with the CP210x device at `/dev/ttyUSB0`, 115200 baud.

## Experimental evidence

All imported evidence is under `experiment/dual_berxel_roarm/results`:

| Directory | Contents |
| --- | --- |
| `task1_results` | USB enumeration/tree, serial mapping, topic/node lists, rates, timestamps, CameraInfo, TF graph |
| `task2_results` | Single/dual-camera depth ROI CSV and JSON summaries |
| `task3_results` | Checkerboard and ArUco annotated validation images |
| `task7_results` | Four paired base-frame point datasets and empirical camera comparison |
| `task8_results` | Fused repeatability and P2–P7 fused/robot calibration data |
| `task9_results` | Position 1 accepted run, incomplete/rejected runs, excluded Position 2 and summaries |
| `task10_results` | Guarded approach test record |

Tasks 4–6 do not have separate result folders because their durable outputs are
the annotated Task 3 images and YAML calibration files in
`src/dual_berxel_calibration/config`.

## Important configuration status

- `dual_camera_extrinsics.yaml`: accepted improved 28/30 dual-camera solution.
- `dual_camera_extrinsics_failed_123mm.yaml`: failure retained for audit only.
- `base_camera_extrinsics_corrected.yaml`: three-point corrected base/camera
  estimate with one validation point; do not move hardware.
- `camera1_empirical_correction.yaml`: makes Camera 1 agree with Camera 2.
- `fused_to_robot_correction.yaml`: provisional six-point rigid robot mapping;
  approximately 10 mm independent error.
- `partner_board_to_robot_affine.yaml` and
  `base_camera_extrinsics_provisional.yaml`: historical/provisional inputs, not
  final autonomous-motion calibration.

## Reproducibility rule

Raw CSV/JSON/YAML and failed or excluded trials are retained. Derived values in
the documentation are traceable to these files. Do not overwrite a calibration
failure with a successful file; create a new labelled result and record which
configuration produced it.
