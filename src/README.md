# ROS 2 packages (`src/`)

Build in a colcon workspace (ROS 2 Humble):

```bash
cd ~/berxel_ros2_ws && colcon build --symlink-install
source install/setup.bash
```

| Package | Language | Purpose |
| --- | --- | --- |
| [`berxel_camera_ros2`](berxel_camera_ros2) | C++ | Berxel iHawk100 RGB-D camera driver |
| [`dual_berxel_calibration`](dual_berxel_calibration) | Python | Dual-camera extrinsics, ArUco depth localisation, point fusion, accuracy recording |
| [`roarm_driver`](roarm_driver) | Python | RoArm serial (CP210x) driver |
| [`eye_to_hand_grasp_demo`](eye_to_hand_grasp_demo) | Python | Eye-to-hand grasp demo with launch file and config |

Phase 1 (MoveIt 2 + Panda simulation) notes have moved to
[`docs/phase1_moveit_panda`](../docs/phase1_moveit_panda/README.md).
