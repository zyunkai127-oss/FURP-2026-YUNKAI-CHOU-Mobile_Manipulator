# Eye-to-Hand MoveIt 2 Grasp Demo (Panda)

This ROS 2 Humble package implements the first project milestone:

`manual object pose (camera) -> TF2 -> object pose (robot base) -> explicit IK check -> MoveIt plan/execute -> close -> attach -> lift`

It targets the Panda configuration already used by this repository. Panda's
actual pymoveit2/SRDF names are `arm` and `gripper`; its base and planning tip
are `panda_link0` and `panda_hand_tcp`. Panda has seven arm joints, so the IK result has
seven values even though the general project specification says six DoF.

## Install

Copy or symlink this directory into the Ubuntu ROS workspace, then build:

```bash
cd ~/ros2_ws/src
ln -s /path/to/this/repository/src/eye_to_hand_grasp_demo .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select eye_to_hand_grasp_demo
source install/setup.bash
```

## Run

Terminal 1 starts the existing Panda MoveIt fake-hardware simulation:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```

Terminal 2 starts the fixed camera TF and grasp pipeline:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch eye_to_hand_grasp_demo demo.launch.py
```

For a safe IK-only test (no robot execution):

```bash
ros2 launch eye_to_hand_grasp_demo demo.launch.py \
  execute:=false
```

The launch file places the fixed camera at `[0.9, -0.4, 0.9]` m. The default
manual detection `[0, 0, 1.084481443]` m in `camera_color_optical_frame`
transforms to approximately `[0.50, 0.20, 0.09]` m in `panda_link0`.

To see the object marker in RViz, add a **Marker** display and select
`/object_marker`. Set RViz's fixed frame to `panda_link0`.

## Change the manual detection

Edit `config/grasp_demo.yaml`, or override the camera-frame position:

```bash
ros2 run eye_to_hand_grasp_demo grasp_demo --ros-args \
  --params-file ~/ros2_ws/install/eye_to_hand_grasp_demo/share/eye_to_hand_grasp_demo/config/grasp_demo.yaml \
  -p object_position_camera:="[0.02, -0.03, 1.00]"
```

Always keep the input pose in the camera frame. The node transforms it and only
sends `panda_link0` poses to MoveIt.

## Scope and Phase 2 boundary

The visible bottle is an RViz marker during approach. It becomes a MoveIt
attached collision object after the fake gripper closes. This avoids ordinary
collision checking rejecting intentional finger/object contact; it does not
prove a physical grasp. In Phase 2, use Gazebo physics for the object and a
contact-aware grasp controller, while retaining the TF and grasp-pose code in
this package.
