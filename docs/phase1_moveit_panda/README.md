# Hello MoveIt — End-Effector Planning & Pick-and-Place (Panda Proxy)

Phase 1 tutorial for the dual-arm mobile manipulator research project: plan,
execute, and grasp with MoveIt 2 and pymoveit2, using the Franka Panda in
simulation as a stand-in until the lab's RoArm-M3-S dual-arm URDF is available.

## Environment

- Ubuntu 22.04.5 LTS (Jammy)
- ROS 2 Humble
- MoveIt 2.5.9 (`ros-humble-moveit-*` package series)
- Workspace: `~/ros2_ws` (`panda_gz_moveit2` + `pymoveit2` + `moveit_ee_tutorial` + `pick_and_place_demo`)

Add to `~/.bashrc`:

## Quick Start

**Terminal 1 — RViz + MoveIt (fake hardware):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch panda_moveit_config ex_fake_control.launch.py
```
This starts `move_group` and RViz with a fake (non-physics) hardware interface.

> Alternative launch files available in `panda_moveit_config`:
> - `move_group.launch.py` — core MoveIt planning node only
> - `ex_ign_control.launch.py` — runs with Ignition/Gazebo physics simulation

**Terminal 2 — built-in pymoveit2 examples:**
```bash
source ~/ros2_ws/install/setup.bash

# Cartesian pose goal (end-effector)
ros2 run pymoveit2 ex_pose_goal.py --ros-args \
  -p position:="[0.5, 0.0, 0.25]" \
  -p quat_xyzw:="[1.0, 0.0, 0.0, 0.0]" \
  -p cartesian:=False

# Gripper
ros2 run pymoveit2 ex_gripper.py --ros-args -p action:="toggle"

# Collision object + replan
ros2 run pymoveit2 ex_collision_primitive.py --ros-args \
  -p shape:="box" -p action:="add" \
  -p position:="[0.5, 0.0, 0.5]" -p dimensions:="[0.1, 0.1, 0.1]"
```

**Terminal 2 — custom pick-and-place demo:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pick_and_place_demo pick_and_place
```
Adds a ground plane and a target box to the planning scene, then executes a
full pick-and-place sequence: pre-grasp → open gripper → descend → close
gripper + attach → retreat → move to place pose → open gripper + detach.

## What You Should See

- RViz2: Panda arm model, planned path animating through each motion.
- Terminal: sequential log messages for each stage (`加入地板...`, `移動到物體上方...`,
  etc.), with no `Planning failed!` errors between steps.

## Known Issues & Notes

**Gripper attach requires `touch_links`, or planning fails after grasp:**
`attach_collision_object()` defaults to `touch_links=[]`. Without specifying
the gripper finger links, the fingers' contact with the grasped object is
misclassified as a collision, and the robot's post-attach state is treated as
already in collision — causing all subsequent planning calls to fail with
`Planning failed! Error code: FAILURE`. Fix:
```python
moveit2.attach_collision_object(
    id=box_id,
    touch_links=["panda_leftfinger", "panda_rightfinger"],
)
```

**`gripper_action_controller/gripper_cmd` action has no server:**
`GripperInterface` was configured with
`gripper_command_action_name="gripper_action_controller/gripper_cmd"`, but
`ros2 action info` shows 0 clients/servers for this action. The gripper is
actually driven through
`/gripper_trajectory_controller/follow_joint_trajectory`
(`control_msgs/action/FollowJointTrajectory`), the same mechanism used for arm
trajectory execution via the MoveIt controller manager.

**Fake hardware has no physics — grasp success is logic-only, not physical:**
`ex_fake_control.launch.py` has no physics engine, gravity, or contact-force
feedback. `gripper.close()` always moves to the commanded joint target
regardless of obstruction, and `attach_collision_object` binds the object to
the gripper purely in software. Grasp success does not verify real physical
contact. For physics-validated grasping, use `ex_gz_control.launch.py`
(Ignition Gazebo) instead.

**Ground-object coordinate consistency:**
When placing a collision box on the ground plane, `box_position[2]` must equal
`box_size[2] / 2`, and `grasp_position` should align to the box's center
height (not the ground). Mismatched values here were the root cause of
inaccurate grasp behavior during early testing — see
`pick_and_place_log.md` for the full debugging record.

## Next: Lab Dual-Arm Platform (RoArm-M3-S)

When the lab's dual-arm URDF is available, replace in `pick_and_place.py`:
- `pymoveit2.robots.panda` preset → custom joint names / `group_name` /
  `end_effector_name` for `left_arm`, `right_arm`
- Launch file → your dual-arm `moveit_config` package

The pymoveit2 call pattern (`MoveIt2`, `GripperInterface`, `move_to_pose`,
`attach_collision_object`) stays the same.