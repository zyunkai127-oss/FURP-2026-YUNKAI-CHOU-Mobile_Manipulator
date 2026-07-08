# Panda Single-Arm Pick-and-Place Experiment Log

## Objective
Implement a complete pick-and-place pipeline on the Panda single-arm simulation
(fake hardware): add scene objects → move above target → open gripper → descend
→ close gripper + attach object → retreat → move to place position → open
gripper + detach object.

## Packages Used
- `pymoveit2` (`MoveIt2` and `GripperInterface` interfaces)
- Custom package: `pick_and_place_demo`

## Script Path
```python
~/ros2_ws/src/pick_and_place_demo/pick_and_place_demo/pick_and_place.py
```
## Issues Encountered and Debugging Log

### Issue 1: Low grasp accuracy — the gripper appeared to "pick up" the object while still above it
- **Symptom**: The program logic executed without errors, but visually the gripper never actually descended to the box's height before the object was carried away.
- **Root Cause**: The `grasp_position` height was not recalculated to match the box's actual resting height after `box_size` was changed — the coordinates were inconsistent, so the descend motion never actually reached the box's height.
- **Additional Finding**: In `fake hardware` mode, there is no physics engine. `attach_collision_object` purely binds the object's coordinates to the gripper in software — it does not check whether physical contact actually occurred. Even if the gripper is hovering with no contact at all, calling `attach` is enough for the object to be treated as "grasped" and moved along with the gripper.
- **Fix**: Unified the coordinate logic — `box_position` height = `box_size / 2` (resting on the ground), `grasp_position` aligned to the box's center height, and `pre_grasp_position` set clearly above the box (a safe hover height).

### Issue 2: The object appeared to pass through the floor
- **Symptom**: During motion, the arm/object visually appeared to pass straight through the ground plane.
- **Investigated Causes**: Suspected a delay in planning scene updates, or the collision object not being synchronized into the planner in time.
- **Follow-up**: This was not observed again after the coordinate fix in Issue 1, and may have been related to the incorrect ground-resting height calculation — worth continuing to monitor.

### Issue 3: Planning failed after closing the gripper and attaching the object
- **Symptom**: Terminal output showed:
```python
Planning failed! Error code: FAILURE
Cannot execute motion because the provided/planned trajectory is invalid.
Cannot wait until motion is executed (no motion is in progress).
```

As a result, subsequent motions ("retreat", "move to place position") never actually executed, and the arm remained stationary.
- **Root Cause**: Inspecting the `pymoveit2` source code, specifically the `attach_collision_object()` definition in `moveit2.py`, revealed that the `touch_links` parameter defaults to an empty list `[]`. This means no links were permitted to legitimately contact the attached object. When the gripper fingers closed onto the box, this contact was misclassified as a collision, putting the robot's starting state into collision as soon as the object was attached — preventing the OMPL planner from planning any further motion from that state.
- **Fix**: Explicitly specify `touch_links` when calling `attach_collision_object`, allowing the gripper fingers to contact the object:
```python
  moveit2.attach_collision_object(
      id=box_id,
      touch_links=["panda_leftfinger", "panda_rightfinger"],
  )
```
  (Finger link names taken from the gripper group definition in `panda.srdf`.)
- **Side Discovery**: By reading the source code, confirmed that `attach_collision_object` is implemented via a topic publish to `/attached_collision_object` (type `moveit_msgs/msg/AttachedCollisionObject`), rather than a service call — this refined a detail in the system's data-flow diagram.

### Issue 4: Whether the gripper's opening width is sufficient to grasp the object
- **Verification**: Checked `pymoveit2/robots/panda.py`:
```python
  OPEN_GRIPPER_JOINT_POSITIONS = [0.04, 0.04]
  CLOSED_GRIPPER_JOINT_POSITIONS = [0.0, 0.0]
```
  The Panda gripper uses prismatic (linear) joints; each finger can move 0.04 m outward, giving a maximum opening of roughly 0.08 m — sufficient to grasp a 0.06 m box.
- **Important Limitation Identified**: `fake hardware` has no physics engine or collision detection. `gripper.close()` simply moves the joint to the commanded target position regardless of obstruction, and does not stop early upon contact or report any contact-force feedback. The current system has no way to "sense" whether an object is actually grasped — success depends entirely on software logic (`attach_collision_object`) rather than physical verification.

## Takeaways and Next Steps
1. Coordinate consistency is critical: object size, resting height, and grasp height must all be derived from a single consistent formula to avoid mismatched manually-entered values.
2. `fake hardware` is well suited for validating MoveIt's planning logic and the attach/detach mechanism, but cannot validate real physical grasp feasibility (no gravity, no collision detection, no contact-force feedback).
3. If real grasp feasibility needs to be validated, the next step should be switching to a simulation environment with a physics engine (e.g. `ex_ign_control.launch.py`, using Ignition Gazebo), to obtain real gravity, collision, and friction simulation.
4. While debugging the `touch_links` issue, directly reading the `pymoveit2` source code to confirm actual API behavior proved more reliable than guessing from documentation or assumption — a good standard practice to carry forward for future debugging.