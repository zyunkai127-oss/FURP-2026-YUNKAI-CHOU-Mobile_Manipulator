# MoveIt + Panda Single-Arm Environment Setup — Troubleshooting Log

## Environment Info
- OS: Ubuntu 22.04.5 LTS (Jammy)
- ROS2 Distro: Humble
- MoveIt Version: 2.5.9 (ros-humble-moveit-* package series)
- VMware Version: [fill in from Windows host: VMware Workstation → Help → About]
- Date: [fill in the date you resolved these issues]

## Issue Log

### Issue 1: Missing `moveit_servo` package
- **Symptom**: `package 'moveit_servo' not found`
- **Root Cause**: Package was not installed
- **Fix**: Attempted to install via `sudo apt install ros-humble-moveit-servo`, which led to the next issue

### Issue 2: Broken GPG key configuration in APT sources
- **Symptom**: `apt update` failed with `Conflicting values set for option Signed-By`
- **Root Cause**: Two ROS source list files coexisted in `/etc/apt/sources.list.d/` — a manually created `ros2.list` (correctly pointing to a keyring file path) and the system's original `ros2.sources` (embedding the full PGP key text directly in the `Signed-By` field). Both defined different signing configurations for the same repository, causing apt to detect a conflict.
- **Fix**: Removed the duplicate `ros2.sources` file, keeping only a single clean `ros2.list`

### Issue 3: VMware virtual machine network failure
- **Symptom**: `apt install` failed with `Temporary failure resolving 'packages.ros.org'`; even the gateway IP was unreachable via ping
- **Root Cause**: VMware's virtual network service (NAT) itself had hung — unrelated to ROS/Ubuntu configuration
- **Fix**: Restarted the Windows host machine, which forced VMware to rebuild its virtual network adapter bindings and restored connectivity

### Issue 4: `moveit_planners_ompl` plugin failed to load, causing `move_group` to crash
- **Symptom**: `Exception while loading planner 'ompl_interface/OMPLPlanner'`; the `move_group` process aborted, which also brought down all controller spawners
- **Root Cause**: Investigated multiple possibilities including missing libraries, symbol mismatches, duplicate library conflicts, and version desynchronization. Ultimately traced to minor corruption in the package installation itself, likely related to the earlier APT key configuration failure.
- **Fix**: Force-reinstalled core MoveIt-related packages using `sudo apt install --reinstall`

### Issue 5: `moveit_simple_controller_manager` plugin also failed to load
- **Symptom**: After reinstalling packages from Issue 4, a similar new error appeared, this time for the controller manager plugin failing to load
- **Root Cause**: Same root cause as Issue 4 — not just OMPL, but multiple MoveIt plugin packages from the same batch were affected
- **Fix**: Reinstalled all `ros-humble-moveit-*` packages at once using `--reinstall` to resolve everything in one pass

### Issue 6: Planning group name mismatch
- **Symptom**: `Cannot find planning configuration for group 'panda_arm'`
- **Root Cause**: The `panda_moveit_config` in use (from the `panda_gz_moveit2` project) defines the planning group name as `"arm"` in both the SRDF and `ompl_planning.yaml`, but the `move_to_pose` script in `moveit_ee_tutorial` was hardcoded to connect to `"panda_arm"` — the naming was inconsistent between the two projects.
- **Fix**: Modified the `move_to_pose` source code to change the group name from `"panda_arm"` to `"arm"`, then rebuilt with `colcon build`

## Installed MoveIt Packages (for environment reproduction reference)