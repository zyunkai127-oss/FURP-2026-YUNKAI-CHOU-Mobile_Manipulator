### Week 0 — 2026-06-08

**Attended this week's meeting:** Yes

**Progress this week**
- Successfully registered and configured GitHub account and linked it with GitHub Desktop.
- Forked and initialized the official `FURP-SEP-Template` repository under the correct 2026 naming convention (`FURP-2026-YUNKAI-CHOU-Mobile_Manipulator`).
- Configured Visual Studio Code with critical extensions including Python and Markdown All in One.
- Initialized the mandatory repository directory structure (`/docs`, `/src`, and `/docs/meeting_notes`).

**Challenges & blockers**
- **Git Workflow Friction:** Encountered an initial issue in GitHub Desktop where the "Commit" button was disabled because the mandatory `Summary` field was left blank. Resolved it by learning that Git strictly requires a clear description for every code changes save point.
- **Directory Typos:** Accidentally misnamed several files during initialization (e.g., `REAMDE.md`, `meetinf_notes`), which required clean-up to ensure compliance with the repository layout standard.

**Next steps**
- Access and read the project's cited paper to grasp the mathematical foundation of Whole-Body Control (WBC) and the whole-body Jacobian matrix.
- Follow the **Phase 1 (MoveIt Onboarding)** guidelines: locate the robot description URDF file in the `src/` directory to inspect its kinematic chains.
- Install Anaconda to set up the Python virtualization environment required for future machine learning/RL control stacks.

**Hours spent (optional):** 3h

**Links (optional):** https://github.com/zyunkai127-oss/FURP-2026-YUNKAI-CHOU-Mobile_Manipulator


### Week 1 — 2026-06-15

**Attended this week's meeting:** Yes

**Plan for this week**
- **Monday:** Perform a complete native installation of ROS2 Humble via `apt` on Ubuntu 22.04, deliberately skipping Docker containers to maintain environmental clarity.
- **Tuesday:** Spend 30 minutes practicing essential Linux terminal commands (`pwd`, `ls`, `cd`, `mkdir`, `cat`, `grep`) to build core navigation and diagnostic efficiency.
- **Wednesday:** Source the setup environment and execute `ros2 doctor` to verify system diagnostics and catch potential localization or installation anomalies early.
- **Thursday:** Run a dual-terminal network validation test using the official `talker` and `listener` nodes from `demo_nodes_py` to visually confirm real-time data flow.
- **Friday:** Create the dedicated local ROS2 workspace structure (`~/ros2_ws/src`), execute a successful test compilation using `colcon build`, and verify the local environment sourcing procedure.

**Progress this week**
- **Virtual Environment Setup:** Successfully deployed an Ubuntu 22.04 LTS (Jammy Jellyfish) virtual machine using VMware to establish a stable and official development environment for ROS 2.
- **System Configuration:** Performed comprehensive locale standardization (`en_US.UTF-8`) to ensure compatibility with robot data processing and compilation tasks.
- **ROS 2 Humble Installation:** - Successfully registered the ROS 2 GPG public key (`F42ED6FBAB17C654`) and configured the `/usr/share/keyrings/` directory to secure the repository chain.
  - Completed the installation of `ros-humble-desktop` and `ros-dev-tools`.
- **Environment Verification:** Verified the installation by successfully launching communication nodes (`talker`/`listener`), confirming that the ROS 2 computation graph is correctly configured.
- **Tooling Standardization:** Installed and configured essential utilities (`curl`, `gnupg2`) to facilitate future package management and network-based tooling.

**Challenges & blockers**
- **GPG Verification Failure (`NO_PUBKEY`):** Encountered a persistent signature verification error while running `sudo apt update`.
  - *Resolution:* Diagnosed the issue as a mismatch in keyring paths and missing network utilities. I resolved this by manually downloading the official ROS 2 key via `curl`, configuring the source list with the `signed-by` parameter, and re-initializing the package index.
- **Missing Dependencies:** Encountered `command not found` errors for `curl` and `wget` on the fresh Ubuntu installation.
  - *Resolution:* Updated the local package database and manually installed the required network utilities to enable the subsequent repository authentication process.
- **GPG Service Initialization:** Experienced issues with `dirmngr` due to a lack of initialized GPG directories.
  - *Resolution:* Opted for a direct file-based installation of the GPG key to bypass network-dependent GPG service failures, ensuring a more robust configuration on the virtualized system.



**Hours spent (optional):** 10h

**Links (optional):** https://github.com/zyunkai127-oss/FURP-2026-YUNKAI-CHOU-Mobile_Manipulator

### Week 2 — 2026-06-22

**Attended this week's meeting:** NO MEETING

**Core Phases for This Week**
- **Phase 1: Look at the Graph:** Run the official Python-based `talker` and `listener` demonstration nodes, and utilize the graphical tool `rqt_graph` to see exactly how they are connected in real-time.
- **Phase 2: Deconstruct the System:** Launch the laboratory's ground robot simulation environment (Bringup), control it using the keyboard teleop, and observe the continuous data streams of odometry (`/odom`) and LiDAR (`/scan`).
- **Phase 3: Inject Your Node:** Write your very first custom ROS2 Python package and node to implement a data-forwarding script, successfully achieving this week's practical milestone.

**Plan for this week**
* **Monday:** Execute the official Python-based `talker` and `listener` demonstration nodes, and utilize the `rqt_graph` tool to visualize the underlying node-to-topic computation graph topology.
* **Tuesday:** Launch the ground robot simulation environment using `ros2 launch ground_robot bringup.launch.py`, and inspect the standard robot model and live 2D LiDAR laser scans within RViz2.
* **Wednesday:** Activate the keyboard teleoperation node, echo the real-time odometry stream via `ros2 topic echo /odom`, and study the closed-loop motion system dynamics between velocity inputs and state outputs.
* **Thursday:** Scaffold a custom ROS2 Python package named `week1_echo` utilizing the `ament_python` build type, and write a structured, object-oriented node that subscribes to an input topic and forwards data.
* **Friday:** Compile the custom package using `colcon build`, source the workspace's local setup file, run the new node, and visually confirm its correct insertion into the computational network using `rqt_graph`.

**Hours spent (optional):** 6h

**Links (optional):** https://github.com/zyunkai127-oss/FURP-2026-YUNKAI-CHOU-Mobile_Manipulator


### Week 3 — 2026-07-05

**Attended this week's meeting:** Yes

**Progress this week**

- Systematically debugged and resolved a critical Ubuntu APT repository conflict (GPG key configuration error) and VMware NAT network failure that blocked package installations
- Repaired corrupted MoveIt environments by forcefully reinstalling core dependencies (`ros-humble-moveit-*` and `moveit_servo`) to resolve OMPL planner and controller manager plugin crashes
- Created a custom ROS 2 Python package (`moveit_ee_tutorial`) following standard directory structures and `setup.py` configurations
- Implemented a Cartesian pose control script (`move_to_pose.py`) utilizing `pymoveit2` and official ROS 2 `geometry_msgs/PoseStamped` messages
- Successfully verified motion planning and Inverse Kinematics (IK) execution for the Panda arm in RViz2 using fake controllers (`ex_fake_control.launch.py`)

**Challenges & blockers**

- Dependency & APT Chain Reaction: A missing `moveit_servo` package exposed a corrupted `/etc/apt/sources.list` (raw PGP key injection), requiring grep-based searches and manual surgical removal to restore `apt update` functionality
- Plugin Load Failures: OMPL and simple controller manager plugins crashed during launch, which was traced back to corrupted package installations and solved via a complete suite `--reinstall`
- Namespace Mismatches: The MoveIt SRDF configuration defined the planning group as `"arm"`, while the initial script targeted `"panda_arm"`, causing communication failures; resolved by aligning the group names in the source code
- API Versioning Traps: Encountered parameter deprecations in the third-party `pymoveit2` library (e.g., `end_effector_name` changes), bypassed by adopting the robust ROS 2 `PoseStamped` standard

**Next steps**

- Move towards Whole-Body Control (WBC) by exploring omni-directional mobile chassis kinematics
- Implement and test collision-aware planning (adding obstacle primitives to the MoveIt planning scene)
- Transition from RViz2 fake controllers to full physics simulation in Gazebo

**Hours spent (optional):** approximately 12h

**Links (optional):** [`src/moveit_ee_tutorial/moveit_ee_tutorial/move_to_pose.py`](../src/moveit_ee_tutorial/moveit_ee_tutorial/move_to_pose.py)

### Week 5 — 2026-07-12

**Attended this week's meeting:** Yes

**Plan for this week**
- Connect the Berxel HK100Q RGB-D camera to the Ubuntu 22.04 virtual machine and verify VMware USB passthrough.
- Install and configure the official Berxel Linux SDK and ROS 2 driver.
- Confirm the units and encoding of the depth-image data.
- Convert selected depth-image pixels into three-dimensional coordinates using the camera intrinsic parameters.
- Develop a manual Region of Interest (ROI) interface for selecting an object and estimating its 3D position.
- Publish the estimated object position as a ROS 2 `PoseStamped` message for later TF2 and MoveIt integration.

**Progress this week**

- **Berxel HK100Q Hardware Integration:** Successfully connected the Berxel iHawk100/HK100Q camera to Ubuntu through VMware USB passthrough. The camera was recognized with USB ID `0603:000a`, with `/dev/video2` and `/dev/video3` device nodes available.

- **SDK and ROS 2 Driver Setup:** Installed and configured Berxel Linux SDK 2.0.161 and built the Berxel ROS 2 driver workspace. The driver successfully published RGB, depth, camera information, and point-cloud data.

- **Camera Configuration Correction:** Discovered that the launch file was loading `berxel_ihawk_100.yaml` instead of the required `berxel_ihawk_100Q.yaml`. Corrected the configuration and enabled RGB-depth alignment and point-cloud generation.

- **Depth Format Verification:** Experimentally confirmed that the depth topic contains integer values in millimetres. For example, a raw value of `2299` represents `2299 mm = 2.299 m`. Therefore, the processing node was configured with `depth_mode:=millimeters` without applying an additional Q12.4 bit shift.

- **Single-Pixel 3D Reconstruction:** Developed `depth_probe.py` to sample a `5 × 5` neighbourhood around a selected pixel, remove zero-depth values, calculate the median depth, convert millimetres to metres, and reconstruct the corresponding 3D point.

- **Camera Intrinsic Calibration Usage:** Read the following intrinsic parameters from the ROS 2 `CameraInfo` message:

```text
fx = 391.3235
fy = 391.3235
cx = 320.8533
cy = 202.9705
```

- **Pixel Back-Projection:** Used the pinhole-camera back-projection equations:

```text
X = (u - cx) × Z / fx
Y = (v - cy) × Z / fy
Z = depth
```

This converts a depth-image pixel `(u, v)` into a metric 3D position `(X, Y, Z)` in the camera optical frame.

- **3D Point Publication:** Successfully published reconstructed points through `/object_point_camera` and the measured depth through `/depth_probe/depth_m`. A centre-image test produced:

```yaml
frame_id: berxel_camera/berxel_camera_depth_optical_frame

point:
  x: 0.000862
  y: 0.000173
  z: 2.299
```

Since the selected pixel was close to the camera principal point, the near-zero X and Y values were consistent with the expected geometry.

- **Manual ROI Pose Estimator:** Developed `manual_roi_pose_estimator.py` and `manual_roi_pose_estimator.launch.py`. The node allows the user to draw a bounding box around a target object, samples the central 40% of the ROI, removes invalid depth measurements, calculates the median depth, and converts the ROI centre into a 3D position.

- **Non-Blocking User Interface:** Implemented the OpenCV ROI interface in a separate thread so that object selection does not block the ROS 2 RGB-D callbacks.

- **ROS 2 Visualization Outputs:** The manual ROI node successfully publishes:

```text
/selected_object_pose_camera
/manual_roi/debug_image
/selected_object_marker
```

The interface supports:

```text
s: Select an ROI
r: Clear the current ROI
q: Close the node
```

- **Manual ROI Validation:** Successfully tested the complete manual-selection pipeline with the following result:

```text
Pixel: (294, 294)
Depth: 0.512 m
XYZ: (-0.035, 0.119, 0.512) m
Frame: berxel_camera/berxel_camera_depth_optical_frame
```

The manually calculated position matched the program output. The result indicates that the selected point was approximately `3.5 cm` to the left of the optical axis, `11.9 cm` below it, and `51.2 cm` in front of the camera.

- **Python Environment Repair:** Resolved a NumPy ABI incompatibility caused by NumPy 2.2.6, which prevented Ubuntu Matplotlib, ROS 2 `cv_bridge`, and Ultralytics from operating together. The environment was stabilized using NumPy 1.26.4 and OpenCV 4.11.0.

- **Project Storage Correction:** Moved the Windows copy of the project outside OneDrive to avoid synchronization conflicts and retained the active Ubuntu ROS 2 package at `~/ros2_ws/src/eye_to_hand_grasp_demo`.

**Challenges & blockers**

- **Incorrect Camera Configuration:** The ROS 2 launch file initially loaded the configuration for the iHawk100 instead of the HK100Q. This prevented reliable RGB-depth correspondence. The issue was resolved by changing the configuration to `berxel_ihawk_100Q.yaml` and enabling alignment.

- **Depth-Encoding Ambiguity:** It was initially unclear whether the depth values represented millimetres or Q12.4 fixed-point data. Direct measurement and comparison confirmed that the driver publishes integer millimetres, preventing an incorrect additional bit shift.

- **Invalid and Noisy Depth Values:** Individual depth pixels occasionally contained zeros or unstable measurements. This was addressed by sampling a local neighbourhood or the central ROI area, rejecting invalid values, and using the median instead of a single measurement.

- **Python ABI Conflict:** NumPy 2.2.6 was incompatible with ROS 2 packages compiled against the NumPy 1.x ABI. Downgrading to NumPy 1.26.4 restored compatibility with `cv_bridge`, Matplotlib, OpenCV, and Ultralytics.

- **Blocking ROI Interface:** Running the OpenCV GUI in the main execution path risked blocking incoming image callbacks. The GUI was moved to a dedicated thread to preserve continuous RGB-D processing.

- **OneDrive Path Interference:** The Windows Desktop directory had been redirected through OneDrive, creating a risk of file synchronization and path conflicts. The actual project was moved to a local non-OneDrive location, with only a shortcut retained on the visible desktop.

**Next steps**

- Measure targets at several known distances using a ruler and quantify the depth error, repeatability, and stability of the estimated 3D coordinates.
- Test how the estimation accuracy changes when the object is located near the centre and edges of the image.
- Integrate Ultralytics YOLO to replace the manual ROI with automatic object detection while reusing the existing depth-filtering and back-projection modules.
- Configure TF2 to transform the detected pose from `berxel_camera/berxel_camera_depth_optical_frame` into the Panda robot base frame, `panda_link0`.
- Publish and verify the transformed target pose in RViz2.
- Send the base-frame target pose to MoveIt for inverse kinematics, collision-aware motion planning, and physical grasp execution.

**Hours spent (optional):** Approximately 15h

**Links (optional):**
- [`depth_probe.py`](../src/eye_to_hand_grasp_demo/eye_to_hand_grasp_demo/depth_probe.py)
- [`manual_roi_pose_estimator.py`](../src/eye_to_hand_grasp_demo/eye_to_hand_grasp_demo/manual_roi_pose_estimator.py)
- [`manual_roi_pose_estimator.launch.py`](../src/eye_to_hand_grasp_demo/launch/manual_roi_pose_estimator.launch.py)

---

## Week ending 7 August 2026 — Dual iHawk calibration, fusion and RoArm approach

**Completed**

- Brought up two serial-bound Berxel iHawk100 RGB-D cameras and measured their
  rates, intrinsics, TF trees, and RGB timestamp skew.
- Verified that simultaneous depth streaming did not materially degrade a
  controlled 20 × 20 ROI.
- Built and validated a rigid 9 × 7 checkerboard plus four-ID orientation
  target; both cameras detected all 48 inner corners below 1 px reprojection
  error.
- Calibrated the dual-camera transform with 30 diverse board poses and
  validated three independent board locations below 2 mm mean 3-D error.
- Implemented ID-4 RGB-D localization, camera-frame back-projection, base-frame
  transforms, empirical inter-camera correction, buffered time matching, and
  inverse-variance fusion.
- Connected the RoArm M3 over CP210x serial, captured fused/robot point pairs,
  fitted a rigid fused-to-robot correction, and tested an independent point.
- Implemented accuracy recording, an annotated marker viewer, and supervised
  guarded approach variants ending 20 mm or 10 mm above the marker.

**Key results**

- Improved camera extrinsic fit: 2.081 mm translation RMSE, 0.202° rotation RMSE.
- Static fused-point repeatability: 0.387 mm 3-D RMSE, 0.548 mm maximum.
- Six-point fused-to-RoArm fit: 6.322 mm RMSE, 9.480 mm maximum residual.
- Independent/local robot-referenced checks: approximately 9.6–12.0 mm error.

**Problems resolved**

- Standardized CycloneDDS/localhost environments across every terminal and
  repaired stale daemon, participant-index, QoS, and workspace-overlay issues.
- Replaced single-message RGB/depth pairing with nearest-timestamp buffers.
- Connected previously separate camera and `base_link` TF trees and retained an
  explicitly labelled calibration fallback.
- Distinguished marker occlusion from ROS communication failure using a live
  annotated viewer.

**Limitation and next step**

The sub-millimetre repeatability does not remove the approximately 10–12 mm
absolute robot bias. The present system is suitable only for supervised
coordinate testing. The next calibration should use more non-coplanar points,
a mechanically defined TCP, consistent tilt/roll, and held-out workspace
validation before autonomous low-clearance grasping.

Full procedures, equations, raw evidence, and troubleshooting are documented
in [`dual_berxel_roarm`](dual_berxel_roarm/README.md).
