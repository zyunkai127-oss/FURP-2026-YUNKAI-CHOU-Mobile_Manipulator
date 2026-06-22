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