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

**Challenges & blockers**


**Next steps**


**Hours spent (optional):** 3h

**Links (optional):** https://github.com/zyunkai127-oss/FURP-2026-YUNKAI-CHOU-Mobile_Manipulator