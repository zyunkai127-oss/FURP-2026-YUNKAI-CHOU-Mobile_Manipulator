# 🤖 Project Mobile Manipulator WBC: Week 2 Full Research Log
> **Date:** 2026-02-23 (Tuesday)  
> **Author:** yunkai_chou  
> **Environment:** Ubuntu 22.04 LTS (Humble) - Air-Gapped Virtual Machine Sandbox  

---

## 📐 1. System Architecture & Computational Graph
The framework utilizes a completely decoupled, asynchronous, modular design pattern typical in professional robotics middleware. This pattern separates hardware sensing interfaces from our safety-critical closed-loop optimization brains, ensuring strict timing compliance.

### Graph Elements Breakdown:
* 🟢 **`sensor_node (Publisher)`**: Acts as the robot's physical sensor hardware (e.g., LiDAR array). It measures environmental states and periodically broadcasts data packets.
* 🧠 **`control_node (Subscriber)`**: Acts as the central vehicle controller (the robot's brain). It intercepts incoming topic packets asynchronously and runs algorithmic logic to issue safety triggers or actuation parameters.
* 🧬 **`Topic (/sensor_data)`**: A strongly typed unidirectional data pipeline handling the transport layer over DDS without requiring nodes to discover network origins explicitly.

---

## 💻 2. Core Python Node Communication Implementation

### 🏎️ Component 1: Sensor Node (`sensor_node.py`)
This node instantiates a periodic timer to actively broadcast raw environmental detection strings into the active DDS network topology.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher_node')
        self.publisher_ = self.create_publisher(String, '/sensor_data', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Sensor Publisher Node has initialized successfully!')

    def timer_callback(self):
        msg = String()
        msg.data = 'Obstacle detected at 0.5 meters!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Broadcasting Data Packet: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 🧠 Component 2: Controller Node (control_node.py)
An event-driven script that runs string-parsing evaluations to execute critical safety braking commands when spatial boundaries are breached.

```Python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ControllerSubscriber(Node):
    def __init__(self):
        super().__init__('controller_subscriber_node')
        self.subscription = self.create_subscription(
            String, 
            '/sensor_data', 
            self.listener_callback, 
            10)
        self.get_logger().info('Controller Subscriber Node has initialized successfully!')

    def listener_callback(self, msg):
        self.get_logger().info(f'Brain Processed Payload: "{msg.data}"')
        if '0.5 meters' in msg.data:
            self.get_logger().warn('CRITICAL BRAKING SEQUENCE TRIPPED: Obstacle proximity limit breached!')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```
### 🛠️ Component 3: Build Tool Registry Configuration (setup.py)
Maps local Python file execution scopes directly into executable paths recognized by the ROS2 backend CLI parser tool layer (ros2 run).

```Python
import os
from glob import glob
from setuptools import setup

package_name = 'week2_echo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunkai_chou',
    maintainer_email='yunkai_chou@todo.todo',
    description='ROS2 automated node perception and bringup execution package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'echo_executable = week2_echo.echo_node:main',
            'run_sensor = week2_echo.sensor_node:main',
            'run_control = week2_echo.control_node:main'
        ],
    },
)
```

### 🛠️ 3. HopperTrex Simulation Bringup & Kinematic Tree Validation
#### 📂 Phase 1: Air-Gapped Cross-System Asset Transfer (Windows ➔ Ubuntu)
Since the virtual machine is strictly offline, a secure local data bridge was constructed using VMware Shared Folders to bypass network constraints.

1. Shared Folder Mounting Channel Restoration: Resolved the reboot mounting failure where the /mnt/hgfs/ mountpoint became inactive by manually calling the FUSE file system agent:

```python
sudo mkdir -p /mnt/hgfs
sudo vmhgfs-fuse .host:/ /mnt/hgfs -o allow_other -o uid=$(id -u)
```

1. Handling String Paths with Spaces & File Ingestion: Utilized string tokenization escape parameters (wrapping paths in double quotes "" to bypass folder spaces) to copy the CAD-exported robot.urdf description asset into the ROS2 workspace tree:

```python
cp "/mnt/hgfs/Ubuntu file transfer/hoppertrex_mjlab/assets/hoppertrex/robot.urdf" ~/mobile_manipulator_ws/src/week2_echo/week2_echo/
```

#### 🔍 Phase 2: Robot Kinematic Model Double-Verification
Conducted granular physical structure audits natively through the command line to verify parent-child coordinate assignments before triggering a graphical layout.

1. XML Syntax & Structural Evaluation:

```python
check_urdf robot.urdf
```
Output: Successfully Parsed XML. Successfully generated the unified parent-child link layout branching outward from the main floating torso frame (chassis_base) down to the respective wheel joints.

1. Actuator Registry Filtering via Regex Interception:

```python
grep "<joint name=" robot.urdf
```

Engineering Observation: Identified 6 active continuous degree-of-freedom (DoF) joints (thigh_left/right, knee_left/right, wheel_left/right) and caught an asymmetric CAD naming convention where the right wheel link was tagged as wheel_2.

#### 🏗️ Phase 3: Autonomous Launch Architecture Engineering
To consolidate process handling, a dedicated Python launch configuration is added directly into the week2_echo workspace package to pull up the hardware state engine, interactive sliders, and graphic workspace simultaneously.

1. Injecting bringup.launch.py File Stream:

```python
cat << 'EOF' > ~/mobile_manipulator_ws/src/week2_echo/launch/bringup.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = os.path.expanduser('~/mobile_manipulator_ws/src/week2_echo/week2_echo/robot.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
EOF
```

1. Workspace Reconstruction and Rebuild Sequence:

```python
cd ~/mobile_manipulator_ws
colcon build --packages-select week2_echo
source install/setup.bash
```

Compilation Output: Finished <<< week2_echo [0.60s] compiled cleanly without errors.

#### 🚀 Phase 4: Data Visualization & Live Sensor-State Aggregation
1. Executing Unified Launch Bringup:

```python
ros2 launch week2_echo bringup.launch.py
```

1. RViz2 Workspace Calibration:
-Fixed Frame: Set to the absolute base transform link: chassis_base.
-Displays Panel: Click Add -> Select RobotModel (under By display type tab) and TF coordinate layers.

1. Live Telemetry Pipeline Capture:
While manipulating the slider knobs on the GUI viewport to dynamically actuate HopperTrex's limbs, an isolated tracking listener terminal was initialized to verify live closed-loop joint cmd states:
```python
source /opt/ros/humble/setup.bash
ros2 topic echo /joint_states
```