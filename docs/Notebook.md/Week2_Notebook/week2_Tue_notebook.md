# 🤖 Project Mobile Manipulator WBC: Week 2 Laboratory Notebook
> **Date:** 2026-02-23 (Tuesday)  
> **Author:** yunkai_chou  
> **Environment:** Ubuntu 22.04 LTS (Offline/Air-gapped Virtual Machine Sandbox)  

---

## 📐 1. System Architecture & Computational Graph
The framework utilizes a completely decoupled, asynchronous, modular design pattern typical in professional robotics middleware. This pattern separates hardware sensing interfaces from our safety-critical closed-loop optimization brains.

### Graph Elements Explained:
* 🟢 **`sensor_node (Publisher)`**: Acts as the robot's physical sensor hardware (e.g., LiDAR or Ultrasonic array). It measures environmental states and periodically broadcasts data packets.
* 🧠 **`control_node (Subscriber)`**: Acts as the central vehicle controller (the robot's brain). It intercepts incoming topic packets asynchronously and runs algorithmic logic to issue safety triggers or actuation parameters.
* 🧬 **`Topic (/sensor_data)`**: A strongly typed unidirectional data pipeline handling the transport layer over DDS without requiring nodes to discover network origins explicitly.

---

## 💻 2. Core Python Implementation & Structural Analysis

### 🏎️ Component 1: Sensor Node (`sensor_node.py`)
This node instantiates a periodic timer to actively broadcast raw environmental detection strings into the active DDS network topology.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SensorPublisher(Node):
    def __init__(self):
        # Initialize the base Node with a distinct registration name in the graph
        super().__init__('sensor_publisher_node')
        
        # Create a publisher pipeline: (DataType, 'Topic Name', Queue/History Buffer Depth)
        self.publisher_ = self.create_publisher(String, '/sensor_data', 10)
        
        # Setup a time-driven mechanism: Triggers the callback loop every 1.0 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Sensor Publisher Node has initialized successfully!')

    def timer_callback(self):
        # 1. Instantiate an empty, standard ROS2 String container
        msg = String()
        # 2. Assign the localized measurement payload
        msg.data = 'Obstacle detected at 0.5 meters!'
        # 3. Stream the message into the active network topology
        self.publisher_.publish(msg)
        self.get_logger().info(f'Broadcasting Data Packet: "{msg.data}"')

def main(args=None):
    # Initialize the underlying DDS communication layer
    rclpy.init(args=args)
    node = SensorPublisher()
    try:
        # Enter an efficient execution spin to block and process periodic timer events
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 🛠️ Component 3: Build Tool Registry Configuration (`setup.py`)
This configuration maps local Python file execution scopes directly into executable paths recognized by the ROS2 backend CLI parser tool layer (`ros2 run`).

```python
from setuptools import setup
import os
from glob import glob

package_name = 'week2_echo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yunkai_chou',
    maintainer_email='yunkai_chou@todo.todo',
    description='ROS2 automated node perception execution package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Alias_Command = Package_Directory.Script_File_Name:Main_Function
            'run_sensor = week2_echo.sensor_node:main',
            'run_control = week2_echo.control_node:main'
        ],
    },
)

```

---

## 📋 3. Standard Operating Procedure (SOP) Development Workflow

### Phase 1: Terminal Code Ingestion (No-GUI Text Editing via EOF streams)
To edit or deploy your Python scripts directly without access to an interactive desktop GUI editor, leverage the native shell file stream redirection tool wrapper inside your Ubuntu bash window terminal.

#### 1. Ingesting `sensor_node.py`:
```bash
cat << 'EOF' > ~/mobile_manipulator_ws/src/week2_echo/week2_echo/sensor_node.py
# Paste Sensor Publisher code here
EOF
```
### 📓 Cell 10 (Markdown): Compilation Sweep and Execution SOP
```markdown
### Phase 2: System Compilation Sweep
Every modification made within a package source block requires a rigid workspace build-refresh cycle executed at the workspace root directory:

```bash
# Move to workspace root directory
cd ~/mobile_manipulator_ws

# Recompile the localized software bundle
colcon build --packages-select week2_echo

# Refresh terminal environment path markers (Crucial Step!)
source install/setup.bash
```

### Phase 3: Distributed Execution Setup
Open two separate bash shell windows to trigger and observe the live network data transmission streams:

#### Terminal Window 1 (Perception Hardware Stream)
```python
source ~/mobile_manipulator_ws/install/setup.bash
ros2 run week2_echo run_sensor
```
#### Terminal Window 2 (Control Logic Evaluation Brain)
```python
source ~/mobile_manipulator_ws/install/setup.bash
ros2 run week2_echo run_control
```