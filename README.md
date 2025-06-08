# 🤖 UGV Robotics Project using ROS 2 Humble and Gazebo

This project simulates an Unmanned Ground Vehicle (UGV) in a 3D environment using ROS 2 (Humble) and Gazebo. It includes multiple ROS packages that handle sensor simulation, UGV modeling, object detection, and navigation.

---

## 📦 Packages Included

- `sensor_reading`: Publishes and subscribes to simulated sensor data
- `gazebo_test`: Launches basic simulation world and robot
- `yolobot_gazebo`: Integrates the robot model and sensors into Gazebo
- `yolobot_recognition`: Implements person/object detection using YOLO
- `yolov8_msgs`: Custom message definitions for YOLOv8 inference

---

## 🛠️ Setup Instructions

### 🐧 ROS 2 Humble & Workspace Setup

```bash
# Create the workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clone your repository (replace with your actual repo URL)
git clone https://github.com/Nidhin-jyothi/Witsense-AI.git

# Move into the workspace root
cd ~/robot_ws

# Build the workspace
colcon build

# Source the setup script before using ROS 2 commands
source install/setup.bash
