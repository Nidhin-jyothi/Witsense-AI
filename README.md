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

# Clone repository 
git clone https://github.com/Nidhin-jyothi/Witsense-AI.git

# Move into the workspace root
cd ~/robot_ws

# Build the workspace
colcon build

# Source the setup script before using ROS 2 commands
source install/setup.bash

```

## 📡 Running Sensor Simulation

We begin by testing the sensor_reading package. This package contains:

A publisher node: sends random float values between 0 and 1 on the topic /sensor_x_readings

A subscriber node: listens to /sensor_x_readings and prints a message when values are greater than 0.5

## ✅ Commands to Run
Open a new terminal (and remember to source the workspace in each one):

```bash
source ~/robot_ws/install/setup.bash
```

Then run:

Publisher Node

```bash
ros2 run sensor_reading sensor_publisher
```

Subscriber Node

```bash
ros2 run sensor_reading sensor_subscriber
```