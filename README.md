# AEP Package VR - Driver-in-the-Loop ADAS Platform

A ROS2 Humble package implementing a Driver-in-the-Loop (DiL) testing platform for Advanced Driver Assistance Systems (ADAS) using the MentorPi Acker robotic vehicle with VR integration, emergency braking, and remote steering capabilities.

## 📖 Background

This project implements a Driver-in-the-Loop concept where a human operator controls a physical robotic vehicle in real-time through a simulator-based interface. Unlike purely virtual simulations, this approach uses actual hardware to test ADAS functions under real sensor latency and actuator response constraints. The system provides live sensor feedback and safety interventions while maintaining a safe, controlled testing environment. The platform is built on the MentorPi robotic vehicle equipped with a Raspberry Pi 5, 360° TOF LiDAR, monocular camera with pan/tilt servos, and Ackermann steering. ROS2 serves as the communication framework, enabling modular development through publisher-subscriber patterns across distributed nodes for perception, control, and actuation.

---

## 📋 Table of Contents

- [Background](#background)
- [Features](#features)
- [System Architecture](#system-architecture)
- [File Structure](#file-structure)
- [Prerequisites](#prerequisites)
- [Robot Connection Guide](#robot-connection-guide)
- [Deployment Guide](#deployment-guide)
- [Usage](#usage)
- [Useful Commands](#useful-commands)
- [License](#license)
- [Contributors](#contributors)

---

## ✨ Features

- **Remote Steering Control**: UDP-based steering with Logitech G27 wheel support
- **VR Head Tracking**: Camera servo control via VR headset (Quest 3)
- **Autonomous Emergency Braking (AEB)**: Dynamic brake distance calculation based on vehicle speed
- **LIDAR Integration**: Real-time obstacle detection in 90° front sector
- **Live Camera Stream**: MJPEG stream with overlay information (distance, gear, warnings)
- **Message Overlay System**: Real-time HMI status display on camera feed

---

## 🏗️ System Architecture

```
┌─────────────────┐         ┌──────────────────────────────┐
│   Host PC       │  UDP    │     MentorPi Robot           │
│  (Windows)      ├────────►│     (Raspberry Pi 5)         │
│                 │  6005   │                              │
│  gen_client.py  │         │  ┌────────────────────────┐  │
└─────────────────┘         │  │   gen_steering.py      │  │
                            │  │   (UDP Server)         │  │
                            │  └──────────┬─────────────┘  │
                            │             │                │
                            │  ┌──────────▼─────────────┐  │
                            │  │  lidar_distance_brake  │  │
                            │  │  (AEB Logic)           │  │
                            │  └──────────┬─────────────┘  │
                            │             │                │
        ┌───────────────────┼─────────────┼────────────────┤
        │                   │             │                │
┌───────▼──────┐    ┌───────▼──────┐  ┌──▼─────────┐  ┌──▼──────────┐
│  VR_listener │    │ message_node │  │  stream.py │  │ LIDAR       │
│  (VR → Servo)│    │  (Overlay    │  │  (Camera + │  │ (STL-19P)   │
│              │    │   Aggregator)│  │   MJPEG)   │  │             │
└──────────────┘    └──────────────┘  └────────────┘  └─────────────┘
     UDP 5005            ROS2 Topics      HTTP 5000      /scan_raw
```

**Key ROS2 Topics:**
- `/controller/cmd_vel` (Twist) - Motor commands
- `/scan_raw` (LaserScan) - LIDAR data
- `/lidar/min_distance` (Float32) - Obstacle distance
- `/steering/emergency_brake_active` (Bool) - AEB status
- `/ascamera/camera_publisher/rgb0/image` (Image) - Camera feed
- `ros_robot_controller/pwm_servo/set_state` (SetPWMServoState) - Servo control

---

## 📁 File Structure

**Note**: GitHub repository uses branches to organize folders. Each branch represents a directory in the actual package structure.

```
aep_package_vr/                    # Root package folder
│
├── package.xml                    # ROS2 package manifest (file)
├── setup.py                       # Python package setup (file)
├── setup.cfg                      # Setup configuration (file)
├── README.md                      # This file (file)
├── DEPLOY.md                      # Deployment notes (file)
├── SETUP.md                       # Setup instructions (file)
│
├── aep_package_vr/               # Main Python package folder
│   ├── __init__.py               # Package initializer (file)
│   ├── gen_steering.py           # UDP steering server (file) - ROBOT ONLY
│   ├── lidar_distance_brake.py   # LIDAR processing + AEB (file)
│   ├── message_node.py           # Overlay message aggregator (file)
│   ├── stream.py                 # Camera stream with overlays (file)
│   ├── VR_listener.py            # VR headset pan/tilt control (file)
│   └── aep_launcher.py           # System launcher (file)
│
├── launch/                       # ROS2 launch files folder
│   ├── aep_vr.launch.py          # Full system launch (file)
│   └── safe_mode.launch.py       # Safe mode launch (file) - no VR
│
├── resource/                     # ROS2 resource folder
│   └── aep_package_vr            # Resource marker (file)
│
└── gen_client.py                 # UDP client (file) - HOST PC ONLY (Windows)
```

### File Types Legend:
- **Folders** are indicated with trailing `/`
- **Files** are individual Python scripts or configuration files
- All `.py` files in `aep_package_vr/` folder run on the robot
- `gen_client.py` runs **ONLY** on the Windows host PC

### ⚠️ Important Note on gen_client.py

**`gen_client.py` is ONLY intended for use on the host PC running Windows.** This script:
- Reads Logitech G27 steering wheel inputs (NOT G29)
- Sends UDP commands to the robot on port 6005
- Requires Windows-specific dependencies (pygame, inputs library)
- Should **NOT** be deployed to the robot
- Located in the root package directory, not in `aep_package_vr/` folder

---

## 🔧 Prerequisites

### On Robot (Raspberry Pi 5)
- Ubuntu 22.04 (in Docker container)
- ROS2 Humble
- Python 3.10+
- Required ROS2 packages:
  - `ros_robot_controller_msgs`
  - `geometry_msgs`
  - `sensor_msgs`
  - `std_msgs`

### On Host PC (Windows)
- Python 3.8+
- Logitech G27 steering wheel
- Libraries: `pygame`, `inputs`, `socket`

### On Meta Quest 3 (VR Headset)
- **VR Program**: AEP VR Controller application for camera pan/tilt control
- **Download Link**: [VR Program for Meta Quest 3 (APK)](https://drive.google.com/file/d/1o8UfGv3zOQDEk2olwaVFeMNh2V7p7X_I/view?usp=sharing)
- **Note**: This link provides the VR application APK file needed for the system

**Installation Instructions:**
1. Download the VR program APK from the Google Drive link above
2. Enable Developer Mode on your Meta Quest 3 headset (Settings → System → Developer)
3. Install the APK file on your headset using sideloading (via Meta Quest Developer Hub, SideQuest, or ADB)
4. Ensure the VR headset is connected to the same network as the robot (192.168.149.x)
5. Launch the VR application before starting the full system launch with `ros2 run aep_package_vr aep_vr.launch.py`

**Important**: Verify the downloaded file is the correct APK before installation


---

## 🔌 Robot Connection Guide

### Step 1: SSH into Raspberry Pi

```bash
ssh pi@192.168.149.1
# Password: raspberrypi
```

### Step 2: Enter Docker Container

```bash
# List containers to get container ID
docker ps

# Enter the container (replace <containerID> with actual ID)
docker exec -it -u ubuntu -w /home/ubuntu <containerID> /bin/bash
```

### Step 3: Source ROS2 Environment

**You must run these commands every time you enter the Docker container:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 📦 Deployment Guide

### Overview

The deployment process involves:
1. **Transfer package from host PC to Raspberry Pi** (using SCP)
2. **Copy from Pi to Docker container**
3. **Build the package** (using colcon)

### Detailed Steps

#### 1. Transfer Package to Raspberry Pi

From your **host PC** (Windows PowerShell or Linux terminal):

```bash
# Navigate to the package directory on your host
cd /path/to/aep_package_vr

# Use SCP to copy the entire package to the Pi in a neat working folder
scp -r aep_package_vr pi@192.168.149.1:/home/pi/AEP/
```

When prompted, enter password: `raspberrypi`

#### 2. SSH into Pi and Copy to Docker

```bash
# SSH to Pi
ssh pi@192.168.149.1

# Get Docker container ID
docker ps

# Copy package into Docker workspace
docker cp /home/pi/AEP/aep_package_vr <containerID>:/home/ubuntu/ros2_ws/src/

# Enter Docker
docker exec -it -u ubuntu -w /home/ubuntu <containerID> /bin/bash
```

#### 3. Build the Package

Inside the Docker container:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/ros2_ws

# Build the package
colcon build --packages-select aep_package_vr

# Source the newly built package
source install/setup.bash
```

#### 4. Verify Installation

```bash
# Check if package is recognized
ros2 pkg list | grep aep_package_vr

# List available executables
ros2 pkg executables aep_package_vr
```

Expected output:
```
aep_package_vr aep_launcher
aep_package_vr gen_steering
aep_package_vr lidar_distance
aep_package_vr message_node
aep_package_vr stream
aep_package_vr vr_listener
```

---

## 🚀 Usage

### Launch Commands

The package provides two main launch configurations:

#### 1. Full System Launch (with VR)

Launches all nodes including VR head tracking:

```bash
ros2 run aep_package_vr aep_vr.launch.py
```

**Nodes started:**
- `gen_steering` - UDP steering server
- `lidar_distance` - LIDAR obstacle detection + AEB
- `message_node` - Overlay message aggregator
- `stream` - Camera stream with overlays
- `vr_listener` - VR headset pan/tilt control

#### 2. Safe Mode Launch (without VR)

Launches system without VR control - useful for testing or when VR is unavailable:

```bash
ros2 launch aep_package_vr safe_mode.launch.py
```

**Nodes started:**
- `gen_steering` - UDP steering server
- `lidar_distance` - LIDAR obstacle detection + AEB
- `message_node` - Overlay message aggregator
- `stream` - Camera stream with overlays

### Running the Client (Host PC - Windows)

On your **Windows PC**:

```bash
# Navigate to where you have gen_client.py
cd C:\path\to\gen_client.py

# Run the client
python3 gen_client.py
```

**Client Configuration:**
- Robot IP: `192.168.149.1`
- Port: `6005`
- Steering wheel: Logitech G27 (detected automatically)

### Accessing Camera Stream

Once the system is running, access the camera stream from any browser:

- **Live MJPEG stream**: `http://192.168.149.1:5000/video_feed`
- **Single snapshot**: `http://192.168.149.1:5000/snapshot`

---

## 🛠️ Useful Commands

### File Editing & Viewing

```bash
# View file contents
cat gen_steering.py

# Edit file (paste code, then press Ctrl+D)
cat > gen_steering.py
# ... paste your code here ...
# Press Ctrl+D when done
```

### Navigation

```bash
# Change directory
cd ~/ros2_ws/src/aep_package_vr

# Go back one directory
cd ..

# Go to home directory
cd ~

# List files
ls -la

# Show current directory
pwd
```

### ROS2 Commands

```bash
# List all topics
ros2 topic list

# Monitor a topic
ros2 topic echo /lidar/min_distance

# Show topic info
ros2 topic info /controller/cmd_vel

# List running nodes
ros2 node list

# Show node info
ros2 node info /gen_steering

# List all packages
ros2 pkg list

# List executables in package
ros2 pkg executables aep_package_vr
```

### Building & Sourcing

```bash
# Build specific package
colcon build --packages-select aep_package_vr

# Build with verbose output
colcon build --packages-select aep_package_vr --event-handlers console_direct+

# Clean build
rm -rf build/ install/ log/
colcon build --packages-select aep_package_vr

# Source workspace
source ~/ros2_ws/install/setup.bash
```

### Docker Commands

```bash
# List running containers
docker ps

# List all containers (including stopped)
docker ps -a

# Enter container
docker exec -it -u ubuntu -w /home/ubuntu <containerID> /bin/bash

# Copy file from host to container
docker cp /path/on/pi/file.py <containerID>:/home/ubuntu/destination/

# Copy file from container to host
docker cp <containerID>:/home/ubuntu/file.py /path/on/pi/
```

### Network & Debugging

```bash
# Check if robot is reachable
ping 192.168.149.1

# Monitor network traffic on port 6005
sudo tcpdump -i any port 6005

# Check which process is using a port
sudo lsof -i :6005

# Kill a process
kill -9 <PID>
```

---

## 📝 License

This project is licensed under the **MIT License**.

Copyright (c) 2025 Chalmers University of Technology - Automotive Engineering Project

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

---

## 👥 Contributors

### Students
- Abhay Chouhan
- Linus Fäldt
- Elliot Jonsson
- Fanxiang Liao
- Jingyu Wang
- Prakash Raju Sridharraju

### Supervision
- **Responsible Professor**: Marco Dozza
- **Supervisor**: Rahul Rajendra Pai

### Institution
**Chalmers University of Technology**  
Department of Mechanics and Maritime Sciences  
Gothenburg, Sweden

---

**Project Year**: 2025  
**Course**: Advanced Engineering Project (AEP)

---

**Last Updated**: December 2025
