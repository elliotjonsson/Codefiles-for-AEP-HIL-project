# AEP Package VR - Advanced Robotics Control System

A ROS2 Humble package for autonomous vehicle control with VR integration, emergency braking, and remote steering capabilities for the MentorPi Acker robot platform.

## 📋 Table of Contents

- [Features](#features)
- [System Architecture](#system-architecture)
- [File Structure](#file-structure)
- [Prerequisites](#prerequisites)
- [Robot Connection Guide](#robot-connection-guide)
- [Deployment Guide](#deployment-guide)
- [Usage](#usage)
- [Node Documentation](#node-documentation)
- [Useful Commands](#useful-commands)
- [Troubleshooting](#troubleshooting)

---

## ✨ Features

- **Remote Steering Control**: UDP-based steering with Logitech G29 wheel support
- **VR Head Tracking**: Camera servo control via VR headset (Quest 3)
- **Autonomous Emergency Braking (AEB)**: Dynamic brake distance calculation based on speed
- **LIDAR Integration**: Real-time obstacle detection in 90° front sector
- **Live Camera Stream**: MJPEG stream with overlay information (distance, gear, warnings)
- **Message Overlay System**: Real-time status display on camera feed

---

## 🏗️ System Architecture

```
┌─────────────────┐         ┌──────────────────┐
│   Host PC       │  UDP    │   MentorPi Robot │
│  (Windows)      ├────────►│   (Raspberry Pi) │
│                 │  6005   │                  │
│  gen_client.py  │         │  gen_steering.py │
└─────────────────┘         └──────────────────┘
                                      │
                            ┌─────────┴─────────┐
                            │                   │
                      ┌─────▼──────┐    ┌──────▼──────┐
                      │   LIDAR    │    │   Camera    │
                      │  Distance  │    │   Stream    │
                      └─────┬──────┘    └──────┬──────┘
                            │                   │
                            └─────────┬─────────┘
                                      │
                              ┌───────▼────────┐
                              │  Message Node  │
                              │   (Overlay)    │
                              └────────────────┘
```

---

## 📁 File Structure

**Note**: GitHub repository uses branches to organize folders. Each branch represents a directory in the actual file structure.

```
aep_package_vr/
├── package.xml                 # ROS2 package manifest
├── setup.py                    # Python package setup
├── setup.cfg                   # Setup configuration
├── README.md                   # This file
├── DEPLOY.md                   # Deployment notes
├── SETUP.md                    # Setup instructions
│
├── aep_package_vr/            # Main package directory
│   ├── __init__.py
│   ├── gen_steering.py        # UDP steering server (robot)
│   ├── lidar_distance_brake.py # LIDAR processing + AEB
│   ├── message_node.py        # Overlay message aggregator
│   ├── stream.py              # Camera stream with overlays
│   ├── VR_listener.py         # VR headset pan/tilt control
│   └── aep_launcher.py        # System launcher
│
├── launch/                    # Launch files
│   └── aep_vr.launch.py       # Full system launch
│
├── resource/                  # ROS2 resource directory
│   └── aep_package_vr
│
└── gen_client.py              # UDP client (HOST PC ONLY - Windows)
```

### ⚠️ Important Note on gen_client.py

**`gen_client.py` is ONLY intended for use on the host PC running Windows.** This script:
- Reads Logitech G29 steering wheel inputs
- Sends UDP commands to the robot on port 6005
- Requires Windows-specific dependencies (pygame, inputs library)
- Should **NOT** be deployed to the robot

---

## 🔧 Prerequisites

### On Robot (Raspberry Pi)
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
- Logitech G29 steering wheel (optional)
- Libraries: `pygame`, `inputs`, `socket`

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

💡 **Pro Tip**: Add these to your `.bashrc` inside the container to auto-source on login:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
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

# Use SCP to copy the entire package to the Pi
scp -r aep_package_vr pi@192.168.149.1:/home/pi/
```

When prompted, enter password: `raspberrypi`

#### 2. SSH into Pi and Copy to Docker

```bash
# SSH to Pi
ssh pi@192.168.149.1

# Get Docker container ID
docker ps

# Copy package into Docker workspace
docker cp /home/pi/aep_package_vr <containerID>:/home/ubuntu/ros2_ws/src/

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
ros2 launch aep_package_vr aep_vr.launch.py
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

### Running Individual Nodes

You can also run nodes individually for testing:

```bash
# Steering server
ros2 run aep_package_vr gen_steering

# LIDAR distance monitoring
ros2 run aep_package_vr lidar_distance

# Camera stream
ros2 run aep_package_vr stream

# Message overlay aggregator
ros2 run aep_package_vr message_node

# VR listener
ros2 run aep_package_vr vr_listener
```

### Running the Client (Host PC - Windows)

On your **Windows PC**:

```bash
# Navigate to where you have gen_client.py
cd C:\path\to\gen_client.py

# Run the client
python gen_client.py
```

**Client Configuration:**
- Robot IP: `192.168.149.1`
- Port: `6005`
- Steering wheel: Logitech G29 (detected automatically)

---

## 📖 Node Documentation

### 1. gen_steering.py

**Purpose**: UDP server that receives steering commands and controls the robot

**Subscribed Topics:**
- `/lidar/min_distance` (Float32) - Front obstacle distance

**Published Topics:**
- `/controller/cmd_vel` (Twist) - Motor velocity commands
- `/steering/current_gear` (Int32) - Current gear number
- `/steering/emergency_brake_active` (Bool) - Emergency brake status

**Parameters:**
- `max_linear`: 0.7 m/s (max speed)
- `max_angular`: 3.0 rad/s (max turn rate)
- `min_stop_distance_cm`: 15.0 cm (minimum brake distance)

**UDP Message Format (JSON):**
```json
{
  "throttle": 0.5,    // [-1.0, 1.0]
  "brake": 0.0,       // [0.0, 1.0]
  "steering": -0.3,   // [-1.0, 1.0]
  "gear": 4           // integer
}
```

**Key Features:**
- Ackermann steering model (90° max steering angle)
- Dynamic AEB based on speed
- Command timeout safety (120s)
- Network gap tolerance

---

### 2. lidar_distance_brake.py

**Purpose**: Processes LIDAR data and publishes minimum distance in front 90° sector

**Subscribed Topics:**
- `/scan_raw` (LaserScan) - Raw LIDAR data

**Published Topics:**
- `/lidar/min_distance` (Float32) - Minimum distance in cm

**Processing:**
- Filters 90° front sector (±45° from center)
- Validates ranges (0.05m < range < range_max)
- Normalizes angles to [-π, π]
- Converts to centimeters for publishing

---

### 3. message_node.py

**Purpose**: Aggregates system status for camera overlay

**Subscribed Topics:**
- `/lidar/min_distance` (Float32)
- `/steering/current_gear` (Int32)
- `/steering/emergency_brake_active` (Bool)
- `/camera_servo/angles` (Int32MultiArray) - Pan/tilt PWM values

**Provides:**
- `get_messages()` → dict with keys: `warning`, `distance`, `gear`, `pan_tilt`

---

### 4. stream.py

**Purpose**: Camera stream with real-time overlay

**Subscribed Topics:**
- `/ascamera/camera_publisher/rgb0/image` (Image)

**HTTP Endpoints:**
- `http://192.168.149.1:5000/video_feed` - MJPEG stream (~30 FPS)
- `http://192.168.149.1:5000/snapshot` - Single JPEG snapshot

**Overlay Information:**
- Emergency brake warning (center, red)
- Distance to obstacle (bottom center, yellow)
- Current gear (top left, yellow)
- Pan/Tilt servo angles (top right, yellow)

---

### 5. VR_listener.py

**Purpose**: VR headset pan/tilt control for camera servos

**Subscribed Topics:**
- None (UDP input)

**Published Topics:**
- `ros_robot_controller/pwm_servo/set_state` (SetPWMServoState)
- `/camera_servo/angles` (Int32MultiArray) - Raw PWM values

**UDP Configuration:**
- Port: `5005`
- Message format: `P:<pitch>,Y:<yaw>` (ASCII, degrees)

**Servo Mapping:**
- Pitch servo: ID 1 (vertical)
- Yaw servo: ID 2 (horizontal)
- Range: ±90° → PWM 1000-2000 (center: 1500)

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

# Alternative: Use nano (if available)
nano gen_steering.py
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

## 🐛 Troubleshooting

### Common Issues

#### 1. "No executable found" error when launching

**Problem**: Executable names in `setup.py` don't match launch file

**Solution**:
```bash
# Check available executables
ros2 pkg executables aep_package_vr

# Update launch file to use correct names
# OR update setup.py entry_points
```

#### 2. Steering too sensitive / not responsive

**Problem**: Steering angle configuration

**Solution**: Edit `gen_steering.py` line ~201:
```python
# Current: 90° max steering
steering_angle = steering * math.radians(90.0)

# Reduce for less sensitivity (e.g., 30°)
steering_angle = steering * math.radians(30.0)
```

#### 3. Emergency brake activating too early

**Problem**: AEB parameters too conservative

**Solution**: Adjust in `gen_steering.py`:
```python
self.t_reaction = 0.25      # Increase for later braking
self.a_max = 1.0            # Increase for shorter brake distance
self.safety_margin = 0.0    # Reduce for tighter tolerances
self.min_stop_distance_cm = 15.0  # Minimum trigger distance
```

#### 4. Camera stream not accessible

**Problem**: Stream node not running or firewall blocking

**Solution**:
```bash
# Check if stream node is running
ros2 node list | grep stream

# Test locally on robot
curl http://localhost:5000/snapshot

# From host PC, access:
# http://192.168.149.1:5000/video_feed
```

#### 5. UDP commands not reaching robot

**Problem**: Network configuration or firewall

**Solution**:
```bash
# On robot, check if port is listening
sudo lsof -i :6005

# Test UDP connectivity
# On robot:
nc -lu 6005

# On host:
echo "test" | nc -u 192.168.149.1 6005
```

#### 6. Package build fails

**Problem**: Missing dependencies or syntax errors

**Solution**:
```bash
# Check for missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with verbose output to see errors
colcon build --packages-select aep_package_vr --event-handlers console_direct+

# Check Python syntax
python3 -m py_compile src/aep_package_vr/aep_package_vr/gen_steering.py
```

#### 7. "Cannot source setup.bash" after build

**Problem**: Build failed or incomplete

**Solution**:
```bash
# Check build output
cat log/latest_build/aep_package_vr/stdout_stderr.log

# Rebuild from scratch
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select aep_package_vr
```

---

## 📊 System Parameters Reference

### Steering Parameters
| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| max_linear | 0.7 m/s | 0.1-1.5 | Maximum forward speed |
| max_angular | 3.0 rad/s | 1.0-5.0 | Maximum turn rate |
| wheelbase | 0.145 m | - | Robot wheelbase (fixed) |
| steering_angle_max | 90° | 20-90° | Maximum steering angle |

### AEB Parameters
| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| t_reaction | 0.25 s | 0.1-0.5 | Reaction time |
| a_max | 1.0 m/s² | 0.5-2.0 | Max deceleration |
| robot_offset | 0.15 m | 0.1-0.3 | LIDAR to front bumper |
| safety_margin | 0.0 m | 0.0-0.2 | Extra safety distance |
| min_stop_distance_cm | 15 cm | 10-50 | Minimum brake distance |

### LIDAR Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| Front sector | ±45° | 90° total coverage |
| Min range | 5 cm | Minimum valid distance |
| Max range | From LaserScan | Sensor-specific |

### Camera Stream
| Parameter | Value | Description |
|-----------|-------|-------------|
| JPEG quality | 85 | Compression quality (0-100) |
| Frame rate | ~30 FPS | Target streaming rate |
| Port | 5000 | HTTP server port |

---

## 📝 License

[Add your license information here]

## 👥 Contributors

[Add contributor information here]

## 📧 Contact

[Add contact information here]

---

**Last Updated**: December 2024
