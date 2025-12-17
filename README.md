# AEP Package VR

ROS2 package for VR-controlled robot with camera streaming, LIDAR-based emergency braking, and autonomous steering control.

## Package Contents

This package includes 5 nodes that work together:

1. **lidar_distance** - Processes LIDAR scan data and publishes minimum distance in front 90° sector
2. **vr_listener** - Receives VR headset orientation via UDP and controls camera servos (pan/tilt)
3. **gen_steering** - Generates steering commands with emergency brake logic based on LIDAR data
4. **message_node** - Collects robot state and builds overlay texts (embedded in stream node)
5. **stream** - Camera stream with HTTP MJPEG server and real-time overlays

## Installation

1. Navigate to your ROS2 workspace source directory:
```bash
cd ~/ros2_ws/src
```

2. The package is already created at: `c:\Users\testuser\Desktop\Robot\code\src\aep_package_vr`

3. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select aep_package_vr
source install/setup.bash
```

## Usage

### Option 1: Using the Launcher (Recommended)
Launch all nodes with a single command:
```bash
ros2 run aep_package_vr aep_launch
```

### Option 2: Using Launch File
```bash
ros2 launch aep_package_vr aep_vr.launch.py
```

### Option 3: Manual Node Launch (for debugging)
Launch nodes individually in separate terminals:

```bash
# Terminal 1 - LIDAR distance
ros2 run aep_package_vr lidar_distance

# Terminal 2 - VR listener
ros2 run aep_package_vr vr_listener

# Terminal 3 - Steering generator
ros2 run aep_package_vr gen_steering

# Terminal 4 - Camera stream
ros2 run aep_package_vr stream
```

## Interfaces

### Published Topics
- `/lidar/min_distance` (Float32) - Minimum obstacle distance in cm
- `/camera_servo/angles` (Int32MultiArray) - Camera servo PWM values [pan, tilt]
- `/cmd_vel` (Twist) - Robot velocity commands
- `/steering/current_gear` (Int32) - Current gear (-1=Reverse, 0=Neutral, 1-5=Forward)
- `/steering/emergency_brake_active` (Bool) - Emergency brake status
- `ros_robot_controller/pwm_servo/set_state` (SetPWMServoState) - Camera servo control

### Subscribed Topics
- `/scan_raw` (LaserScan) - Raw LIDAR scan data
- `/ascamera/camera_publisher/rgb0/image` (Image) - Camera frames
- TCP port 6005 - Steering control input
- UDP port 5005 - VR headset orientation input

### HTTP Services
- `http://localhost:8080` - MJPEG camera stream with overlays

## Node Details

### lidar_distance_brake
Processes LIDAR data and publishes minimum distance in front 90° sector for emergency brake logic.

### VR_listener
Receives VR headset orientation (pitch/yaw) via UDP and controls camera servos. Publishes servo angles for overlay display.

### gen_steering
Receives steering commands via TCP socket and publishes velocity commands. Implements emergency brake when obstacles are too close based on current speed.

### stream
Provides HTTP MJPEG camera stream with real-time overlays:
- Emergency brake warning (red, center)
- Distance to obstacle (yellow, bottom)
- Current gear (yellow, top-left)
- Camera pan/tilt angles (yellow, top-right)

## Configuration

Modify parameters in `launch/aep_vr.launch.py`:
- `max_linear`: Maximum linear velocity (default: 0.7 m/s)
- `max_angular`: Maximum angular velocity (default: 3.0 rad/s)
- `min_stop_distance_cm`: Minimum stop distance (default: 15 cm)

## Dependencies
- rclpy
- geometry_msgs
- std_msgs
- sensor_msgs
- ros_robot_controller_msgs
- cv_bridge
- opencv-python
- numpy

## Troubleshooting

**Issue**: Nodes don't start
- Ensure all dependencies are installed
- Check that ros_robot_controller_msgs package is built
- Verify camera and LIDAR topics are publishing

**Issue**: Camera stream not available
- Check http://localhost:8080 in browser
- Verify camera topic `/ascamera/camera_publisher/rgb0/image` is active

**Issue**: Emergency brake always active
- Check LIDAR is publishing on `/scan_raw`
- Verify lidar_distance node is running
- Check obstacle clearance in front of robot

## License
TODO: License declaration
