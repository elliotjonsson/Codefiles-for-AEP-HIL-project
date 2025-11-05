# MentorPi ROS 2 Nodes — Camera, Joystick & LiDAR

This repository contains a small set of ROS 2 nodes for a Raspberry Pi–based robot (MentorPi). The nodes let you:

- Drive the robot with a gamepad and send velocity commands (`joystick_control_1010.py`).
- Pan/tilt a camera gimbal via servos from a joystick (`CameraControl.1010.py`) or keyboard (`controller1005.py`).
- Stream the camera feed as MJPEG over HTTP (via `controller1005.py`).
- Monitor minimum distance from a LiDAR and publish it for other nodes (`lidar_distance_node.py`).
---

## Repository layout

```
.
├── CameraControl.1010.py            # Joystick → camera servos (pan/tilt) + angle publisher
├── controller1005.py                # Camera HTTP stream + keyboard (W/A/S/D) servo control
├── control.py                       # (alt) Camera servo control from joystick
├── joystick_control_1010.py         # Gamepad → geometry_msgs/Twist (driving) + buzzer + gimbal
├── lidar_distance_node.py           # LiDAR /scan_raw → /lidar/min_distance (Float32)
└── robot-basic-code/                # Simple non‑ROS camera/motion examples
    ├── Move-robot.py
    ├── image-process.py
    └── camera-servo.py
```

> Filenames in this README are shown without the “(1)” suffix for clarity. Keep your actual names or rename the files to match the above for consistency.

---

## Requirements

- **ROS 2 Humble** (Ubuntu 22.04 recommended) with Python 3
- Packages:
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `std_srvs`
  - `ros_robot_controller_msgs` (for `BuzzerState`, `SetPWMServoState`, `PWMServoState`)
  - `joy` / `joy_linux` (for `/joy` gamepad messages)
  - A LiDAR publishing `sensor_msgs/LaserScan` on `/scan_raw`
- Servo controller compatible with `ros_robot_controller/pwm_servo/set_state`

> If `ros_robot_controller_msgs` is not installed from your distro, add its source to your workspace and build with `colcon`.

---

## Quick start (run as standalone scripts)

You can run these nodes without turning them into a full ROS 2 package.

1. Source your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. In one terminal, start your joystick driver (example):
   ```bash
   ros2 run joy joy_node
   ```

3. Run any of the nodes (replace with the path to your file):
   ```bash
   # Driving + buzzer + gimbal from gamepad
   python3 joystick_control_1010\ \(1\).py

   # Camera gimbal from joystick
   python3 CameraControl.1010\ \(1\).py

   # Camera gimbal from SSH keyboard + HTTP video stream
   python3 controller1005.py

   # LiDAR min distance publisher
   python3 lidar_distance_node\ \(1\).py
   ```

> Tip: Add executable bit and a shebang line (`#!/usr/bin/env python3`) to call them directly.

---

## (Recommended) Use as a ROS 2 package

Create a new Python package (example name: `mentorpi_nodes`) and move these scripts into `mentorpi_nodes/`.

```bash
ros2 pkg create mentorpi_nodes --build-type ament_python --dependencies rclpy sensor_msgs geometry_msgs std_msgs std_srvs
```

- Put the `.py` files under `mentorpi_nodes/`
- In `setup.cfg`, mark them as console scripts (entry points)
- In `package.xml`, add `ros_robot_controller_msgs` and other missing deps
- Build and run:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 run mentorpi_nodes joystick_control
```

<details>
<summary>Example <code>setup.py</code> console scripts</summary>

```python
entry_points={
    'console_scripts': [
        'joystick_control = mentorpi_nodes.joystick_control_1010:main',
        'camera_control = mentorpi_nodes.CameraControl_1010:main',
        'camera_http = mentorpi_nodes.controller1005:main',
        'lidar_distance = mentorpi_nodes.lidar_distance_node:main',
    ],
}
```
</details>

---

## Node summaries & topics

### 1) `joystick_control_1010.py` — Drive + buzzer + gimbal
- **Subscribes:** `/joy` (`sensor_msgs/Joy`)
- **Publishes:**
  - `controller/cmd_vel` (`geometry_msgs/Twist`) — robot linear/angular velocity
  - `ros_robot_controller/set_buzzer` (`ros_robot_controller_msgs/BuzzerState`)
  - `ros_robot_controller/pwm_servo/set_state` (`ros_robot_controller_msgs/SetPWMServoState`) — camera gimbal
- **Default mapping (from code comments):**
  - Steering moved to **left stick X** (`lx`) → `Twist.angular.z`
  - Throttle on **left stick Y** (`ly`) → `Twist.linear.x`
  - **Right stick** is free for gimbal control (pan/tilt)
- **Notes:** Dead‑zone handling and value mapping (`sdk.common.val_map`) are applied.

### 2) `CameraControl.1010.py` / `control.py` — Joystick → camera servos
- **Subscribes:** `/joy` (`sensor_msgs/Joy`)
- **Publishes:**
  - `ros_robot_controller/pwm_servo/set_state` (`SetPWMServoState`)
  - `/camera_servo/angles` (`std_msgs/Int32MultiArray`) — `[pan, tilt]` for debugging/visualization
- **Behavior:** map joystick axes to servo IDs with min/max bounds and rate limiting.

### 3) `controller1005.py` — HTTP stream + keyboard servo control
- **Subscribes:** `/ascamera/camera_publisher/rgb0/image` (`sensor_msgs/Image`)
- **Publishes:** `ros_robot_controller/pwm_servo/set_state` for the camera gimbal
- **Features:**
  - **HTTP MJPEG stream at** `http://<robot-ip>:5000/video_feed`
  - **SSH keyboard control**: W/A/S/D move the gimbal; press `Ctrl+C` to center & exit
  - Thread‑safe image buffering and log‑quiet HTTP handler

### 4) `lidar_distance_node.py` — Min range publisher
- **Subscribes:** `/scan_raw` (`sensor_msgs/LaserScan`)
- **Publishes:** `/lidar/min_distance` (`std_msgs/Float32`) — meters
- **Behavior:** Computes min valid range in FOV, logs warnings if too close (and can be used to trigger stop/warning in other nodes).

---

## Typical runtime graph

```
/joy → [joystick_control] → /controller/cmd_vel
                       └→ ros_robot_controller/set_buzzer
                       └→ ros_robot_controller/pwm_servo/set_state (gimbal)

/ascamera/camera_publisher/rgb0/image → [controller1005] → HTTP :5000/video_feed
/scan_raw → [lidar_distance_node] → /lidar/min_distance
```

---

## Parameters & customization

- **Dead‑zones / scaling** (in `joystick_control_1010.py`): tune `min_value`, `max_linear`, `max_angular`.
- **Servo limits & IDs** (in camera control scripts): adjust `SERVO_CENTER`, min/max bounds, and `PWMServoState.id` for your controller.
- **HTTP port** (`controller1005.py`): default `5000` (change `HTTP_PORT`).

---

## Troubleshooting

- **No `/joy` messages**: verify `ros2 topic echo /joy` and correct joystick driver (`joy`/`joy_linux`). Some controllers need `SDL_GAMECONTROLLERCONFIG` or `udev` rules.
- **No video stream**: confirm camera node publishes to `/ascamera/camera_publisher/rgb0/image`. Check firewall if the `/video_feed` doesn’t open.
- **Servos don’t move**: make sure `ros_robot_controller_msgs` types match your servo board and that the node publishes to the correct topic.
- **LiDAR “No measurement.”**: ensure `/scan_raw` exists and `range_min < range < range_max` are being published.

---

## Development tips

- Use `--symlink-install` in `colcon build` for instant code reloads.
- Inspect traffic with:
  ```bash
  ros2 topic list
  ros2 topic echo /controller/cmd_vel
  ros2 topic hz /lidar/min_distance
  ```
- Record a quick bag while testing:
  ```bash
  ros2 bag record /joy /controller/cmd_vel /lidar/min_distance
  ```

---

---

## Credits

MentorPi AEP project — camera/servo/joystick/LiDAR nodes by Chalmers University of Technology TME170 course.
