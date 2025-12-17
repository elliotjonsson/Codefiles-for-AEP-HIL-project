#!/usr/bin/env python3
"""
MessageNode: collects robot state and builds overlay texts for the video stream.

Subscribes to:
- /camera_servo/angles (Int32MultiArray)       -> Pan/Tilt servo PWM (1000–2000)
- /lidar/min_distance (Float32)                -> Min distance in cm
- /steering/current_gear (Int32)               -> Gear from gen_steering
- /steering/emergency_brake_active (Bool)      -> AEB flag from gen_steering

Provides:
- get_latest() -> dict:
    {
      "warning":   "Emergency Brake Activated, Please Reverse" or None,
      "pan_tilt":  "Pan/Tilt: +10 / -5" or None,
      "distance":  "123 cm" or None,
      "gear":      "Gear: 4" / "Gear: R" / "Gear: N" or None
    }
"""

from typing import Optional, Dict
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32, Int32, Bool


class MessageNode(Node):
    def __init__(self):
        super().__init__("message_node")

        # State
        self.pan: Optional[int] = None          # PWM 1000–2000
        self.tilt: Optional[int] = None         # PWM 1000–2000
        self.distance_cm: Optional[float] = None
        self.gear: Optional[int] = None         # -1 = R, 0 = N, >0 = forward gears
        self.emergency_brake: bool = False

        # Latest overlay texts
        self._latest: Dict[str, Optional[str]] = {}
        self._lock = threading.Lock()

        # Subscriptions
        self.create_subscription(
            Int32MultiArray,
            "/camera_servo/angles",
            self.angles_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/lidar/min_distance",
            self.distance_callback,
            10,
        )
        self.create_subscription(
            Int32,
            "/steering/current_gear",
            self.gear_callback,
            10,
        )
        self.create_subscription(
            Bool,
            "/steering/emergency_brake_active",
            self.brake_callback,
            10,
        )

        self.get_logger().info("MessageNode started - building overlay texts")

    # ---------- Callbacks ----------

    def angles_callback(self, msg: Int32MultiArray):
        try:
            if len(msg.data) >= 2:
                self.pan = int(msg.data[0])
                self.tilt = int(msg.data[1])
            else:
                self.pan = None
                self.tilt = None
        except Exception as e:
            self.get_logger().warn(f"Error parsing angles: {e}")
            self.pan = None
            self.tilt = None
        self._update_texts()

    def distance_callback(self, msg: Float32):
        try:
            self.distance_cm = float(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Error parsing distance: {e}")
            self.distance_cm = None
        self._update_texts()

    def gear_callback(self, msg: Int32):
        try:
            self.gear = int(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Error parsing gear: {e}")
            self.gear = None
        self._update_texts()

    def brake_callback(self, msg: Bool):
        try:
            self.emergency_brake = bool(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Error parsing brake flag: {e}")
            self.emergency_brake = False
        self._update_texts()

    # ---------- Text building ----------

    def _update_texts(self):
        texts = self.build_texts(
            distance_cm=self.distance_cm,
            pan=self.pan,
            tilt=self.tilt,
            gear=self.gear,
            emergency_brake=self.emergency_brake,
        )
        with self._lock:
            self._latest = texts

    def build_texts(
        self,
        distance_cm: Optional[float],
        pan: Optional[int],
        tilt: Optional[int],
        gear: Optional[int],
        emergency_brake: bool,
    ) -> Dict[str, Optional[str]]:

        # 1) Emergency warning
        if emergency_brake:
            warning = "Emergency Brake Activated, Please Reverse"
        else:
            warning = None

        # 2) Pan/Tilt string
        pan_tilt = None
        if pan is not None and tilt is not None:
            # Map PWM 1000–2000 -> approx -90..+90 deg
            pan_angle = int((pan - 1500) / 500.0 * 90.0)
            tilt_angle = int((tilt - 1500) / 500.0 * 90.0)
            pan_tilt = f"Pan/Tilt: {pan_angle:+d} / {tilt_angle:+d}"

        # 3) Distance string
        distance_text = None
        if distance_cm is not None:
            try:
                distance_text = f"{distance_cm:.0f} cm"
            except Exception:
                distance_text = f"{distance_cm} cm"

        # 4) Gear string
        gear_text = None
        if gear is not None:
            if gear == -1:
                gear_text = "Gear: R"
            elif gear == 0:
                gear_text = "Gear: N"
            else:
                gear_text = f"Gear: {gear}"

        return {
            "warning": warning,
            "pan_tilt": pan_tilt,
            "distance": distance_text,
            "gear": gear_text,
        }

    # ---------- API for stream node ----------

    def get_latest(self) -> Dict[str, Optional[str]]:
        with self._lock:
            return dict(self._latest)


def main():
    rclpy.init()
    node = MessageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
