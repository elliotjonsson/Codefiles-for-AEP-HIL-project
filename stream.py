#!/usr/bin/env python3
"""
Camera Stream Node with HTTP MJPEG server at ~30 FPS.

Reads camera frames from:
    /ascamera/camera_publisher/rgb0/image

Fetches overlay texts from MessageNode (message_node.py):
    "warning"   -> center, red:  "Emergency Brake Activated, Please Reverse"
    "distance"  -> bottom center, yellow: "123 cm"
    "gear"      -> top-left, yellow: "Gear: 4"
    "pan_tilt"  -> top-right, yellow: "Pan/Tilt: +10 / -5"

IMPORTANT:
- Emergency brake here = ONLY a text overlay. No logic in this file
  stops the camera or the steering.
- We use .copy() on the frame to avoid "read-only buffer" issues when drawing text.
"""

import sys
import os
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer

# Allow importing message_node from the same folder
sys.path.insert(0, os.path.dirname(__file__))
from message_node import MessageNode


latest_frame = None
frame_lock = threading.Lock()
message_node_instance: MessageNode = None


class CameraStreamNode(Node):
    def __init__(self):
        super().__init__("camera_stream_node")

        self.bridge = CvBridge()
        self.frame_counter = 0

        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            "/ascamera/camera_publisher/rgb0/image",
            self.image_callback,
            10,
        )

        self.get_logger().info(
            "CameraStreamNode started (HTTP /video_feed on port 5000)."
        )

    def image_callback(self, msg: Image):
        """Receive camera frame, draw overlay texts, store JPEG in global latest_frame."""
        global latest_frame, message_node_instance

        try:
            # Convert ROS image -> OpenCV
            orig_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # IMPORTANT: take a COPY so the buffer is writable
            cv_image = orig_image.copy()
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        h, w = cv_image.shape[:2]

        # Get overlay texts from MessageNode
        texts = {}
        if message_node_instance is not None:
            try:
                texts = message_node_instance.get_latest()
            except Exception as e:
                self.get_logger().warn(f"Failed to get overlay texts: {e}")
                texts = {}

        # Draw overlays – protected with try so they never kill the stream
        try:
            # 1) Warning (center, RED, with black outline)
            warning = texts.get("warning")
            if warning:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8  # slightly smaller so it fits
                thickness = 2
                (tw, th), _ = cv2.getTextSize(warning, font, font_scale, thickness)

                x = (w - tw) // 2
                if x < 0:
                    x = 10  # safety if very long text

                # vertical center
                y = h // 2

                # Black outline for readability
                cv2.putText(
                    cv_image,
                    warning,
                    (x, y),
                    font,
                    font_scale,
                    (0, 0, 0),
                    thickness + 2,
                    cv2.LINE_AA,
                )
                # Red text on top
                cv2.putText(
                    cv_image,
                    warning,
                    (x, y),
                    font,
                    font_scale,
                    (0, 0, 255),
                    thickness,
                    cv2.LINE_AA,
                )

            # 2) Distance (bottom center, YELLOW)
            distance_text = texts.get("distance")
            if distance_text:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1.0
                thickness = 2
                (tw, th), _ = cv2.getTextSize(distance_text, font, font_scale, thickness)
                x = (w - tw) // 2
                y = h - 20
                cv2.putText(
                    cv_image,
                    distance_text,
                    (x, y),
                    font,
                    font_scale,
                    (0, 255, 255),  # YELLOW
                    thickness,
                    cv2.LINE_AA,
                )

            # 3) Gear (top-left, YELLOW)
            gear_text = texts.get("gear")
            if gear_text:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.9
                thickness = 2
                x = 20
                y = 40
                cv2.putText(
                    cv_image,
                    gear_text,
                    (x, y),
                    font,
                    font_scale,
                    (0, 255, 255),  # YELLOW
                    thickness,
                    cv2.LINE_AA,
                )

            # 4) Pan/Tilt (top-right, YELLOW)
            pan_tilt_text = texts.get("pan_tilt")
            if pan_tilt_text:
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.8
                thickness = 2
                (tw, th), _ = cv2.getTextSize(pan_tilt_text, font, font_scale, thickness)
                x = w - tw - 20
                y = 40
                cv2.putText(
                    cv_image,
                    pan_tilt_text,
                    (x, y),
                    font,
                    font_scale,
                    (0, 255, 255),  # YELLOW
                    thickness,
                    cv2.LINE_AA,
                )

        except Exception as e:
            # If something goes wrong here, DO NOT kill the stream
            self.get_logger().warn(
                f"Overlay drawing error: {e}", throttle_duration_sec=2.0
            )

        # Encode as JPEG and store in global latest_frame
        try:
            ok, buffer = cv2.imencode(".jpg", cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if ok:
                with frame_lock:
                    latest_frame = buffer.tobytes()
        except Exception as e:
            self.get_logger().error(f"JPEG encoding error: {e}")

        self.frame_counter += 1
        if self.frame_counter % 30 == 0:
            self.get_logger().info(
                f"Streaming frame {self.frame_counter} (~30 FPS)",
                throttle_duration_sec=1.0,
            )


# ---------- HTTP MJPEG & Snapshot server ----------

class MJPEGStreamHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silence default logging
        pass

    def do_GET(self):
        if self.path == "/video_feed":
            # MJPEG stream endpoint
            self.send_response(200)
            self.send_header("Content-type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            try:
                while True:
                    with frame_lock:
                        frame = latest_frame

                    if frame is None:
                        time.sleep(0.05)
                        continue

                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")

                    # ~30 FPS
                    time.sleep(0.033)

            except Exception:
                # Client disconnected etc. – not fatal
                pass

        elif self.path == "/snapshot":
            # Snapshot endpoint: returns a single JPEG (used by Unity VR polling)
            with frame_lock:
                frame_data = latest_frame

            if frame_data is None:
                self.send_response(503)
                self.send_header("Content-type", "text/plain")
                self.end_headers()
                self.wfile.write(b"No frame available")
                return

            self.send_response(200)
            self.send_header("Content-type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame_data)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            self.wfile.write(frame_data)

        else:
            self.send_response(404)
            self.end_headers()


def start_http_server():
    try:
        server = HTTPServer(("0.0.0.0", 5000), MJPEGStreamHandler)
        print("HTTP server started on port 5000 (/video_feed)")
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        return server
    except Exception as e:
        print(f"Failed to start HTTP server: {e}")
        return None


def main(args=None):
    global message_node_instance

    rclpy.init(args=args)

    # Start message node (builds overlay texts)
    message_node_instance = MessageNode()
    # Start camera node (reads images and draws overlays)
    camera_node = CameraStreamNode()

    # HTTP server runs in its own thread
    start_http_server()

    # ROS2 MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(message_node_instance)
    executor.add_node(camera_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        camera_node.destroy_node()
        message_node_instance.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
