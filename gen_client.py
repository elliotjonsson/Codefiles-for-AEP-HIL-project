#!/usr/bin/env python3
import pygame
import socket
import time
import json
from typing import Tuple

# ================== CONFIG ==================
ROBOT_IP = "192.168.149.1"
ROBOT_PORT = 6005
SEND_HZ = 10  # how often to send commands in Hz (reduced to prevent buffer overflow)

# Which joystick to use (0 = first detected)
JOYSTICK_INDEX = 0

# Axis mapping for G27 (Windows has only 2 axes!)
STEERING_AXIS = 0   # steering wheel (-1 = full left, +1 = full right)
PEDAL_AXIS = 1      # combined pedal axis (throttle/brake combined)
# Note: G27 on Windows reports only 2 axes:
#   Axis 0 = Steering
#   Axis 1 = Pedals (throttle positive, brake negative, or combined)

# G27 Button mapping (from g27_client_mac.py)
# Wheel buttons: 0-21 (varies by driver)
# Shifter buttons: varies by driver

# G27 Sequential Gearbox Controls
# Button 4 = Right paddle (shift UP: N→1→2→3→4→5→6)
# Button 5 = Left paddle (shift DOWN: 6→5→4→3→2→1→N→R)
# Sequential order: R (-1) ← N (0) → 1 → 2 → 3 → 4 → 5 → 6
PADDLE_SHIFT_UP = 4
PADDLE_SHIFT_DOWN = 5

# Enable paddle shifter mode
USE_SIMPLE_GEAR = False  # Now using paddle shifters
USE_PADDLE_SHIFTERS = True

# Debug mode: prints all button states to help identify shifter mapping
DEBUG_BUTTONS = False  # Set to True to see all button presses

# Gear speed factors (percentage of max speed)
GEAR_FACTORS = {
    1: 0.16,
    2: 0.32,
    3: 0.48,
    4: 0.64,
    5: 0.80,
    6: 1.00,
}
REVERSE_FACTOR = 0.5  # Reverse = 50% of max

# Deadzones
SPEED_DEADZONE = 0.02
STEERING_DEADZONE = 0.02

# ============================================


def normalize_axis(value: float, invert: bool = False) -> float:
    """Return axis in [-1, 1], optionally inverted."""
    if invert:
        value = -value
    return max(-1.0, min(1.0, value))


def pedal_to_01(raw: float) -> float:
    """
    If pedal axis gives -1..1 where:
      1.0 = released, -1.0 = fully pressed
    map it to 0..1 (0 = released, 1 = fully pressed).
    """
    return (1.0 - raw) / 2.0


class GenericWheelClient:
    def __init__(self):
        self.robot_ip = ROBOT_IP
        self.robot_port = ROBOT_PORT
        self.send_delay = 1.0 / SEND_HZ

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        pygame.init()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        print(f"\n=== Joystick Detection ===")
        print(f"Detected {joystick_count} joystick(s).")
        
        # List all detected devices
        for i in range(joystick_count):
            js = pygame.joystick.Joystick(i)
            js.init()
            print(f"  [{i}] {js.get_name()} - Axes: {js.get_numaxes()}, Buttons: {js.get_numbuttons()}")
            js.quit()

        if joystick_count == 0:
            raise RuntimeError(
                "No joystick / wheel detected.\n"
                "• Make sure the wheel is plugged in and powered on.\n"
                "• Check that Windows sees it in 'Set up USB game controllers'.\n"
                "• Then try again."
            )

        if JOYSTICK_INDEX >= joystick_count:
            raise RuntimeError(
                f"JOYSTICK_INDEX={JOYSTICK_INDEX}, but only {joystick_count} devices found. "
                "Adjust JOYSTICK_INDEX or plug in the correct wheel."
            )

        self.joystick = pygame.joystick.Joystick(JOYSTICK_INDEX)
        self.joystick.init()

        print("=== Generic Wheel Client ===")
        print(f"Using joystick: {self.joystick.get_name()}")
        print(f"Axes: {self.joystick.get_numaxes()}, Buttons: {self.joystick.get_numbuttons()}")
        print(f"Sending to {self.robot_ip}:{self.robot_port} @ {SEND_HZ} Hz")

        self.last_send_time = 0.0
        self.last_nonzero_time = 0.0
        self.current_gear = 0  # Start in NEUTRAL
        self.last_paddle_up_state = False
        self.last_paddle_down_state = False

    def read_pedals(self) -> Tuple[float, float]:
        """
        G27 on Windows: pedals are combined on axis 1.
        Gas pedal (negative) = throttle
        Brake pedal (positive) = brake
        Returns (throttle, brake) both in range 0.0 to 1.0
        """
        if self.joystick.get_numaxes() < 2:
            return 0.0, 0.0
        
        pedal_raw = self.joystick.get_axis(PEDAL_AXIS)
        
        # G27 pedal axis:
        # Negative (-1.0 to 0.0) = Gas pedal pressed
        # Positive (0.0 to +1.0) = Brake pedal pressed
        
        if pedal_raw < 0:
            throttle = -pedal_raw  # Convert to positive 0.0 to 1.0
            brake = 0.0
        elif pedal_raw > 0:
            throttle = 0.0
            brake = pedal_raw  # 0.0 to 1.0
        else:
            throttle = 0.0
            brake = 0.0
        
        return throttle, brake

    def update_gear(self) -> int:
        """Sequential gearbox: R(-1) ← N(0) → 1 → 2 → 3 → 4 → 5 → 6"""
        if USE_SIMPLE_GEAR:
            return 3
        
        if not USE_PADDLE_SHIFTERS:
            return self.current_gear
        
        # Read paddle buttons
        paddle_up = self.joystick.get_button(PADDLE_SHIFT_UP)
        paddle_down = self.joystick.get_button(PADDLE_SHIFT_DOWN)
        
        # Shift UP (right paddle): N→1→2→3→4→5→6
        if paddle_up and not self.last_paddle_up_state:
            if self.current_gear < 6:
                self.current_gear += 1
                gear_name = self._get_gear_name(self.current_gear)
                print(f"\n>> Shifted UP to {gear_name}")
        
        # Shift DOWN (left paddle): 6→5→4→3→2→1→N→R
        if paddle_down and not self.last_paddle_down_state:
            if self.current_gear > -1:
                self.current_gear -= 1
                gear_name = self._get_gear_name(self.current_gear)
                print(f"\n>> Shifted DOWN to {gear_name}")
        
        self.last_paddle_up_state = paddle_up
        self.last_paddle_down_state = paddle_down
        
        return self.current_gear
    
    def _get_gear_name(self, gear: int) -> str:
        """Get display name for gear."""
        if gear == -1:
            return "REVERSE"
        elif gear == 0:
            return "NEUTRAL"
        else:
            return f"Gear {gear}"

    def read_inputs(self) -> Tuple[float, float, int]:
        """
        Read wheel + pedals + gear and return (speed, steering, gear).
        speed > 0 = forward, speed < 0 = reverse.
        """
        pygame.event.pump()

        # Debug: print all button and axis states
        if DEBUG_BUTTONS:
            for i in range(self.joystick.get_numbuttons()):
                if self.joystick.get_button(i):
                    print(f"Button {i} pressed")
            # Show all axes
            print("\rAxes: ", end="")
            for i in range(self.joystick.get_numaxes()):
                print(f"[{i}]={self.joystick.get_axis(i):+.2f} ", end="")
            print("    ", end="")  # Clear rest of line

        # --- Steering ---
        # Inverted for correct direction
        steering = -float(self.joystick.get_axis(STEERING_AXIS))

        # --- Pedals ---
        throttle, brake = self.read_pedals()

        # --- Gear (sequential shifting) ---
        gear = self.update_gear()

        # --- Compute final throttle based on gear ---
        if gear > 0:
            # Forward gears 1-6
            factor = GEAR_FACTORS.get(gear, 1.0)
            throttle = throttle * factor
        elif gear == -1:
            # Reverse gear
            throttle = -throttle * REVERSE_FACTOR
        else:
            # Neutral (gear = 0) - no movement
            throttle = 0.0

        # Brake reduces speed in all gears (except neutral)
        if gear != 0:
            throttle *= (1.0 - brake)

        # Deadzones
        if abs(throttle) < SPEED_DEADZONE:
            throttle = 0.0
        if abs(brake) < SPEED_DEADZONE:
            brake = 0.0
        if abs(steering) < STEERING_DEADZONE:
            steering = 0.0

        return float(throttle), float(brake), float(steering), int(gear)

    def send_packet(self, throttle: float, brake: float, steering: float, gear: int = 0):

        print("Steering: ", steering)
        print("Throttle: ", throttle)

        packet = {
            "throttle": float(throttle),
            "brake": float(brake),
            "steering": float(steering),
            "gear": int(gear),
        }
        data = json.dumps(packet).encode("utf-8")
        self.sock.sendto(data, (self.robot_ip, self.robot_port))

    def failsafe_check(self):
        """
        Simple failsafe: if we haven't sent anything for 0.5 s, send a stop command.
        """
        now = time.time()
        if now - self.last_send_time > 0.5:
            self.send_packet(0.0, 0.0, 0.0, 0)

    def run(self):
        print("\n=== Logitech G27 Controller Active ===")
        print(f"Robot: {self.robot_ip}:{self.robot_port}")
        print("\nControls:")
        print("  Steering     = Wheel (AXIS 0)")
        print("  Gas Pedal    = AXIS 1 negative (throttle)")
        print("  Brake Pedal  = AXIS 1 positive (brake)")
        
        if USE_SIMPLE_GEAR:
            print("\nGears: SIMPLE MODE (fixed gear 3 = 48% max speed)")
        elif USE_PADDLE_SHIFTERS:
            print("\nGears: SEQUENTIAL GEARBOX (starts in NEUTRAL)")
            print("  Right Paddle (Button 4) = SHIFT UP")
            print("    N → 1 → 2 → 3 → 4 → 5 → 6")
            print("  Left Paddle  (Button 5) = SHIFT DOWN")
            print("    6 → 5 → 4 → 3 → 2 → 1 → N → R")
            print(f"  Current: {self._get_gear_name(self.current_gear)}")
        else:
            print("\nGears: H-pattern shifter enabled")
        
        print("\nPress Ctrl+C to stop\n")
        
        try:
            while True:
                throttle, brake, steering, gear = self.read_inputs()
                steering = steering
                steering = max(-0.9, min(0.9, steering))

                # Send packet with error handling
                try:
                    self.send_packet(throttle, brake, steering, gear)
                    self.last_send_time = time.time()
                except OSError as e:
                    # Socket buffer full - wait a bit
                    print(f"\nSocket error: {e} - reducing send rate")
                    time.sleep(0.1)
                    continue
                
                if abs(throttle) > SPEED_DEADZONE or abs(steering) > STEERING_DEADZONE:
                    self.last_nonzero_time = self.last_send_time

                #self.failsafe_check()
                time.sleep(self.send_delay)
        except KeyboardInterrupt:
            print("\nExiting client, sending stop...")
            self.send_packet(0.0, 0.0, 0.0, 0)


if __name__ == "__main__":
    client = GenericWheelClient()
    client.run()
