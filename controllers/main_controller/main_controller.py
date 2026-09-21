"""
main_controller.py

Robotic Competition - Section 1 Autonomous Navigation & Collision Prevention
Deterministic Safety State Machine with Encoder Closed-Loop Control

Features:
  - Moving-average / median filter on front, left, and right distance sensors
  - Hard collision prevention with multi-tier distance safety thresholds
  - Side-clearance-based turn selection
  - Encoder-controlled closed-loop turning (90° / 180°)
  - Straight-line heading stabilization via wheel encoder feedback
  - Hysteresis to prevent rapid state oscillation
"""

from controller import Robot
import math
from collections import deque

# --- Simulation & Kinematics Parameters ---
TIME_STEP = 32
MAX_SPEED = 6.0

WHEEL_RADIUS = 0.02      # 2 cm
WHEEL_BASE = 0.064       # 6.4 cm distance between wheels (2 * 0.032m)
RAD_PER_ROBOT_RAD = WHEEL_BASE / (2.0 * WHEEL_RADIUS)  # 1.6 wheel rad per robot rad

# --- Speed Constants (rad/s) ---
CRUISE_SPEED = 5.0
APPROACH_SPEED = 2.5
TURN_SPEED = 1
MIN_TURN_SPEED = 0.8

# --- Distance Safety Thresholds (meters) ---
EMERGENCY_DISTANCE = 0.05      # Immediate stop
STOP_DISTANCE = 0.05           # Full stop before obstacle
SLOW_DISTANCE = 0.08           # Deceleration threshold
EXIT_OBSTACLE_DISTANCE = 0.10  # Hysteresis threshold to return to cruise
SIDE_MIN_CLEARANCE = 0.05      # Minimum side clearance required to turn toward a side

# --- Sensor Filtering Class ---
class SensorFilter:
    def __init__(self, window_size=5, default_val=2.0):
        self.window_size = window_size
        self.default_val = default_val
        self.buffer = deque(maxlen=window_size)

    def update(self, val):
        if val is None or math.isnan(val) or val < 0:
            val = self.default_val
        self.buffer.append(val)
        sorted_vals = sorted(self.buffer)
        return sorted_vals[len(sorted_vals) // 2]  # Median filter

    @property
    def value(self):
        if not self.buffer:
            return self.default_val
        sorted_vals = sorted(self.buffer)
        return sorted_vals[len(sorted_vals) // 2]


# --- Main Controller Initialization ---
robot = Robot()

# Motors setup
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# LEDs setup
led_blue = robot.getDevice('led_blue')
led_green = robot.getDevice('led_green')
if led_blue:
    led_blue.set(1)
if led_green:
    led_green.set(1)

# Sensors setup
front_ds = robot.getDevice('front_distance_sensor')
left_ds = robot.getDevice('left_distance_sensor')
right_ds = robot.getDevice('right_distance_sensor')
camera = robot.getDevice('endpoint_camera')

left_encoder = robot.getDevice('left_wheel_sensor')
right_encoder = robot.getDevice('right_wheel_sensor')

if front_ds: front_ds.enable(TIME_STEP)
if left_ds: left_ds.enable(TIME_STEP)
if right_ds: right_ds.enable(TIME_STEP)
if camera: camera.enable(TIME_STEP)
if left_encoder: left_encoder.enable(TIME_STEP)
if right_encoder: right_encoder.enable(TIME_STEP)

# Initial sensor filter instances
filter_front = SensorFilter(window_size=5, default_val=2.0)
filter_left = SensorFilter(window_size=5, default_val=2.0)
filter_right = SensorFilter(window_size=5, default_val=2.0)

# Controller State Variables
state = "FORWARD"
prev_state = None
settle_counter = 0

# Encoder tracking variables for turns and heading
turn_target_angle = 0.0
turn_start_l_enc = 0.0
turn_start_r_enc = 0.0

heading_ref_l = 0.0
heading_ref_r = 0.0

step_counter = 0

print("[NAVIGATION CONTROLLER] Safety State Machine Initialized.")

# Allow initial sensor step to populate readings
robot.step(TIME_STEP)
filter_front.update(front_ds.getValue() if front_ds else 2.0)
filter_left.update(left_ds.getValue() if left_ds else 2.0)
filter_right.update(right_ds.getValue() if right_ds else 2.0)

if left_encoder: heading_ref_l = left_encoder.getValue()
if right_encoder: heading_ref_r = right_encoder.getValue()


def get_encoder_values():
    l = left_encoder.getValue() if left_encoder else 0.0
    r = right_encoder.getValue() if right_encoder else 0.0
    if math.isnan(l): l = 0.0
    if math.isnan(r): r = 0.0
    return l, r


while robot.step(TIME_STEP) != -1:
    step_counter += 1

    # Update sensor filters
    f_raw = front_ds.getValue() if front_ds else 2.0
    l_raw = left_ds.getValue() if left_ds else 2.0
    r_raw = right_ds.getValue() if right_ds else 2.0

    f_dist = filter_front.update(f_raw)
    l_dist = filter_left.update(l_raw)
    r_dist = filter_right.update(r_raw)

    l_enc, r_enc = get_encoder_values()

    # --- HARD EMERGENCY OVERRIDE ---
    # If front sensor detects an emergency obstacle (< EMERGENCY_DISTANCE), force stop immediately
    if f_dist <= EMERGENCY_DISTANCE and state not in ["STOP_FOR_OBSTACLE", "CHOOSE_TURN", "TURNING", "VERIFY_CLEAR"]:
        state = "STOP_FOR_OBSTACLE"
        settle_counter = 0

    # --- SAFETY STATE MACHINE LOGIC ---
    if state == "FORWARD":
        # Straight driving with encoder-based heading correction
        l_delta = l_enc - heading_ref_l
        r_delta = r_enc - heading_ref_r
        heading_err = l_delta - r_delta
        
        # P-controller to keep straight heading
        kp = 0.8
        l_speed = CRUISE_SPEED - kp * heading_err
        r_speed = CRUISE_SPEED + kp * heading_err
        
        l_speed = max(-MAX_SPEED, min(MAX_SPEED, l_speed))
        r_speed = max(-MAX_SPEED, min(MAX_SPEED, r_speed))
        
        left_motor.setVelocity(l_speed)
        right_motor.setVelocity(r_speed)

        # Transition checks
        if f_dist <= STOP_DISTANCE:
            state = "STOP_FOR_OBSTACLE"
            settle_counter = 0
        elif f_dist <= SLOW_DISTANCE:
            state = "APPROACH_OBSTACLE"

    elif state == "APPROACH_OBSTACLE":
        # Slow down on approach
        l_delta = l_enc - heading_ref_l
        r_delta = r_enc - heading_ref_r
        heading_err = l_delta - r_delta
        
        kp = 0.8
        l_speed = APPROACH_SPEED - kp * heading_err
        r_speed = APPROACH_SPEED + kp * heading_err
        
        left_motor.setVelocity(l_speed)
        right_motor.setVelocity(r_speed)

        if f_dist <= STOP_DISTANCE:
            state = "STOP_FOR_OBSTACLE"
            settle_counter = 0
        elif f_dist > EXIT_OBSTACLE_DISTANCE:
            state = "FORWARD"

    elif state == "STOP_FOR_OBSTACLE":
        # Complete stop and wait for settling
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        settle_counter += 1

        if settle_counter >= 3:
            state = "CHOOSE_TURN"

    elif state == "CHOOSE_TURN":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)

        # Evaluate side sensors for best turn direction
        if l_dist > r_dist and l_dist >= SIDE_MIN_CLEARANCE:
            turn_target_angle = math.pi / 2.0   # +90 deg (Turn Left)
        elif r_dist > l_dist and r_dist >= SIDE_MIN_CLEARANCE:
            turn_target_angle = -math.pi / 2.0  # -90 deg (Turn Right)
        elif l_dist >= SIDE_MIN_CLEARANCE:
            turn_target_angle = math.pi / 2.0   # Turn Left
        elif r_dist >= SIDE_MIN_CLEARANCE:
            turn_target_angle = -math.pi / 2.0  # Turn Right
        else:
            # Both sides tight: execute 180° turn or turn toward max space
            if l_dist >= r_dist:
                turn_target_angle = math.pi      # +180 deg
            else:
                turn_target_angle = -math.pi     # -180 deg

        turn_start_l_enc = l_enc
        turn_start_r_enc = r_enc
        state = "TURNING"
        settle_counter = 0

    elif state == "TURNING":
        # Closed-loop turn using wheel encoders
        d_l = l_enc - turn_start_l_enc
        d_r = r_enc - turn_start_r_enc
        
        # Robot angular rotation accomplished (CCW positive)
        actual_robot_turn = (d_r - d_l) / (2.0 * RAD_PER_ROBOT_RAD)
        remaining_angle = abs(turn_target_angle) - abs(actual_robot_turn)

        if remaining_angle > 0.03:  # 0.03 rad threshold (~1.7 deg)
            # Speed scaling for smooth deceleration near end of turn
            speed_factor = min(1.0, max(MIN_TURN_SPEED / TURN_SPEED, remaining_angle / 0.4))
            curr_turn_speed = TURN_SPEED * speed_factor

            if turn_target_angle > 0:
                # Turn Left (CCW): left wheel back, right wheel forward
                left_motor.setVelocity(-curr_turn_speed)
                right_motor.setVelocity(curr_turn_speed)
            else:
                # Turn Right (CW): left wheel forward, right wheel back
                left_motor.setVelocity(curr_turn_speed)
                right_motor.setVelocity(-curr_turn_speed)
        else:
            # Reached target turn angle
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            settle_counter += 1
            if settle_counter >= 3:
                state = "VERIFY_CLEAR"

    elif state == "VERIFY_CLEAR":
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        settle_counter += 1

        if settle_counter >= 2:
            if f_dist > STOP_DISTANCE + 0.05:
                state = "RESUME_FORWARD"
            else:
                # Still blocked, turn again
                state = "CHOOSE_TURN"

    elif state == "RESUME_FORWARD":
        # Reset heading reference for new straight segment
        heading_ref_l, heading_ref_r = get_encoder_values()
        state = "FORWARD"

    # --- Debug Logging ---
    if state != prev_state or step_counter % 15 == 0:
        print(f"F: {f_dist:.2f}m | L: {l_dist:.2f}m | R: {r_dist:.2f}m | STATE: {state}")
        prev_state = state
