"""
main_controller.py

Controller for TwoWheelRobot with sensor initialization and validation logging.
Enables and reads:
  - front_distance_sensor
  - left_distance_sensor
  - right_distance_sensor
  - endpoint_camera
  - left_wheel_sensor
  - right_wheel_sensor

Logs sensor readings periodically to the Webots console to allow calibration
and physical validation before implementing navigation algorithms.
"""

from controller import Robot
import random

TIME_STEP = 64
MAX_SPEED = 6.0

robot = Robot()

# --- Motors setup ---
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# --- LEDs setup ---
led_blue = robot.getDevice('led_blue')
led_green = robot.getDevice('led_green')
if led_blue:
    led_blue.set(1)
if led_green:
    led_green.set(1)

# --- Sensors setup ---
front_ds = robot.getDevice('front_distance_sensor')
left_ds = robot.getDevice('left_distance_sensor')
right_ds = robot.getDevice('right_distance_sensor')
camera = robot.getDevice('endpoint_camera')

left_encoder = robot.getDevice('left_wheel_sensor')
right_encoder = robot.getDevice('right_wheel_sensor')

# Enable all sensors
if front_ds:
    front_ds.enable(TIME_STEP)
if left_ds:
    left_ds.enable(TIME_STEP)
if right_ds:
    right_ds.enable(TIME_STEP)
if camera:
    camera.enable(TIME_STEP)
if left_encoder:
    left_encoder.enable(TIME_STEP)
if right_encoder:
    right_encoder.enable(TIME_STEP)

steps_left = 0
left_speed = MAX_SPEED
right_speed = MAX_SPEED
step_counter = 0

print("[SENSOR VALIDATION INITIALIZED] All sensors configured and enabled.")

while robot.step(TIME_STEP) != -1:
    step_counter += 1

    # --- Sensor Validation Logging (every ~10 steps / ~0.6 seconds) ---
    if step_counter % 10 == 0:
        front_val = front_ds.getValue() if front_ds else -1.0
        left_val = left_ds.getValue() if left_ds else -1.0
        right_val = right_ds.getValue() if right_ds else -1.0
        
        left_pos = left_encoder.getValue() if left_encoder else 0.0
        right_pos = right_encoder.getValue() if right_encoder else 0.0
        
        img = camera.getImage() if camera else None
        cam_status = f"OK ({camera.getWidth()}x{camera.getHeight()})" if (camera and img) else "No Frame"

        print(f"[Step {step_counter}] "
              f"Distance (F/L/R): {front_val:.2f}m | {left_val:.2f}m | {right_val:.2f}m | "
              f"Encoders (L/R): {left_pos:.2f}rad | {right_pos:.2f}rad | "
              f"Camera: {cam_status}")

    # --- Basic motion loop ---
    if steps_left <= 0:
        action = random.choice(['forward', 'forward', 'forward', 'turn_left', 'turn_right'])

        if action == 'forward':
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED
            steps_left = random.randint(20, 60)
        elif action == 'turn_left':
            left_speed = -MAX_SPEED * 0.5
            right_speed = MAX_SPEED * 0.5
            steps_left = random.randint(5, 15)
        else:  # turn_right
            left_speed = MAX_SPEED * 0.5
            right_speed = -MAX_SPEED * 0.5
            steps_left = random.randint(5, 15)

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    steps_left -= 1
