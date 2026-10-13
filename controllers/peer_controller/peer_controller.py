"""
peer_controller.py

Basic obstacle avoidance and maze navigation logic adapted to your robot's existing sensors.
"""

from controller import Robot
import math

TIME_STEP = 32
MAX_SPEED = 5.0

robot = Robot()

# 1. Initialize Motors
left_motor = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# 2. Initialize Sensors
# Using all 9 distance sensors from TwoWheelRobot.proto
sensor_names = [
    "ds_m90", "ds_m60", "ds_m40", "ds_m20",
    "ds_0",
    "ds_p20", "ds_p40", "ds_p60", "ds_p90"
]

sensors = {}
for name in sensor_names:
    sensor = robot.getDevice(name)
    if sensor:
        sensor.enable(TIME_STEP)
        sensors[name] = sensor

camera = robot.getDevice('front_camera')
if camera:
    camera.enable(TIME_STEP)

inertial_unit = robot.getDevice('imu')
if inertial_unit:
    inertial_unit.enable(TIME_STEP)

# Step once to initialize the sensors before reading them
robot.step(TIME_STEP)

# Capture the direction the robot is facing at the start of the simulation
if inertial_unit:
    rpy = inertial_unit.getRollPitchYaw()
    TARGET_YAW = rpy[2] if rpy else 0.0
else:
    TARGET_YAW = 0.0

print(f"Target Heading Locked: {TARGET_YAW:.2f} radians")
print("Starting Target Heading + Obstacle Avoidance with Escape Maneuvers...")

# State Variables for the Escape Maneuver
stuck_counter = 0
escape_counter = 0
is_escaping = False

# Helper function to safely read sensor distances
def get_val(name):
    if name in sensors:
        val = sensors[name].getValue()
        if math.isnan(val) or val <= 0:
            return 0.6  # Default to max range if nothing is seen
        return val
    return 0.6

# 4. Main Control Loop
while robot.step(TIME_STEP) != -1:
    
    # Read distance sensor values
    # We take the minimum reading (closest object) on each side
    front_val = get_val("ds_0")
    
    left_val = min(
        get_val("ds_m90"),
        get_val("ds_m60"),
        get_val("ds_m40"),
        get_val("ds_m20")
    )
    
    right_val = min(
        get_val("ds_p90"),
        get_val("ds_p60"),
        get_val("ds_p40"),
        get_val("ds_p20")
    )
    
    # Read current orientation
    if inertial_unit:
        rpy = inertial_unit.getRollPitchYaw()
        current_yaw = rpy[2] if rpy else 0.0
    else:
        current_yaw = 0.0

    # Calculate difference between current heading and target heading
    angle_diff = current_yaw - TARGET_YAW
    
    # Normalize the angle difference to handle the -pi to pi boundary wrap-around
    if angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    elif angle_diff < -math.pi:
        angle_diff += 2 * math.pi

    # In Webots (with your sensors), smaller values mean the obstacle is closer
    # Let's consider anything closer than 15cm as an obstacle
    obstacle_front = front_val < 0.15
    obstacle_left = left_val < 0.15
    obstacle_right = right_val < 0.15

    # ---------------------------------------------
    # ESCAPE MANEUVER LOGIC
    # ---------------------------------------------
    if not is_escaping:
        # If we detect ANY obstacle, increase the stuck counter
        if obstacle_front or obstacle_left or obstacle_right:
            stuck_counter += 1
        else:
            stuck_counter = 0 # Path is clear, reset counter
            
        # If we've been struggling with an obstacle for ~2 seconds (60 steps at 32ms)
        if stuck_counter > 60:
            print("Robot is stuck! Initiating Escape Maneuver!")
            is_escaping = True
            escape_counter = 45  # 45 steps total for the escape (~1.5s)
            stuck_counter = 0

    if is_escaping:
        # Phase 1 of escape: Reverse for 30 steps (~1 second)
        if escape_counter > 15:
            left_speed = -MAX_SPEED
            right_speed = -MAX_SPEED
        # Phase 2 of escape: Spin right for 15 steps (~0.5 seconds)
        else:
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED
            
        escape_counter -= 1
        if escape_counter <= 0:
            print("Escape complete. Resuming normal navigation.")
            is_escaping = False
            
        # Apply speeds and immediately loop (skip normal logic)
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        continue

    # ---------------------------------------------
    # 1. NORMAL OBSTACLE AVOIDANCE (Highest Priority)
    # ---------------------------------------------
    left_speed = MAX_SPEED
    right_speed = MAX_SPEED

    if obstacle_front:
        # Higher value means the obstacle is further away (more space)
        if left_val > right_val:
            left_speed = -MAX_SPEED
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = -MAX_SPEED
            
    elif obstacle_left:
        left_speed = MAX_SPEED
        right_speed = 0.0
        
    elif obstacle_right:
        left_speed = 0.0
        right_speed = MAX_SPEED

    # ---------------------------------------------
    # 2. TARGET HEADING (Run only if path is clear)
    # ---------------------------------------------
    else:
        if angle_diff > 0.1:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED * 0.5
        elif angle_diff < -0.1:
            left_speed = MAX_SPEED * 0.5
            right_speed = MAX_SPEED
        else:
            left_speed = MAX_SPEED
            right_speed = MAX_SPEED

    # Apply speeds to motors
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
