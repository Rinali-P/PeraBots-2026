from controller import Robot
import math


# ============================================================
# BASIC SETTINGS
# ============================================================

TIME_STEP = 32

# Wheel
WHEEL_RADIUS = 0.02       # 2 cm
AXLE_LENGTH = 0.064       # distance between wheels

# Speeds
FORWARD_SPEED = 1
TURN_SPEED = 1

# Safety distance from obstacle
# We want approximately 10 cm clearance.
SAFE_DISTANCE = 0.10

# Start position of our OWN map
# The robot does NOT know its real arena coordinates.
x = 0.0
y = 0.0
theta = 0.0


# ============================================================
# ROBOT
# ============================================================

robot = Robot()


# ============================================================
# MOTORS
# ============================================================

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

# Velocity-control mode
left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)


# ============================================================
# ENCODERS
# ============================================================

left_encoder = robot.getDevice("left_wheel_sensor")
right_encoder = robot.getDevice("right_wheel_sensor")

left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)


# ============================================================
# IMU
# ============================================================

imu = robot.getDevice("imu")
imu.enable(TIME_STEP)


# ============================================================
# DISTANCE SENSORS
# ============================================================

sensor_names = [
    "ds_m90",
    "ds_m60",
    "ds_m40",
    "ds_m20",
    "ds_0",
    "ds_p20",
    "ds_p40",
    "ds_p60",
    "ds_p90"
]

sensor_angles = [
    -90,
    -60,
    -40,
    -20,
    0,
    20,
    40,
    60,
    90
]

sensors = []

for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    sensors.append(sensor)


# ============================================================
# ODOMETRY VARIABLES
# ============================================================

previous_left = 0.0
previous_right = 0.0


# ============================================================
# SMALL EXPLORATION MAP
# ============================================================
#
# This is NOT the arena map.
#
# It is simply the robot remembering:
#
# "I have already been around here."
#
# Each cell represents 10 cm x 10 cm.
#

MAP_CELL = 0.10

visited_cells = set()


def get_cell(x, y):
    """Convert robot position into a small map cell."""

    cell_x = round(x / MAP_CELL)
    cell_y = round(y / MAP_CELL)

    return (cell_x, cell_y)


def remember_position():
    """Remember where the robot has been."""

    cell = get_cell(x, y)
    visited_cells.add(cell)


def was_visited(x_pos, y_pos):
    """Check whether a position was already visited."""

    cell = get_cell(x_pos, y_pos)

    return cell in visited_cells


# ============================================================
# MOTOR FUNCTION
# ============================================================

def set_speed(left, right):

    left_motor.setVelocity(left)
    right_motor.setVelocity(right)


def stop():

    set_speed(0.0, 0.0)


# ============================================================
# IMU HEADING
# ============================================================

def get_yaw():

    roll, pitch, yaw = imu.getRollPitchYaw()

    return yaw


# ============================================================
# ODOMETRY
# ============================================================

def update_odometry():

    global previous_left
    global previous_right
    global x
    global y
    global theta

    current_left = left_encoder.getValue()
    current_right = right_encoder.getValue()

    # Change in wheel rotation
    delta_left = current_left - previous_left
    delta_right = current_right - previous_right

    previous_left = current_left
    previous_right = current_right

    # Wheel movement
    left_distance = delta_left * WHEEL_RADIUS
    right_distance = delta_right * WHEEL_RADIUS

    # Robot movement
    distance = (left_distance + right_distance) / 2.0

    # Use IMU for heading
    theta = get_yaw()

    # Update our OWN coordinate system
    x += distance * math.cos(theta)
    y += distance * math.sin(theta)


# ============================================================
# SENSOR FUNCTIONS
# ============================================================

def read_sensor(index):

    value = sensors[index].getValue()

    # Protect against invalid values
    if value <= 0:
        return 0.0

    return value


def get_front_distance():

    # Sensors around the front
    values = [
        read_sensor(3),   # -20
        read_sensor(4),   # 0
        read_sensor(5)    # +20
    ]

    return min(values)


def get_left_distance():

    values = [
        read_sensor(0),   # -90
        read_sensor(1),   # -60
        read_sensor(2)    # -40
    ]

    return min(values)


def get_right_distance():

    values = [
        read_sensor(6),   # +40
        read_sensor(7),   # +60
        read_sensor(8)    # +90
    ]

    return min(values)


# ============================================================
# CHECK WHETHER A DIRECTION HAS SPACE
# ============================================================

def direction_is_free(angle_degrees, distance=0.20):

    """
    Predict a point in front of the robot.

    We use this only for our small visited-position memory.
    """

    angle = theta + math.radians(angle_degrees)

    test_x = x + distance * math.cos(angle)
    test_y = y + distance * math.sin(angle)

    return not was_visited(test_x, test_y)


# ============================================================
# CHOOSE AVOIDANCE DIRECTION
# ============================================================

def choose_direction():

    left_distance = get_left_distance()
    right_distance = get_right_distance()

    left_free = direction_is_free(-60)
    right_free = direction_is_free(60)

    print(
        "Left:",
        round(left_distance, 3),
        " Right:",
        round(right_distance, 3)
    )

    # --------------------------------------------------------
    # Prefer a direction that we have not visited
    # --------------------------------------------------------

    if left_free and not right_free:
        return "LEFT"

    if right_free and not left_free:
        return "RIGHT"

    # --------------------------------------------------------
    # If both are new, choose the direction with more space
    # --------------------------------------------------------

    if left_distance > right_distance:
        return "LEFT"

    else:
        return "RIGHT"


# ============================================================
# TURN
# ============================================================

def turn_left():

    print("Turning LEFT")

    set_speed(-TURN_SPEED, TURN_SPEED)

    # Turn at least a minimum angle, then keep turning until front is clear
    start_angle = get_yaw()

    while robot.step(TIME_STEP) != -1:

        update_odometry()
        remember_position()

        current_angle = get_yaw()

        difference = current_angle - start_angle

        # Normalize angle
        while difference > math.pi:
            difference -= 2 * math.pi

        while difference < -math.pi:
            difference += 2 * math.pi

        # Check if we have turned at least 45 degrees and the path is clear
        if abs(difference) >= math.radians(45):
            if get_front_distance() > SAFE_DISTANCE + 0.05:
                break

    stop()


def turn_right():

    print("Turning RIGHT")

    set_speed(TURN_SPEED, -TURN_SPEED)

    # Turn at least a minimum angle, then keep turning until front is clear
    start_angle = get_yaw()

    while robot.step(TIME_STEP) != -1:

        update_odometry()
        remember_position()

        current_angle = get_yaw()

        difference = current_angle - start_angle

        # Normalize angle
        while difference > math.pi:
            difference -= 2 * math.pi

        while difference < -math.pi:
            difference += 2 * math.pi

        # Check if we have turned at least 45 degrees and the path is clear
        if abs(difference) >= math.radians(45):
            if get_front_distance() > SAFE_DISTANCE + 0.05:
                break

    stop()


# ============================================================
# INITIALIZE ENCODERS
# ============================================================

robot.step(TIME_STEP)

previous_left = left_encoder.getValue()
previous_right = right_encoder.getValue()

theta = get_yaw()

remember_position()


# ============================================================
# MAIN LOOP
# ============================================================

print("======================================")
print(" SIMPLE EXPLORATION CONTROLLER")
print("======================================")
print("Robot starts its own map at (0, 0)")
print("Safe obstacle distance:", SAFE_DISTANCE)
print("Map cell:", MAP_CELL)
print()


while robot.step(TIME_STEP) != -1:

    # --------------------------------------------------------
    # 1. Update position
    # --------------------------------------------------------

    update_odometry()

    # --------------------------------------------------------
    # 2. Remember current location
    # --------------------------------------------------------

    remember_position()

    # --------------------------------------------------------
    # 3. Read obstacles
    # --------------------------------------------------------

    front = get_front_distance()

    # --------------------------------------------------------
    # 4. Print information
    # --------------------------------------------------------

    print(
        "Position:",
        round(x, 2),
        round(y, 2),
        "Heading:",
        round(math.degrees(theta), 1),
        "Front:",
        round(front, 3)
    )

    # --------------------------------------------------------
    # 5. Obstacle detected
    # --------------------------------------------------------

    if front > 0 and front < SAFE_DISTANCE:

        print("OBSTACLE DETECTED")

        stop()

        # Let robot settle
        for _ in range(5):

            if robot.step(TIME_STEP) == -1:
                break

            update_odometry()
            remember_position()

        # ----------------------------------------------------
        # Choose left or right
        # ----------------------------------------------------

        direction = choose_direction()

        if direction == "LEFT":

            turn_left()

        else:

            turn_right()

        continue

    # --------------------------------------------------------
    # 6. Check whether forward area was already visited
    # --------------------------------------------------------

    forward_distance = 0.15

    future_x = x + forward_distance * math.cos(theta)
    future_y = y + forward_distance * math.sin(theta)

    if was_visited(future_x, future_y):

        print("Already visited area -> choosing new direction")

        stop()

        direction = choose_direction()

        if direction == "LEFT":

            turn_left()

        else:

            turn_right()

        continue

    # --------------------------------------------------------
    # 7. Nothing dangerous -> go forward
    # --------------------------------------------------------

    set_speed(FORWARD_SPEED, FORWARD_SPEED)