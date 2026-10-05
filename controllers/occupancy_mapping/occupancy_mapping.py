import math
from controller import Robot

robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# ============================================================
# ROBOT
# ============================================================

left_motor = robot.getDevice("left_wheel_motor")
right_motor = robot.getDevice("right_wheel_motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

left_sensor = robot.getDevice("left_wheel_sensor")
right_sensor = robot.getDevice("right_wheel_sensor")

left_sensor.enable(TIME_STEP)
right_sensor.enable(TIME_STEP)

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
# ROBOT GEOMETRY
# ============================================================

WHEEL_R = 0.02
AXLE = 0.064

# Distance from robot center to sensor origin.
# Approximate value from your current robot configuration.
SENSOR_OFFSET = 0.038

# ============================================================
# ARENA
# ============================================================

ARENA_WIDTH = 2.44
ARENA_HEIGHT = 1.22

# ============================================================
# GRID
# ============================================================

CELL = 0.05

GRID_W = int(math.ceil(ARENA_WIDTH / CELL))
GRID_H = int(math.ceil(ARENA_HEIGHT / CELL))

# 0 = unknown
# 1 = free
# 2 = occupied

grid = [
    [0 for _ in range(GRID_W)]
    for _ in range(GRID_H)
]

# ============================================================
# ROBOT POSE
# ============================================================

x = 0.30
y = 0.20
theta = 0.0

previous_left = 0.0
previous_right = 0.0

first_encoder_reading = True


# ============================================================
# WORLD → GRID
# ============================================================

def world_to_grid(wx, wy):

    gx = int(wx / CELL)
    gy = int(wy / CELL)

    if gx < 0 or gx >= GRID_W:
        return None

    if gy < 0 or gy >= GRID_H:
        return None

    return gx, gy


# ============================================================
# MARK CELL
# ============================================================

def mark_cell(wx, wy, value):

    result = world_to_grid(wx, wy)

    if result is None:
        return

    gx, gy = result

    grid[gy][gx] = value


# ============================================================
# PRINT MAP
# ============================================================

def print_map():

    print("\n---------------- MAP ----------------")

    for row in reversed(grid):

        line = ""

        for cell in row:

            if cell == 0:
                line += "?"

            elif cell == 1:
                line += "."

            elif cell == 2:
                line += "#"

        print(line)

    print("-------------------------------------\n")


# ============================================================
# MOTOR
# ============================================================

left_motor.setVelocity(2.0)
right_motor.setVelocity(2.0)


# ============================================================
# MAIN LOOP
# ============================================================

counter = 0

while robot.step(TIME_STEP) != -1:

    # --------------------------------------------------------
    # 1. READ ENCODERS
    # --------------------------------------------------------

    left = left_sensor.getValue()
    right = right_sensor.getValue()

    if first_encoder_reading:

        previous_left = left
        previous_right = right

        first_encoder_reading = False

        continue

    delta_left = left - previous_left
    delta_right = right - previous_right

    previous_left = left
    previous_right = right

    # Wheel distances
    dl = delta_left * WHEEL_R
    dr = delta_right * WHEEL_R

    # Robot displacement
    distance = (dl + dr) / 2.0

    dtheta = (dr - dl) / AXLE

    theta += dtheta

    x += distance * math.cos(theta)
    y += distance * math.sin(theta)

    # --------------------------------------------------------
    # 2. MARK ROBOT POSITION
    # --------------------------------------------------------

    mark_cell(x, y, 1)

    # --------------------------------------------------------
    # 3. READ DISTANCE SENSORS
    # --------------------------------------------------------

    for sensor, angle_deg in zip(sensors, sensor_angles):

        value = sensor.getValue()

        # Ignore invalid/very large readings
        if value <= 0:
            continue

        # Limit maximum sensor range
        distance_sensor = min(value, 0.6)

        angle = math.radians(angle_deg)

        # Sensor direction relative to robot
        sensor_angle = theta + angle

        # End point of ray
        hit_x = (
            x
            + SENSOR_OFFSET * math.cos(theta)
            + distance_sensor * math.cos(sensor_angle)
        )

        hit_y = (
            y
            + SENSOR_OFFSET * math.sin(theta)
            + distance_sensor * math.sin(sensor_angle)
        )

        # ----------------------------------------------------
        # Mark endpoint as occupied
        # ----------------------------------------------------

        mark_cell(hit_x, hit_y, 2)

    # --------------------------------------------------------
    # 4. DISPLAY
    # --------------------------------------------------------

    counter += 1

    if counter >= 20:

        print(
            f"Pose: "
            f"x={x:.3f}, "
            f"y={y:.3f}, "
            f"theta={theta:.3f}"
        )

        print_map()

        counter = 0