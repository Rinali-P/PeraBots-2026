"""
main_controller.py

PeraBots 2026 - Phase 1
Generalized Section 1 Autonomous Navigation

CURRENT STRATEGY
----------------
1. Localization:
      - Wheel encoders estimate travelled distance.
      - IMU provides heading.
      - Pose = (x, y, theta).

2. Mapping:
      - Distance sensors observe walls/obstacles.
      - Sensor rays update an occupancy grid.
      - Free space and occupied space are stored.

3. Exploration:
      - Robot searches for frontier cells.
      - A frontier = known free space next to unknown space.
      - Robot prefers frontiers that can reveal new space.
      - Already explored areas are not selected as exploration goals.

4. Planning:
      - A* plans from the CURRENT robot position
        to the selected unexplored frontier.

5. Safety:
      - Distance sensors always have priority over A*.
      - If an obstacle becomes dangerously close,
        the robot immediately slows/stops/avoids it.

6. FUTURE CAMERA:
      - Camera can later detect the red transition/end target.
      - That detected target will become the A* goal.
      - No red target coordinate is hard-coded here.

IMPORTANT
---------
- No hard-coded obstacle locations.
- No hard-coded transition coordinate.
- No predetermined obstacle route.
- No boundary-frontier assumption.
- No GPS.
- No Supervisor.
"""

from controller import Robot
import math
import heapq


# ============================================================
# ROBOT / CONTROL PARAMETERS
# ============================================================

TIME_STEP = 32

MAX_MOTOR = 10.0
MAX_WHEEL = 3.0

WHEEL_RADIUS = 0.020
AXLE_LENGTH = 0.064

# ------------------------------------------------------------
# Occupancy grid
# ------------------------------------------------------------

GRID_RES = 0.02

# Robot radius + safety margin.
ROBOT_RADIUS = 0.05
SAFETY_MARGIN = 0.02
INFLATION_RADIUS = ROBOT_RADIUS + SAFETY_MARGIN

# Distance sensor useful range.
SENSOR_RANGE = 0.30

# ------------------------------------------------------------
# Immediate obstacle safety
# ------------------------------------------------------------

FRONT_SLOW = 0.20
FRONT_BLOCK = 0.10
FRONT_EMERGENCY = 0.06

SIDE_SLOW = 0.12
SIDE_BLOCK = 0.06

# ------------------------------------------------------------
# Occupancy log-odds
# ------------------------------------------------------------

OCCUPIED_THRESHOLD = 0.55
FREE_THRESHOLD = -0.25

# ------------------------------------------------------------
# Planning
# ------------------------------------------------------------

REPLAN_TIME = 0.8

# Maximum distance around robot from which frontiers
# are considered.
FRONTIER_SEARCH_RADIUS = 65

# A target is considered reached within this distance.
TARGET_REACHED_DISTANCE = 0.075

# Do not select a frontier immediately next to robot.
MIN_FRONTIER_DISTANCE = 0.12

# ------------------------------------------------------------
# Stuck detection
# ------------------------------------------------------------

STUCK_TIME = 2.5
MIN_PROGRESS = 0.035


# ============================================================
# DISTANCE SENSOR CONFIGURATION
# ============================================================

SENSOR_NAMES = [
    "ds_m90",
    "ds_m60",
    "ds_m40",
    "ds_m20",
    "ds_0",
    "ds_p20",
    "ds_p40",
    "ds_p60",
    "ds_p90",
]

SENSOR_ANGLES = [
    math.radians(-90),
    math.radians(-60),
    math.radians(-40),
    math.radians(-20),
    0.0,
    math.radians(20),
    math.radians(40),
    math.radians(60),
    math.radians(90),
]

# Approximate sensor positions relative to robot centre.
#
# These are only used to improve the map ray origin.
# They are NOT obstacle coordinates.
#
# If the actual PROTO sensor positions differ significantly,
# these values should be changed to match the robot.
SENSOR_OFFSETS = [
    (0.025, -0.030),
    (0.038, -0.020),
    (0.038, -0.013),
    (0.038, -0.006),
    (0.038,  0.000),
    (0.038,  0.006),
    (0.038,  0.013),
    (0.038,  0.020),
    (0.025,  0.030),
]


# ============================================================
# GENERAL UTILITIES
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def distance(a, b):
    return math.hypot(
        a[0] - b[0],
        a[1] - b[1]
    )


# ============================================================
# OCCUPANCY GRID
# ============================================================

class OccupancyGrid:

    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2

    def __init__(self):

        # We use a local map rather than assuming the robot
        # knows an absolute world position.
        #
        # The robot's starting position is the map origin.
        #
        # Large enough for the Section 1 arena and exploration.
        self.w = 180
        self.h = 180

        # Origin is in the centre of this grid.
        self.origin_x = self.w // 2
        self.origin_y = self.h // 2

        self.logodds = [
            [0.0 for _ in range(self.h)]
            for _ in range(self.w)
        ]

    # --------------------------------------------------------
    # Grid boundaries
    # --------------------------------------------------------

    def inside(self, gx, gy):

        return (
            0 <= gx < self.w and
            0 <= gy < self.h
        )

    # --------------------------------------------------------
    # World -> grid
    # --------------------------------------------------------

    def world_to_grid(self, x, y):

        gx = int(
            round(x / GRID_RES)
        ) + self.origin_x

        gy = int(
            round(y / GRID_RES)
        ) + self.origin_y

        return gx, gy

    # --------------------------------------------------------
    # Grid -> world
    # --------------------------------------------------------

    def grid_to_world(self, gx, gy):

        x = (
            gx - self.origin_x
        ) * GRID_RES

        y = (
            gy - self.origin_y
        ) * GRID_RES

        return x, y

    # --------------------------------------------------------
    # Map update
    # --------------------------------------------------------

    def update_free(self, gx, gy):

        if not self.inside(gx, gy):
            return

        self.logodds[gx][gy] -= 0.30

        self.logodds[gx][gy] = max(
            -4.0,
            self.logodds[gx][gy]
        )

    def update_occupied(self, gx, gy):

        if not self.inside(gx, gy):
            return

        self.logodds[gx][gy] += 0.85

        self.logodds[gx][gy] = min(
            5.0,
            self.logodds[gx][gy]
        )

    # --------------------------------------------------------
    # Cell state
    # --------------------------------------------------------

    def state(self, gx, gy):

        if not self.inside(gx, gy):
            return self.OCCUPIED

        value = self.logodds[gx][gy]

        if value > OCCUPIED_THRESHOLD:
            return self.OCCUPIED

        if value < FREE_THRESHOLD:
            return self.FREE

        return self.UNKNOWN

    def occupied(self, gx, gy):

        return (
            self.state(gx, gy)
            == self.OCCUPIED
        )

    def free(self, gx, gy):

        return (
            self.state(gx, gy)
            == self.FREE
        )

    def unknown(self, gx, gy):

        return (
            self.state(gx, gy)
            == self.UNKNOWN
        )


# ============================================================
# ROBOT
# ============================================================

class Bot:

    def __init__(self, robot):

        self.robot = robot

        # ====================================================
        # MOTORS
        # ====================================================

        self.left_motor = robot.getDevice(
            "left_wheel_motor"
        )

        self.right_motor = robot.getDevice(
            "right_wheel_motor"
        )

        self.left_motor.setPosition(
            float("inf")
        )

        self.right_motor.setPosition(
            float("inf")
        )

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # ====================================================
        # ENCODERS
        # ====================================================

        self.left_encoder = robot.getDevice(
            "left_wheel_sensor"
        )

        self.right_encoder = robot.getDevice(
            "right_wheel_sensor"
        )

        self.left_encoder.enable(
            TIME_STEP
        )

        self.right_encoder.enable(
            TIME_STEP
        )

        # ====================================================
        # IMU
        # ====================================================

        self.imu = robot.getDevice(
            "imu"
        )

        self.imu.enable(
            TIME_STEP
        )

        # ====================================================
        # DISTANCE SENSORS
        # ====================================================

        self.sensors = []

        for name in SENSOR_NAMES:

            sensor = robot.getDevice(
                name
            )

            sensor.enable(
                TIME_STEP
            )

            self.sensors.append(
                sensor
            )
        
        # ====================================================
        # CAMERA
        # ====================================================

        self.camera = robot.getDevice("front_camera")
        self.camera.enable(TIME_STEP)

        # ====================================================
        # POSE
        # ====================================================

        # The robot starts at the origin of its own map.
        #
        # This is NOT assuming the competition transition
        # or obstacle coordinates.
        self.x = 0.0
        self.y = 0.0

        # Heading is initialized from the IMU.
        self.th = 0.0

        self.imu_offset = None

        self.prev_left = None
        self.prev_right = None

        # ====================================================
        # MAP
        # ====================================================

        self.grid = OccupancyGrid()

        # ====================================================
        # EXPLORATION MEMORY
        # ====================================================

        # Cells that the robot has physically visited.
        #
        # We do NOT forbid travelling through these cells.
        # They may be necessary to reach a new frontier.
        self.visited_cells = set()

        # Previously selected frontier targets.
        #
        # This prevents repeatedly selecting the exact same
        # exploration point.
        self.completed_frontiers = set()

        # ====================================================
        # DEBUG
        # ====================================================

        self.last_sensor_print = -10.0
        self.last_pose_print = -10.0

    # ========================================================
    # MOTOR CONTROL
    # ========================================================

    def drive(self, left, right):

        left = clamp(
            left,
            -MAX_MOTOR,
            MAX_MOTOR
        )

        right = clamp(
            right,
            -MAX_MOTOR,
            MAX_MOTOR
        )

        self.left_motor.setVelocity(
            left
        )

        self.right_motor.setVelocity(
            right
        )

    def stop(self):

        self.drive(
            0.0,
            0.0
        )

    # ========================================================
    # SENSOR READINGS
    # ========================================================

    def sensor_values(self):

        values = []

        for sensor in self.sensors:

            try:

                value = sensor.getValue()

                if not math.isfinite(value):

                    value = SENSOR_RANGE

                value = clamp(
                    value,
                    0.001,
                    SENSOR_RANGE
                )

            except Exception:

                value = SENSOR_RANGE

            values.append(value)

        return values

    # ========================================================
    # IMU
    # ========================================================

    def imu_heading(self):

        values = self.imu.getRollPitchYaw()

        if values is None:
            return self.th

        return values[2]

    # ========================================================
    # INITIALIZE HEADING
    # ========================================================

    def initialize_pose(self):

        imu_theta = self.imu_heading()

        self.imu_offset = imu_theta

        self.th = 0.0

        print(
            f"[POSE] Initial IMU heading = "
            f"{imu_theta:.3f} rad"
        )

        print(
            "[POSE] Local map origin = (0, 0)"
        )

    # ========================================================
    # POSE UPDATE
    # ========================================================

    def update_pose(self):

        left = self.left_encoder.getValue()
        right = self.right_encoder.getValue()

        # First encoder reading.
        if self.prev_left is None:

            self.prev_left = left
            self.prev_right = right

            return

        # ----------------------------------------------------
        # Wheel movement
        # ----------------------------------------------------

        dl = (
            left - self.prev_left
        ) * WHEEL_RADIUS

        dr = (
            right - self.prev_right
        ) * WHEEL_RADIUS

        self.prev_left = left
        self.prev_right = right

        # Centre displacement.
        dc = (
            dl + dr
        ) * 0.5

        # Wheel odometry heading change.
        dtheta_odo = (
            dr - dl
        ) / AXLE_LENGTH

        # ----------------------------------------------------
        # IMU heading
        # ----------------------------------------------------

        imu_raw = self.imu_heading()

        if self.imu_offset is None:

            self.imu_offset = imu_raw

        imu_theta = wrap(
            imu_raw - self.imu_offset
        )

        # ----------------------------------------------------
        # Odometry predicted heading
        # ----------------------------------------------------

        predicted_theta = wrap(
            self.th + dtheta_odo
        )

        # Difference between IMU and odometry.
        imu_error = wrap(
            imu_theta -
            predicted_theta
        )

        # ----------------------------------------------------
        # Sensor fusion
        # ----------------------------------------------------
        #
        # Odometry gives movement.
        #
        # IMU stabilizes orientation.
        #
        # We use a moderate correction rather than letting
        # one sensor completely dominate.
        # ----------------------------------------------------

        self.th = wrap(
            predicted_theta +
            0.35 * imu_error
        )

        # ----------------------------------------------------
        # Position
        # ----------------------------------------------------

        self.x += (
            dc *
            math.cos(self.th)
        )

        self.y += (
            dc *
            math.sin(self.th)
        )

        # ----------------------------------------------------
        # Mark current map cell as visited.
        # ----------------------------------------------------

        gx, gy = self.grid.world_to_grid(
            self.x,
            self.y
        )

        self.visited_cells.add(
            (gx, gy)
        )

    # ========================================================
    # SENSOR RAY TRANSFORMATION
    # ========================================================

    def sensor_world_pose(
        self,
        index
    ):

        offset_x, offset_y = (
            SENSOR_OFFSETS[index]
        )

        # Rotate sensor offset into world frame.

        world_x = (
            self.x +
            offset_x *
            math.cos(self.th) -
            offset_y *
            math.sin(self.th)
        )

        world_y = (
            self.y +
            offset_x *
            math.sin(self.th) +
            offset_y *
            math.cos(self.th)
        )

        return (
            world_x,
            world_y
        )

    # ========================================================
    # MAP UPDATE
    # ========================================================

    def update_map(self):

        readings = self.sensor_values()

        for index, (
            distance_value,
            sensor_angle
        ) in enumerate(
            zip(
                readings,
                SENSOR_ANGLES
            )
        ):

            # ------------------------------------------------
            # Sensor world position
            # ------------------------------------------------

            sx, sy = self.sensor_world_pose(
                index
            )

            # ------------------------------------------------
            # Sensor world direction
            # ------------------------------------------------

            angle = wrap(
                self.th +
                sensor_angle
            )

            # ------------------------------------------------
            # Determine whether the sensor actually saw
            # an obstacle.
            # ------------------------------------------------

            hit_obstacle = (
                distance_value <
                SENSOR_RANGE * 0.97
            )

            # ------------------------------------------------
            # If no obstacle was detected, the ray still
            # tells us that space up to SENSOR_RANGE is free.
            # ------------------------------------------------

            ray_distance = min(
                distance_value,
                SENSOR_RANGE
            )

            # ------------------------------------------------
            # Endpoint
            # ------------------------------------------------

            ex = (
                sx +
                ray_distance *
                math.cos(angle)
            )

            ey = (
                sy +
                ray_distance *
                math.sin(angle)
            )

            # ------------------------------------------------
            # Grid coordinates
            # ------------------------------------------------

            gx0, gy0 = (
                self.grid.world_to_grid(
                    sx,
                    sy
                )
            )

            gx1, gy1 = (
                self.grid.world_to_grid(
                    ex,
                    ey
                )
            )

            # ------------------------------------------------
            # Ray tracing
            # ------------------------------------------------

            dx = gx1 - gx0
            dy = gy1 - gy0

            steps = max(
                abs(dx),
                abs(dy),
                1
            )

            for i in range(steps):

                t = i / steps

                gx = int(
                    round(
                        gx0 +
                        dx * t
                    )
                )

                gy = int(
                    round(
                        gy0 +
                        dy * t
                    )
                )

                self.grid.update_free(
                    gx,
                    gy
                )

            # ------------------------------------------------
            # If an obstacle was detected, mark the endpoint
            # occupied.
            # ------------------------------------------------

            if hit_obstacle:

                self.grid.update_occupied(
                    gx1,
                    gy1
                )

    # ========================================================
    # OBSTACLE INFORMATION
    # ========================================================

    def obstacle_information(self):

        r = self.sensor_values()

        return {

            "left": r[0],

            "left60": r[1],

            "left40": r[2],

            "left20": r[3],

            "front": r[4],

            "right20": r[5],

            "right40": r[6],

            "right60": r[7],

            "right": r[8],
        }

    # ========================================================
    # FRONTIER DETECTION
    # ========================================================

    def frontier_cells(self):

        frontiers = []

        gx0, gy0 = (
            self.grid.world_to_grid(
                self.x,
                self.y
            )
        )

        min_x = max(
            1,
            gx0 -
            FRONTIER_SEARCH_RADIUS
        )

        max_x = min(
            self.grid.w - 2,
            gx0 +
            FRONTIER_SEARCH_RADIUS
        )

        min_y = max(
            1,
            gy0 -
            FRONTIER_SEARCH_RADIUS
        )

        max_y = min(
            self.grid.h - 2,
            gy0 +
            FRONTIER_SEARCH_RADIUS
        )

        # ----------------------------------------------------
        # Search known free cells.
        # ----------------------------------------------------

        for gx in range(
            min_x,
            max_x + 1
        ):

            for gy in range(
                min_y,
                max_y + 1
            ):

                # Frontier must itself be free.
                if not self.grid.free(
                    gx,
                    gy
                ):
                    continue

                # ------------------------------------------------
                # Do not use an already completed frontier.
                # ------------------------------------------------

                if (
                    gx,
                    gy
                ) in self.completed_frontiers:

                    continue

                # ------------------------------------------------
                # Check neighbouring cells for UNKNOWN space.
                # ------------------------------------------------

                has_unknown = False

                for dx, dy in (
                    (1, 0),
                    (-1, 0),
                    (0, 1),
                    (0, -1)
                ):

                    nx = gx + dx
                    ny = gy + dy

                    if not self.grid.inside(
                        nx,
                        ny
                    ):
                        continue

                    if self.grid.unknown(
                        nx,
                        ny
                    ):

                        has_unknown = True
                        break

                if not has_unknown:
                    continue

                wx, wy = (
                    self.grid.grid_to_world(
                        gx,
                        gy
                    )
                )

                d = math.hypot(
                    wx - self.x,
                    wy - self.y
                )

                # Avoid selecting a frontier directly
                # underneath the robot.

                if d < MIN_FRONTIER_DISTANCE:
                    continue

                frontiers.append(
                    (
                        gx,
                        gy,
                        wx,
                        wy
                    )
                )

        return frontiers

    # ========================================================
    # FRONTIER INFORMATION GAIN
    # ========================================================

    def frontier_information_gain(
        self,
        gx,
        gy
    ):

        # Count unknown cells around this frontier.
        #
        # A frontier with more unknown neighbours has a higher
        # probability of revealing new space.

        gain = 0

        radius = 4

        for dx in range(
            -radius,
            radius + 1
        ):

            for dy in range(
                -radius,
                radius + 1
            ):

                if (
                    dx * dx +
                    dy * dy
                ) > radius * radius:

                    continue

                nx = gx + dx
                ny = gy + dy

                if not self.grid.inside(
                    nx,
                    ny
                ):
                    continue

                if self.grid.unknown(
                    nx,
                    ny
                ):

                    gain += 1

        return gain

    # ========================================================
    # CHOOSE EXPLORATION TARGET
    # ========================================================

    def choose_target(self):

        frontiers = (
            self.frontier_cells()
        )

        if not frontiers:
            return None

        best_target = None
        best_score = float("inf")

        for gx, gy, wx, wy in frontiers:

            distance_to_frontier = math.hypot(
                wx - self.x,
                wy - self.y
            )

            information_gain = (
                self.frontier_information_gain(
                    gx,
                    gy
                )
            )

            # ------------------------------------------------
            # Score
            # ------------------------------------------------
            #
            # Lower score is better.
            #
            # Distance matters because we do not want the robot
            # travelling unnecessarily far.
            #
            # Information gain is rewarded because the purpose
            # of the frontier is to discover new space.
            # ------------------------------------------------

            score = (
                distance_to_frontier
                -
                0.006 *
                information_gain
            )

            # Small penalty for cells already physically
            # visited by the robot.
            if (
                gx,
                gy
            ) in self.visited_cells:

                score += 0.08

            if score < best_score:

                best_score = score

                best_target = (
                    wx,
                    wy
                )

        return best_target

    # ========================================================
    # GRID INFLATION
    # ========================================================

    def inflated_occupied(
        self,
        gx,
        gy
    ):

        radius_cells = int(
            math.ceil(
                INFLATION_RADIUS /
                GRID_RES
            )
        )

        for dx in range(
            -radius_cells,
            radius_cells + 1
        ):

            for dy in range(
                -radius_cells,
                radius_cells + 1
            ):

                if (
                    dx * dx +
                    dy * dy
                ) > (
                    radius_cells *
                    radius_cells
                ):

                    continue

                nx = gx + dx
                ny = gy + dy

                if self.grid.occupied(
                    nx,
                    ny
                ):

                    return True

        return False

    # ========================================================
    # A*
    # ========================================================

    def astar(
        self,
        start,
        goal
    ):

        sx, sy = start
        gx, gy = goal

        if not self.grid.inside(
            sx,
            sy
        ):
            return []

        if not self.grid.inside(
            gx,
            gy
        ):
            return []

        # ----------------------------------------------------
        # If goal is occupied, no path.
        # ----------------------------------------------------

        if self.inflated_occupied(
            gx,
            gy
        ):

            return []

        open_set = []

        heapq.heappush(
            open_set,
            (
                0.0,
                (sx, sy)
            )
        )

        came_from = {}

        g_cost = {
            (sx, sy): 0.0
        }

        closed = set()

        # ----------------------------------------------------
        # Heuristic
        # ----------------------------------------------------

        def heuristic(a, b):

            return math.hypot(
                a[0] - b[0],
                a[1] - b[1]
            )

        # ----------------------------------------------------
        # Search
        # ----------------------------------------------------

        while open_set:

            _, current = (
                heapq.heappop(
                    open_set
                )
            )

            if current in closed:
                continue

            closed.add(
                current
            )

            # ------------------------------------------------
            # Goal reached
            # ------------------------------------------------

            if current == (
                gx,
                gy
            ):

                path = []

                node = current

                while node in came_from:

                    path.append(
                        node
                    )

                    node = came_from[
                        node
                    ]

                path.append(
                    (
                        sx,
                        sy
                    )
                )

                path.reverse()

                return path

            cx, cy = current

            # ------------------------------------------------
            # 8-connected movement
            # ------------------------------------------------

            neighbours = (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1)
            )

            for dx, dy in neighbours:

                nx = cx + dx
                ny = cy + dy

                if not self.grid.inside(
                    nx,
                    ny
                ):
                    continue

                # Never plan through occupied/inflated cells.

                if self.inflated_occupied(
                    nx,
                    ny
                ):

                    continue

                # ------------------------------------------------
                # Prevent diagonal corner cutting.
                # ------------------------------------------------

                if (
                    abs(dx) == 1 and
                    abs(dy) == 1
                ):

                    if (
                        self.inflated_occupied(
                            cx + dx,
                            cy
                        )
                        or
                        self.inflated_occupied(
                            cx,
                            cy + dy
                        )
                    ):

                        continue

                diagonal = (
                    abs(dx) == 1 and
                    abs(dy) == 1
                )

                step_cost = (
                    1.4142
                    if diagonal
                    else 1.0
                )

                # ------------------------------------------------
                # Unknown space is slightly more expensive.
                #
                # The target itself is a frontier, so the robot
                # can approach the boundary of unknown space.
                # ------------------------------------------------

                if self.grid.unknown(
                    nx,
                    ny
                ):

                    step_cost *= 1.25

                new_cost = (
                    g_cost[current]
                    +
                    step_cost
                )

                neighbour = (
                    nx,
                    ny
                )

                if (
                    neighbour not in g_cost
                    or
                    new_cost <
                    g_cost[neighbour]
                ):

                    g_cost[
                        neighbour
                    ] = new_cost

                    priority = (
                        new_cost
                        +
                        heuristic(
                            neighbour,
                            (gx, gy)
                        )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            priority,
                            neighbour
                        )
                    )

                    came_from[
                        neighbour
                    ] = current

        return []

    # ========================================================
    # PATH LINE OF SIGHT
    # ========================================================

    def line_is_clear(
        self,
        a,
        b
    ):

        x0, y0 = a
        x1, y1 = b

        dist = math.hypot(
            x1 - x0,
            y1 - y0
        )

        steps = max(
            1,
            int(
                dist /
                GRID_RES
            )
        )

        for i in range(
            steps + 1
        ):

            t = i / steps

            x = (
                x0 +
                (x1 - x0) * t
            )

            y = (
                y0 +
                (y1 - y0) * t
            )

            gx, gy = (
                self.grid.world_to_grid(
                    x,
                    y
                )
            )

            if self.inflated_occupied(
                gx,
                gy
            ):

                return False

        return True

    # ========================================================
    # PATH SIMPLIFICATION
    # ========================================================

    def simplify_path(
        self,
        path
    ):

        if len(path) <= 2:
            return path

        result = []

        i = 0

        while i < len(path):

            result.append(
                path[i]
            )

            best = i + 1

            for j in range(
                i + 1,
                len(path)
            ):

                a = (
                    self.grid.grid_to_world(
                        *path[i]
                    )
                )

                b = (
                    self.grid.grid_to_world(
                        *path[j]
                    )
                )

                if self.line_is_clear(
                    a,
                    b
                ):

                    best = j

            i = best

        return result

    # ========================================================
    # DEBUG PRINT
    # ========================================================

    def debug_print(self):

        now = self.robot.getTime()

        if (
            now -
            self.last_pose_print
            > 1.0
        ):

            self.last_pose_print = now

            print(
                "[POSE] "
                f"x={self.x:.3f} "
                f"y={self.y:.3f} "
                f"theta={math.degrees(self.th):.1f}°"
            )

        if (
            now -
            self.last_sensor_print
            > 2.0
        ):

            self.last_sensor_print = now

            r = self.sensor_values()

            print(
                "[SENSORS] "
                f"L={r[0]:.2f} "
                f"L60={r[1]:.2f} "
                f"L40={r[2]:.2f} "
                f"L20={r[3]:.2f} "
                f"F={r[4]:.2f} "
                f"R20={r[5]:.2f} "
                f"R40={r[6]:.2f} "
                f"R60={r[7]:.2f} "
                f"R={r[8]:.2f}"
            )

    # ========================================================
    # STEP
    # ========================================================

    def step(self):

        if self.robot.step(
            TIME_STEP
        ) == -1:

            return False

        self.update_pose()

        self.update_map()

        self.debug_print()

        return True


# ============================================================
# NAVIGATOR
# ============================================================

class Navigator:

    def __init__(
        self,
        bot
    ):

        self.bot = bot

        # Current A* path.
        self.path = []

        self.path_index = 0

        # Current exploration target.
        self.target = None

        self.last_plan_time = -100.0

        # ----------------------------------------------------
        # Progress monitoring
        # ----------------------------------------------------

        self.last_progress_x = bot.x
        self.last_progress_y = bot.y

        self.last_progress_time = 0.0

        # ----------------------------------------------------
        # Recovery
        # ----------------------------------------------------

        self.recovery_until = 0.0

        self.recovery_direction = 1.0

        # ----------------------------------------------------
        # Target management
        # ----------------------------------------------------

        self.target_reached = False

    # ========================================================
    # TIME
    # ========================================================

    def now(self):

        return self.bot.robot.getTime()

    # ========================================================
    # TARGET REACHED
    # ========================================================

    def target_is_reached(self):

        if self.target is None:
            return True

        d = math.hypot(
            self.target[0] -
            self.bot.x,

            self.target[1] -
            self.bot.y
        )

        return (
            d <
            TARGET_REACHED_DISTANCE
        )

    # ========================================================
    # PLAN
    # ========================================================

    def plan(
        self,
        force_new_target=False
    ):

        # ----------------------------------------------------
        # Choose a new exploration frontier only when needed.
        #
        # This is important.
        #
        # We do NOT choose a different frontier every second.
        # Otherwise the robot can continuously change its mind.
        # ----------------------------------------------------

        if (
            force_new_target
            or
            self.target is None
            or
            self.target_is_reached()
        ):

            if self.target is not None:

                old_gx, old_gy = (
                    self.bot.grid.world_to_grid(
                        self.target[0],
                        self.target[1]
                    )
                )

                self.bot.completed_frontiers.add(
                    (
                        old_gx,
                        old_gy
                    )
                )

            self.target = (
                self.bot.choose_target()
            )

            if self.target is None:

                self.path = []

                print(
                    "[PLAN] No unexplored frontier found."
                )

                return False

        # ----------------------------------------------------
        # Current robot cell
        # ----------------------------------------------------

        sx, sy = (
            self.bot.grid.world_to_grid(
                self.bot.x,
                self.bot.y
            )
        )

        # ----------------------------------------------------
        # Target cell
        # ----------------------------------------------------

        gx, gy = (
            self.bot.grid.world_to_grid(
                self.target[0],
                self.target[1]
            )
        )

        # ----------------------------------------------------
        # A*
        # ----------------------------------------------------

        raw_path = self.bot.astar(
            (
                sx,
                sy
            ),
            (
                gx,
                gy
            )
        )

        if not raw_path:

            print(
                "[PLAN] Target currently unreachable."
            )

            # Make this target unavailable for now.
            self.bot.completed_frontiers.add(
                (
                    gx,
                    gy
                )
            )

            self.target = None
            self.path = []

            return False

        self.path = (
            self.bot.simplify_path(
                raw_path
            )
        )

        self.path_index = 0

        self.last_plan_time = (
            self.now()
        )

        print(
            "[PLAN] "
            f"target=({self.target[0]:.2f}, "
            f"{self.target[1]:.2f}) "
            f"path={len(self.path)}"
        )

        return True

    # ========================================================
    # CURRENT WAYPOINT
    # ========================================================

    def current_waypoint(self):

        if not self.path:
            return None

        while (
            self.path_index <
            len(self.path)
        ):

            gx, gy = (
                self.path[
                    self.path_index
                ]
            )

            wx, wy = (
                self.bot.grid.grid_to_world(
                    gx,
                    gy
                )
            )

            d = math.hypot(
                wx - self.bot.x,
                wy - self.bot.y
            )

            if d < 0.12:

                self.path_index += 1

            else:

                return (
                    wx,
                    wy
                )

        return None

    # ========================================================
    # IMMEDIATE SENSOR SAFETY
    # ========================================================

    def reactive_avoidance(self):

        info = (
            self.bot.obstacle_information()
        )

        front = info["front"]

        # ----------------------------------------------------
        # Use ALL sensors to find the most free path
        # ----------------------------------------------------
        
        left_max = max(
            info["left"], 
            info["left60"], 
            info["left40"], 
            info["left20"]
        )
        
        right_max = max(
            info["right"], 
            info["right60"], 
            info["right40"], 
            info["right20"]
        )

        # ----------------------------------------------------
        # Emergency front obstacle
        # ----------------------------------------------------

        if front < FRONT_EMERGENCY:

            if left_max > right_max:
                self.recovery_direction = -1.0
            else:
                self.recovery_direction = 1.0

            return "emergency"

        # ----------------------------------------------------
        # Front blocked
        # ----------------------------------------------------

        if front < FRONT_BLOCK:

            if left_max > right_max:
                self.recovery_direction = -1.0
            else:
                self.recovery_direction = 1.0

            return "blocked"

        # ----------------------------------------------------
        # Side collision protection
        # ----------------------------------------------------

        if info["left20"] < SIDE_BLOCK or info["left"] < SIDE_BLOCK / 1.5:

            self.recovery_direction = 1.0

            return "left_side"

        if info["right20"] < SIDE_BLOCK or info["right"] < SIDE_BLOCK / 1.5:

            self.recovery_direction = -1.0

            return "right_side"

        return None

    # ========================================================
    # RECOVERY CONTROL
    # ========================================================

    def recovery_control(self):

        info = (
            self.bot.obstacle_information()
        )

        front = info["front"]

        # ----------------------------------------------------
        # If extremely close, reverse.
        # ----------------------------------------------------

        if front < FRONT_EMERGENCY:

            self.bot.drive(
                -1.5,
                -1.5
            )

            return

        # ----------------------------------------------------
        # Otherwise make a curved escape.
        # ----------------------------------------------------

        if self.recovery_direction > 0:

            self.bot.drive(
                1.0,
                -1.0
            )

        else:

            self.bot.drive(
                -1.0,
                1.0
            )

    # ========================================================
    # SPEED LIMIT FROM SENSORS
    # ========================================================

    def safe_speed(self):

        info = (
            self.bot.obstacle_information()
        )

        front = info["front"]

        # Very close.
        if front < FRONT_EMERGENCY:

            return 0.0

        # Close.
        if front < FRONT_BLOCK:

            return 1.0

        # Start slowing down before obstacle.
        if front < FRONT_SLOW:

            ratio = (
                front -
                FRONT_BLOCK
            ) / (
                FRONT_SLOW -
                FRONT_BLOCK
            )

            return clamp(
                1.5 +
                2.0 * ratio,
                1.5,
                3.5
            )

        return MAX_WHEEL

    # ========================================================
    # MAIN CONTROL
    # ========================================================

    def control(self):

        now = self.now()

        # ====================================================
        # RECOVERY MODE
        # ====================================================

        if now < self.recovery_until:

            self.recovery_control()

            return

        # ====================================================
        # PROGRESS MONITOR
        # ====================================================

        progress = math.hypot(
            self.bot.x -
            self.last_progress_x,

            self.bot.y -
            self.last_progress_y
        )

        if progress > MIN_PROGRESS:

            self.last_progress_x = (
                self.bot.x
            )

            self.last_progress_y = (
                self.bot.y
            )

            self.last_progress_time = now

        elif (
            now -
            self.last_progress_time
            > STUCK_TIME
        ):

            print(
                "[RECOVERY] "
                "Robot appears stuck."
            )

            self.recovery_direction *= -1.0

            self.recovery_until = (
                now + 0.8
            )

            self.path = []

            self.bot.stop()

            return

        # ====================================================
        # IMMEDIATE SENSOR SAFETY
        # ====================================================

        avoidance = (
            self.reactive_avoidance()
        )

        if avoidance is not None:

            if avoidance == "emergency":

                self.recovery_until = (
                    now + 0.45
                )

            else:

                self.recovery_until = (
                    now + 0.35
                )

            self.path = []

            self.recovery_control()

            return

        # ====================================================
        # CHECK CURRENT TARGET
        # ====================================================

        if self.target_is_reached():

            print(
                "[EXPLORE] "
                "Frontier reached."
            )

            self.path = []

            self.target = None

            self.plan(
                force_new_target=True
            )

        # ====================================================
        # REPLAN
        # ====================================================

        elif (
            not self.path
            or
            now -
            self.last_plan_time
            > REPLAN_TIME
        ):

            # Replan the route to the SAME target.
            #
            # We only choose a new target when the old target
            # has been reached or becomes impossible.

            self.plan()

        # ====================================================
        # WAYPOINT
        # ====================================================

        waypoint = (
            self.current_waypoint()
        )

        if waypoint is None:

            self.plan(
                force_new_target=True
            )

            waypoint = (
                self.current_waypoint()
            )

            if waypoint is None:

                # No target currently available.
                #
                # Do NOT blindly drive forward.
                # Stop and let the map/exploration logic
                # decide what to do next.

                self.bot.stop()

                return

        wx, wy = waypoint

        # ====================================================
        # HEADING ERROR
        # ====================================================

        desired_heading = math.atan2(
            wy - self.bot.y,
            wx - self.bot.x
        )

        heading_error = wrap(
            desired_heading -
            self.bot.th
        )

        # ====================================================
        # SENSOR-BASED SPEED
        # ====================================================

        speed = self.safe_speed()

        if speed <= 0.0:

            self.bot.stop()

            return

        # Reduce speed when heading error is large.

        if abs(heading_error) > 1.3:

            speed = min(
                speed,
                1.2
            )

        elif abs(heading_error) > 0.7:

            speed = min(
                speed,
                2.2
            )

        # ====================================================
        # PROPORTIONAL STEERING
        # ====================================================

        steering = (
            0.6 *
            heading_error
        )

        steering = clamp(steering, -0.6, 0.6)

        left = (
            speed -
            steering
        )

        right = (
            speed +
            steering
        )

        # ====================================================
        # SIDE CLEARANCE CORRECTION
        # ====================================================

        info = (
            self.bot.obstacle_information()
        )

        # If very close to left wall, gently move right.
        if info["left20"] < SIDE_SLOW:

            correction = (
                SIDE_SLOW -
                info["left20"]
            ) * 5.0

            left += correction
            right -= correction

        # If very close to right wall, gently move left.
        if info["right20"] < SIDE_SLOW:

            correction = (
                SIDE_SLOW -
                info["right20"]
            ) * 5.0

            left -= correction
            right += correction

        # ====================================================
        # FINAL MOTOR LIMIT
        # ====================================================

        left = clamp(
            left,
            -MAX_WHEEL,
            MAX_WHEEL
        )

        right = clamp(
            right,
            -MAX_WHEEL,
            MAX_WHEEL
        )

        self.bot.drive(
            left,
            right
        )


# ============================================================
# MAIN
# ============================================================

def main():

    robot = Robot()

    bot = Bot(
        robot
    )

    navigator = Navigator(
        bot
    )

    print()
    print(
        "=========================================="
    )
    print(
        " PeraBots 2026 - Section 1"
    )
    print(
        " Localization + Mapping + Exploration"
    )
    print(
        "=========================================="
    )
    print()

    print(
        "Localization : Encoders + IMU"
    )

    print(
        "Mapping      : Distance sensors"
    )

    print(
        "Exploration  : Frontier detection"
    )

    print(
        "Planning     : A*"
    )

    print(
        "Safety       : Reactive sensor layer"
    )

    print()

    print(
        "Camera target detection is NOT enabled yet."
    )

    print(
        "Red target will be added later."
    )

    print()

    # ========================================================
    # INITIAL SENSOR SETTLE
    # ========================================================

    for _ in range(10):

        if not bot.step():

            return

    # ========================================================
    # INITIALIZE LOCALIZATION
    # ========================================================

    bot.initialize_pose()

    navigator.last_progress_time = (
        robot.getTime()
    )

    navigator.last_progress_x = (
        bot.x
    )

    navigator.last_progress_y = (
        bot.y
    )

    # ========================================================
    # INITIAL MAP UPDATE
    # ========================================================

    bot.update_map()

    # ========================================================
    # MAIN LOOP
    # ========================================================

    while robot.step(
        TIME_STEP
    ) != -1:

        # ----------------------------------------------------
        # Update localization.
        # ----------------------------------------------------

        bot.update_pose()

        # ----------------------------------------------------
        # Update occupancy map from sensors.
        # ----------------------------------------------------

        bot.update_map()

        # ----------------------------------------------------
        # Navigation / exploration.
        # ----------------------------------------------------

        navigator.control()


if __name__ == "__main__":

    main()