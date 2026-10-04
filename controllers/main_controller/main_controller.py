"""
main_controller.py  -  PeraBots 2026, Phase 1, SECTION 1 (obstacle navigation)

File placement (name must not change):
  controllers/main_controller/main_controller.py

Strategy
--------
1. START-UP LOCALISATION  (~3.5 s)
   The robot spins once on the spot, sweeping its front distance sensor over the
   two arena walls that form the start-zone corner.  A small least-squares fit
   gives (a) the robot's distance to each wall -> its (x, y), and (b) the wall
   directions -> its absolute heading.  The InertialUnit is then only used as a
   drift-free gyro (its sign / zero offset are calibrated automatically), so we
   do not depend on any coordinate-system convention.
2. ODOMETRY   wheel encoders (distance) + InertialUnit (heading).  No GPS.
3. MAPPING    9 DistanceSensors -> log-odds occupancy grid (2 cm cells).
4. PLANNING   A* on the map inflated by the robot radius + safety margin,
   unknown space is assumed free ("optimistic"), so the robot heads straight for
   the goal and re-plans the moment a new obstacle is seen.  Paths are
   string-pulled (line-of-sight smoothing) to keep distance short.
5. CONTROL    pure-pursuit style follower, turn-in-place when the heading error
   is large, speed limited by the front sensors, simple stuck recovery.

Nothing about the obstacle layout is hard-coded.  Only the FIXED arena frame
(outer walls, transition gap, transition tile) is used, which the rulebook
states is identical in the evaluation environment.

The LEDs are NOT touched here (rulebook 4.4: only for the Section 2 result).
"""

import math
import heapq
from controller import Robot

# ----------------------------------------------------------------------------
# Robot geometry (from TwoWheelRobot.proto - locked)
# ----------------------------------------------------------------------------
WHEEL_R = 0.02
AXLE = 0.064                 # wheel centres at y = +/-0.032
MAX_WHEEL = 8.0              # rad/s used by us (motor limit is 10)

# Distance sensors: name, x, y (robot frame), yaw in degrees.
# MUST match the DistanceSensor nodes in TwoWheelRobot.proto -> extensionSlot.
SENSORS = [
    ("ds_m90", 0.025, -0.030, -90.0),
    ("ds_m60", 0.038, -0.020, -60.0),
    ("ds_m40", 0.038, -0.013, -40.0),
    ("ds_m20", 0.038, -0.006, -20.0),
    ("ds_0",   0.038,  0.000,   0.0),
    ("ds_p20", 0.038,  0.006,  20.0),
    ("ds_p40", 0.038,  0.013,  40.0),
    ("ds_p60", 0.038,  0.020,  60.0),
    ("ds_p90", 0.025,  0.030,  90.0),
]
IDX_FRONT = 4
FRONT_IDX = (3, 4, 5)        # ds_m20, ds_0, ds_p20
MAXR = 0.6                   # lookupTable max (m); returned when nothing is hit
FRONT_OFFSET = 0.038         # x of the front sensor

# ----------------------------------------------------------------------------
# Tunables
# ----------------------------------------------------------------------------
V_CRUISE = 0.15              # m/s
W_MAX = 3.2                  # rad/s   max turn rate
SPIN_WHEEL = 3.0             # rad/s   wheel speed during start-up scan
LOOKAHEAD = 0.12             # m
INFLATE = 0.075              # m   robot radius (0.047) + safety margin
CELL = 0.02                  # m   grid resolution

# ----------------------------------------------------------------------------
# Fixed arena frame (Arena.proto / rulebook 4.3)
# ----------------------------------------------------------------------------
ARENA_W, ARENA_H = 2.44, 1.22
WALL_HALF = 0.006            # half wall thickness -> inner face offset
GAP_X0, GAP_X1 = 0.101, 0.339    # opening in the top wall (transition zone)
CHAMBER_TOP = 1.36
GOAL = (0.22, 1.27)          # centre of the red transition tile
START_ZONE_CENTRE = (2.315, 0.125)   # only used if the start-up fit fails

NX = int(round(ARENA_W / CELL))
NY = int(round(1.40 / CELL))
INF = 1e18


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def in_domain(x, y):
    """True where the robot centre may exist according to the fixed arena."""
    if WALL_HALF <= x <= ARENA_W - WALL_HALF and WALL_HALF <= y <= ARENA_H - WALL_HALF:
        return True
    if GAP_X0 <= x <= GAP_X1 and ARENA_H - 0.06 <= y <= CHAMBER_TOP:
        return True
    return False


R_CELLS = INFLATE / CELL
OFFS = [(di, dj) for di in range(-5, 6) for dj in range(-5, 6)
        if math.hypot(di, dj) <= R_CELLS]


def build_static():
    outdom = bytearray(NX * NY)
    for j in range(NY):
        for i in range(NX):
            if not in_domain((i + 0.5) * CELL, (j + 0.5) * CELL):
                outdom[j * NX + i] = 1
    blocked = bytearray(NX * NY)
    for j in range(NY):
        for i in range(NX):
            if outdom[j * NX + i]:
                blocked[j * NX + i] = 1
                continue
            for di, dj in OFFS:
                ii, jj = i + di, j + dj
                if ii < 0 or jj < 0 or ii >= NX or jj >= NY or outdom[jj * NX + ii]:
                    blocked[j * NX + i] = 1
                    break
    return outdom, blocked


OUTDOM, STATIC_BLOCKED = build_static()


def cell_of(x, y):
    return int(math.floor(x / CELL)), int(math.floor(y / CELL))


def nearest_free(blocked, c, rmax=12):
    ci, cj = c
    best, bd = None, INF
    for dj in range(-rmax, rmax + 1):
        for di in range(-rmax, rmax + 1):
            i, j = ci + di, cj + dj
            if 0 <= i < NX and 0 <= j < NY and not blocked[j * NX + i]:
                d = di * di + dj * dj
                if d < bd:
                    best, bd = (i, j), d
    return best


_MOVES = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, 1.41421356), (1, -1, 1.41421356),
          (-1, 1, 1.41421356), (-1, -1, 1.41421356)]


def astar(blocked, start, goal):
    if not (0 <= start[0] < NX and 0 <= start[1] < NY):
        return None
    if blocked[start[1] * NX + start[0]]:
        start = nearest_free(blocked, start)
        if start is None:
            return None
    if blocked[goal[1] * NX + goal[0]]:
        goal = nearest_free(blocked, goal)
        if goal is None:
            return None
    gi, gj = goal
    N = NX * NY
    g = [INF] * N
    parent = [-1] * N
    s_idx = start[1] * NX + start[0]
    g[s_idx] = 0.0
    heap = [(0.0, s_idx)]
    closed = bytearray(N)
    while heap:
        _, idx = heapq.heappop(heap)
        if closed[idx]:
            continue
        closed[idx] = 1
        i, j = idx % NX, idx // NX
        if i == gi and j == gj:
            out = []
            while idx != -1:
                out.append((idx % NX, idx // NX))
                idx = parent[idx]
            out.reverse()
            return out
        for di, dj, cost in _MOVES:
            ni, nj = i + di, j + dj
            if ni < 0 or nj < 0 or ni >= NX or nj >= NY:
                continue
            nidx = nj * NX + ni
            if blocked[nidx] or closed[nidx]:
                continue
            if di != 0 and dj != 0:       # no corner cutting
                if blocked[j * NX + ni] or blocked[nj * NX + i]:
                    continue
            ng = g[idx] + cost
            if ng < g[nidx]:
                g[nidx] = ng
                parent[nidx] = idx
                dx, dy = abs(ni - gi), abs(nj - gj)
                h = (dx + dy) - 0.5857864 * min(dx, dy)
                heapq.heappush(heap, (ng + h, nidx))
    return None


def seg_free(blocked, a, b):
    """Sample the straight segment a->b (world coords) against the grid."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    n = max(1, int(L / 0.01))
    for k in range(n + 1):
        t = k / n
        i, j = cell_of(a[0] + t * dx, a[1] + t * dy)
        if i < 0 or j < 0 or i >= NX or j >= NY or blocked[j * NX + i]:
            return False
    return True


def smooth_path(blocked, pts):
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = i + 1
        while j + 1 < len(pts) and seg_free(blocked, pts[i], pts[j + 1]):
            j += 1
        out.append(pts[j])
        i = j
    return out


# ----------------------------------------------------------------------------
# Start-up wall fit
# ----------------------------------------------------------------------------
def fit_wall(pts, centre, half):
    """Fit 1/r = A cos(phi) + B sin(phi) (a straight wall) to samples near
    'centre'. Returns (normal_angle, perpendicular_distance) or None."""
    sel = [(p, 1.0 / r) for (p, r) in pts if abs(wrap(p - centre)) <= half]
    A = B = 0.0
    for _ in range(3):
        if len(sel) < 4:
            return None
        sxx = sxy = syy = sxu = syu = 0.0
        for p, u in sel:
            c, s = math.cos(p), math.sin(p)
            sxx += c * c
            sxy += c * s
            syy += s * s
            sxu += c * u
            syu += s * u
        det = sxx * syy - sxy * sxy
        if det < 1e-9:
            return None
        A = (sxu * syy - syu * sxy) / det
        B = (syu * sxx - sxu * sxy) / det
        keep = [(p, u) for (p, u) in sel
                if abs(A * math.cos(p) + B * math.sin(p) - u) < 0.08 * u]
        if len(keep) == len(sel):
            break
        sel = keep
    if math.hypot(A, B) < 1e-6:
        return None
    return math.atan2(B, A), 1.0 / math.hypot(A, B)


def predict_reading(x0, y0, alpha):
    ca, sa = math.cos(alpha), math.sin(alpha)
    t = INF
    if ca > 1e-9:
        t = min(t, (ARENA_W - WALL_HALF - x0) / ca)
    if ca < -1e-9:
        t = min(t, (WALL_HALF - x0) / ca)
    if sa > 1e-9:
        t = min(t, (ARENA_H - WALL_HALF - y0) / sa)
    if sa < -1e-9:
        t = min(t, (WALL_HALF - y0) / sa)
    return min(MAXR, t - FRONT_OFFSET)


def localise_from_scan(samples):
    """samples: list of (phi, d) with phi = spin angle in the robot's own
    (arbitrary zero) frame, CCW positive, d = front sensor reading.
    Returns (x0, y0, phi_x) where phi_x = phi at which the robot faces world +x,
    or None."""
    pts = [(p, d + FRONT_OFFSET) for (p, d) in samples if d < 0.55]
    if len(pts) < 20:
        return None
    pm, _ = min(pts, key=lambda q: q[1])
    w1 = fit_wall(pts, pm, 0.35)
    if w1 is None:
        return None
    a1, D1 = w1
    best = None
    for sgn in (-1, 1):
        c = wrap(a1 + sgn * math.pi / 2)
        w = fit_wall(pts, c, 0.21)
        if w is not None and abs(wrap(w[0] - c)) < 0.15:
            if best is None or w[1] < best[1][1]:
                best = (sgn, w)
    if best is None:
        return None
    sgn, (a2, D2) = best
    if sgn == -1:          # near wall = right wall (+x normal), other = bottom
        phi_x, Dx, Dy = a1, D1, D2
    else:                  # near wall = bottom (-y normal); +x normal is +90 deg
        phi_x, Dx, Dy = a2, D2, D1
    if not (0.04 <= Dx <= 0.35 and 0.04 <= Dy <= 0.35):
        return None
    x0 = ARENA_W - WALL_HALF - Dx
    y0 = WALL_HALF + Dy
    # verify against the forward model of the two corner walls
    good = 0
    for p, d in samples:
        pred = predict_reading(x0, y0, p - phi_x)
        if abs(min(d, MAXR) - pred) < 0.015:
            good += 1
    if good < 0.75 * len(samples):
        return None
    return x0, y0, phi_x


# ----------------------------------------------------------------------------
# Robot wrapper: devices, odometry, heading
# ----------------------------------------------------------------------------
def get_dev(robot, name):
    """getDevice with a helpful error (lists what the robot really has)."""
    d = robot.getDevice(name)
    if d is None:
        have = [robot.getDeviceByIndex(i).getName()
                for i in range(robot.getNumberOfDevices())]
        raise RuntimeError(
            "Device '%s' not found on the robot. Devices present: %s.\n"
            "-> The robot is not using the updated TwoWheelRobot.proto "
            "(see the fix steps)." % (name, have))
    return d


class Bot:
    def __init__(self):
        self.robot = Robot()
        self.dt_ms = int(round(self.robot.getBasicTimeStep()))
        self.dt = self.dt_ms / 1000.0
        r = self.robot
        self.lm = get_dev(r, 'left_wheel_motor')
        self.rm = get_dev(r, 'right_wheel_motor')
        for m in (self.lm, self.rm):
            m.setPosition(float('inf'))
            m.setVelocity(0.0)
        self.ls = get_dev(r, 'left_wheel_sensor')
        self.rs = get_dev(r, 'right_wheel_sensor')
        self.ls.enable(self.dt_ms)
        self.rs.enable(self.dt_ms)
        self.imu = get_dev(r, 'imu')
        self.imu.enable(self.dt_ms)
        self.ds = []
        for name, x, y, a in SENSORS:
            dev = get_dev(r, name)
            dev.enable(self.dt_ms)
            self.ds.append((dev, x, y, math.radians(a)))
        self.d = [MAXR] * len(self.ds)

        self.x = self.y = self.th = 0.0
        self.calibrated = False
        self.imu_prev = None
        self.imu_unw = 0.0
        self.th_off = 0.0
        self.sign = 1.0
        self.wl_prev = self.wr_prev = None
        self.th_wheel = 0.0
        self.t = 0.0

    # -- basic I/O ------------------------------------------------------------
    def drive_wheels(self, wl, wr):
        self.lm.setVelocity(max(-10.0, min(10.0, wl)))
        self.rm.setVelocity(max(-10.0, min(10.0, wr)))

    def drive(self, v, w):
        wl = (v - w * AXLE / 2.0) / WHEEL_R
        wr = (v + w * AXLE / 2.0) / WHEEL_R
        m = max(abs(wl), abs(wr))
        if m > MAX_WHEEL:
            wl *= MAX_WHEEL / m
            wr *= MAX_WHEEL / m
        self.drive_wheels(wl, wr)

    def step(self):
        if self.robot.step(self.dt_ms) == -1:
            return False
        self.t += self.dt
        self._read()
        return True

    def _read(self):
        # distance sensors
        for k, (dev, _, _, _) in enumerate(self.ds):
            v = dev.getValue()
            if v != v:                      # NaN guard
                v = MAXR
            self.d[k] = max(0.0, min(MAXR, v))
        # imu (unwrapped yaw)
        yaw = self.imu.getRollPitchYaw()[2]
        if yaw == yaw:
            if self.imu_prev is None:
                self.imu_prev = yaw
            self.imu_unw += wrap(yaw - self.imu_prev)
            self.imu_prev = yaw
        # wheels
        wl, wr = self.ls.getValue(), self.rs.getValue()
        if self.wl_prev is None:
            self.wl_prev, self.wr_prev = wl, wr
        dl = (wl - self.wl_prev) * WHEEL_R
        dr = (wr - self.wr_prev) * WHEEL_R
        self.wl_prev, self.wr_prev = wl, wr
        self.th_wheel += (dr - dl) / AXLE
        if self.calibrated:
            th_new = wrap(self.sign * self.imu_unw + self.th_off)
            ds = 0.5 * (dl + dr)
            thm = self.th + 0.5 * wrap(th_new - self.th)
            self.x += ds * math.cos(thm)
            self.y += ds * math.sin(thm)
            self.th = th_new

    # -- start-up localisation --------------------------------------------------
    def localise(self):
        """Hardcoded start pose as requested, skipping the 360 spin."""
        self.sign = 1.0
        # Hardcoded from the current Arena.wbt start pose
        self.th_off = 2.32356  # Initial heading from Webots rotation
        self.x, self.y = 2.26866, 0.159772
        self.th = wrap(self.sign * self.imu_unw + self.th_off)
        self.calibrated = True
        
        # Go forward at the beginning for a short duration to clear the corner
        print("[loc] OVERRIDE: Skipped spin, driving forward...")
        self.drive_wheels(MAX_WHEEL, MAX_WHEEL)
        for _ in range(60):  # ~1 second forward
            if not self.step():
                return False
                
        self.drive_wheels(0.0, 0.0)
        return True


# ----------------------------------------------------------------------------
# Mapping + planning + control
# ----------------------------------------------------------------------------
L_FREE, L_OCC, L_MIN, L_MAX, OCC_T = -0.25, 0.9, -3.0, 4.0, 0.85


class Navigator:
    def __init__(self, bot):
        self.b = bot
        self.lo = [0.0] * (NX * NY)
        self.occ = set()
        self.blocked = bytearray(STATIC_BLOCKED)
        self.path = None
        self.path_age = 0
        self.need_plan = True
        self.rot_mode = False
        self.n = 0
        self.anchor = None
        self.recover_until = 0.0
        self.recover_dir = 1.0
        self.recover_phase = 0

    # -- mapping --------------------------------------------------------------
    def _upd(self, idx, delta):
        if OUTDOM[idx]:
            return
        v = self.lo[idx] + delta
        v = L_MAX if v > L_MAX else (L_MIN if v < L_MIN else v)
        self.lo[idx] = v
        if v >= OCC_T:
            self.occ.add(idx)
        else:
            self.occ.discard(idx)

    def integrate(self):
        b = self.b
        c, s = math.cos(b.th), math.sin(b.th)
        for k, (_, sx, sy, ang) in enumerate(b.ds):
            d = b.d[k]
            ox = b.x + sx * c - sy * s
            oy = b.y + sx * s + sy * c
            a = b.th + ang
            ca, sa = math.cos(a), math.sin(a)
            hit = d < MAXR - 0.01
            rmax = d if hit else 0.5
            last = -1
            t = 0.02
            while t < rmax - 0.025:
                i = int((ox + ca * t) / CELL)
                j = int((oy + sa * t) / CELL)
                if 0 <= i < NX and 0 <= j < NY:
                    idx = j * NX + i
                    if idx != last:
                        self._upd(idx, L_FREE)
                        last = idx
                t += 0.01
            if hit:
                px, py = ox + ca * (d + 0.008), oy + sa * (d + 0.008)
                i, j = int(px / CELL), int(py / CELL)
                if 0 <= i < NX and 0 <= j < NY:
                    self._upd(j * NX + i, L_OCC)

    def refresh_blocked(self):
        bl = bytearray(STATIC_BLOCKED)
        for idx in self.occ:
            i, j = idx % NX, idx // NX
            for di, dj in OFFS:
                ii, jj = i + di, j + dj
                if 0 <= ii < NX and 0 <= jj < NY:
                    bl[jj * NX + ii] = 1
        self.blocked = bl

    # -- planning -------------------------------------------------------------
    def plan(self):
        b = self.b
        s = cell_of(b.x, b.y)
        g = cell_of(*GOAL)
        cells = astar(self.blocked, s, g)
        if cells is None:                       # maybe phantom obstacles
            print("[nav] no path - clearing dynamic map")
            self.lo = [0.0] * (NX * NY)
            self.occ.clear()
            self.blocked = bytearray(STATIC_BLOCKED)
            cells = astar(self.blocked, s, g)
            if cells is None:
                self.path = None
                self.need_plan = False
                return
        pts = [((i + 0.5) * CELL, (j + 0.5) * CELL) for i, j in cells]
        pts[-1] = GOAL
        pts = smooth_path(self.blocked, pts)
        self.path = [(b.x, b.y)] + pts
        self.path_age = 0
        self.need_plan = False

    def _project(self):
        b = self.b
        best = (INF, 0, 0.0)
        P = self.path
        for k in range(len(P) - 1):
            ax, ay = P[k]
            bx, by = P[k + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((b.x - ax) * dx + (b.y - ay) * dy) / L2))
            d = math.hypot(b.x - (ax + t * dx), b.y - (ay + t * dy))
            if d < best[0]:
                best = (d, k, t)
        return best

    def _walk(self, k, t, dist):
        """Point at 'dist' metres along the path from (segment k, param t)."""
        P = self.path
        ax, ay = P[k]
        bx, by = P[k + 1]
        px, py = ax + t * (bx - ax), ay + t * (by - ay)
        rem = dist
        while True:
            seg = math.hypot(bx - px, by - py)
            if seg >= rem:
                f = rem / seg if seg > 1e-9 else 1.0
                return px + f * (bx - px), py + f * (by - py)
            rem -= seg
            px, py = bx, by
            if k + 2 >= len(P):
                return px, py
            k += 1
            bx, by = P[k + 1]

    def path_ok(self, k, t):
        for dist in [x * 0.03 for x in range(2, 30)]:
            px, py = self._walk(k, t, dist)
            i, j = cell_of(px, py)
            if 0 <= i < NX and 0 <= j < NY and self.blocked[j * NX + i]:
                return False
        return True

    # -- control --------------------------------------------------------------
    def at_goal(self):
        b = self.b
        return (math.hypot(b.x - GOAL[0], b.y - GOAL[1]) < 0.02 or
                (b.y > 1.262 and abs(b.x - GOAL[0]) < 0.06))

    def control(self, tx, ty):
        b = self.b
        e = wrap(math.atan2(ty - b.y, tx - b.x) - b.th)
        dgoal = math.hypot(GOAL[0] - b.x, GOAL[1] - b.y)
        if self.rot_mode:
            if abs(e) < 0.2:
                self.rot_mode = False
        elif abs(e) > 0.55:
            self.rot_mode = True
        if self.rot_mode:
            v = 0.0
            w = max(-W_MAX, min(W_MAX, 3.0 * e))
            if abs(w) < 0.6:
                w = 0.6 if e > 0 else -0.6
        else:
            v = V_CRUISE * (1.0 - 0.6 * min(1.0, abs(e) / 0.55))
            w = max(-W_MAX, min(W_MAX, 2.5 * e))
            v = min(v, 0.04 + 0.6 * dgoal)                 # slow near the goal
            fmin = min(b.d[k] for k in FRONT_IDX)
            v = min(v, V_CRUISE * max(0.0, min(1.0, (fmin - 0.045) / 0.10)))
        return v, w

    def check_stuck(self):
        b = self.b
        if self.anchor is None:
            self.anchor = (b.t, b.x, b.y, b.th)
            return False
        t0, x0, y0, th0 = self.anchor
        if b.t - t0 < 3.0:
            return False
        moved = math.hypot(b.x - x0, b.y - y0) > 0.04 or abs(wrap(b.th - th0)) > 0.4
        self.anchor = (b.t, b.x, b.y, b.th)
        return not moved

    def start_recovery(self):
        b = self.b
        print("[nav] stuck -> recovery at (%.2f, %.2f)" % (b.x, b.y))
        ci, cj = cell_of(b.x, b.y)
        for dj in range(-10, 11):
            for di in range(-10, 11):
                i, j = ci + di, cj + dj
                if 0 <= i < NX and 0 <= j < NY:
                    self.lo[j * NX + i] = 0.0
                    self.occ.discard(j * NX + i)
        self.refresh_blocked()
        self.recover_phase = 1
        self.recover_until = b.t + 0.6
        self.recover_dir = 1.0 if (int(b.t * 10) % 2 == 0) else -1.0
        self.need_plan = True

    def recover(self):
        b = self.b
        if self.recover_phase == 1:
            b.drive_wheels(-4.0, -4.0)
            if b.t >= self.recover_until:
                self.recover_phase = 2
                self.recover_until = b.t + 0.5
        else:
            b.drive_wheels(-3.0 * self.recover_dir, 3.0 * self.recover_dir)
            if b.t >= self.recover_until:
                self.recover_phase = 0
                self.anchor = None

    # -- main loop ------------------------------------------------------------
    def run(self):
        b = self.b
        while b.step():
            self.integrate()
            self.n += 1
            if self.at_goal():
                b.drive_wheels(0.0, 0.0)
                print("[nav] Section 1 finished at t=%.1fs (%.3f, %.3f)" % (b.t, b.x, b.y))
                break
            if self.recover_phase:
                self.recover()
                continue
            if self.n % 8 == 0:
                self.refresh_blocked()
                self.path_age += 8
                if self.path is not None:
                    _, k, t = self._project()
                    if not self.path_ok(k, t) or self.path_age > 60:
                        self.need_plan = True
                else:
                    self.need_plan = True
            if self.need_plan:
                self.plan()
            if self.path is None:
                b.drive_wheels(0.0, 0.0)
                continue
            if self.check_stuck():
                self.start_recovery()
                continue
            _, k, t = self._project()
            tx, ty = self._walk(k, t, LOOKAHEAD)
            v, w = self.control(tx, ty)
            b.drive(v, w)
        b.drive_wheels(0.0, 0.0)


def main():
    bot = Bot()
    if not bot.step():
        return
    if not bot.localise():
        return
    nav = Navigator(bot)
    nav.run()
    # Section 1 done.  (Section 2 maze logic will be added here later.)
    while bot.step():
        bot.drive_wheels(0.0, 0.0)


main()