"""
main_controller.py

PeraBots 2026 - Phase 1 - Complete Advanced SLAM
Step 4: A* Global Planning + 8-Sensor Occupancy Grid
"""

from controller import Robot
import math
import heapq
import collections

TIME_STEP = 16
MAX_WHEEL = 10.0
WHEEL_R = 0.02
ARENA_W, ARENA_H = 2.44, 1.22
GOAL = (0.17, 1.29)

CELL = 0.02
# Section 1 is 2.44 wide, Section 2 is 2.25 wide (total ~4.7m).
MIN_X, MAX_X = -3.0, 3.0
MIN_Y, MAX_Y = -1.0, 3.0
NX = int(round((MAX_X - MIN_X) / CELL))
NY = int(round((MAX_Y - MIN_Y) / CELL))
OFFSET_X = int(round(-MIN_X / CELL))
OFFSET_Y = int(round(-MIN_Y / CELL))
INF = 1e18

# 8 Sensors (Name, X_offset, Y_offset, Angle_in_degrees)
SENSORS = [
    ("ds_front", 0.042, 0, 0.0),
    ("ds_front_left", 0.0297, 0.0297, 45.0),
    ("ds_left", 0, 0.042, 90.0),
    ("ds_back_left", -0.0297, 0.0297, 135.0),
    ("ds_back", -0.042, 0, 180.0),
    ("ds_back_right", -0.0297, -0.0297, -135.0),
    ("ds_right", 0, -0.042, -90.0),
    ("ds_front_right", 0.0297, -0.0297, -45.0)
]

def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def fit_wall(pts, centre, half):
    sel = [(p, 1.0 / r) for (p, r) in pts if abs(wrap(p - centre)) <= half]
    A = B = 0.0
    for _ in range(3):
        if len(sel) < 4: return None
        sxx = sxy = syy = sxu = syu = 0.0
        for p, u in sel:
            c, s = math.cos(p), math.sin(p)
            sxx += c * c; sxy += c * s; syy += s * s; sxu += c * u; syu += s * u
        det = sxx * syy - sxy * sxy
        if det < 1e-9: return None
        A = (sxu * syy - syu * sxy) / det
        B = (syu * sxx - sxu * sxy) / det
        keep = [(p, u) for (p, u) in sel if abs(A * math.cos(p) + B * math.sin(p) - u) < 0.08 * u]
        if len(keep) == len(sel): break
        sel = keep
    if math.hypot(A, B) < 1e-6: return None
    return math.atan2(B, A), 1.0 / math.hypot(A, B)

def localise_from_scan(samples):
    pts = [(p, d + 0.042) for (p, d) in samples if d < 0.75]
    if len(pts) < 20: return None
    pm, _ = min(pts, key=lambda q: q[1])
    w1 = fit_wall(pts, pm, 0.35)
    if w1 is None: return None
    a1, D1 = w1
    best = None
    for sgn in (-1, 1):
        c = wrap(a1 + sgn * math.pi / 2)
        w = fit_wall(pts, c, 0.21)
        if w is not None and abs(wrap(w[0] - c)) < 0.15:
            if best is None or w[1] < best[1][1]: best = (sgn, w)
    if best is None: return None
    sgn, (a2, D2) = best
    if sgn == -1: phi_x, Dx, Dy = a1, D1, D2
    else: phi_x, Dx, Dy = a2, D2, D1
    return 2.44 - 0.006 - Dx, 0.006 + Dy, phi_x

class Bot:
    def __init__(self, robot):
        self.robot = robot
        self.lm = robot.getDevice('left_wheel_motor')
        self.rm = robot.getDevice('right_wheel_motor')
        self.lm.setPosition(float('inf'))
        self.rm.setPosition(float('inf'))
        
        self.ls = robot.getDevice('left_wheel_sensor')
        self.rs = robot.getDevice('right_wheel_sensor')
        self.ls.enable(TIME_STEP)
        self.rs.enable(TIME_STEP)
        
        self.imu = robot.getDevice('inertial_unit')
        self.imu.enable(TIME_STEP)
        
        self.ds = []
        for name, x, y, a in SENSORS:
            dev = robot.getDevice(name)
            dev.enable(TIME_STEP)
            self.ds.append((dev, x, y, math.radians(a)))
            
        self.d = [0.8] * 8
        self.x = self.y = self.th = 0.0
        self.wl_prev = self.wr_prev = None
        
        # SLAM Maps
        self.map = bytearray([127] * (NX * NY)) # 127=Unknown, 0=Free, 255=Obstacle
        self.path = []
        
        # State Machine & Hardware
        self.state = "SECTION_1"
        self.camera = robot.getDevice("camera")
        self.led_blue = robot.getDevice("led_blue")
        self.led_green = robot.getDevice("led_green")
        
        # Color Counting & Memory
        self.blue_count = 0
        self.green_count = 0
        self.tile_memory = []
        self.final_end_cell = None
        self.step_count = 0

    def drive_wheels(self, wl, wr):
        self.lm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wl)))
        self.rm.setVelocity(max(-MAX_WHEEL, min(MAX_WHEEL, wr)))

    def step(self):
        if self.robot.step(TIME_STEP) == -1: return False
        
        # Read Sensors
        for k, (dev, _, _, _) in enumerate(self.ds):
            v = dev.getValue()
            self.d[k] = 0.8 if v != v else max(0.0, min(0.8, v))
            
        # Update Odometry
        self.th = self.imu.getRollPitchYaw()[2]
        wl, wr = self.ls.getValue(), self.rs.getValue()
        if self.wl_prev is not None:
            ds = 0.5 * ((wl - self.wl_prev) + (wr - self.wr_prev)) * WHEEL_R
            self.x += ds * math.cos(self.th)
            self.y += ds * math.sin(self.th)
        self.wl_prev, self.wr_prev = wl, wr
        
        self.wl_prev, self.wr_prev = wl, wr
        
        if self.state == "SECTION_1":
            self.scan_match()
            self.update_map()
        
        self.step_count += 1
        return True

    def scan_match(self):
        for k, (_, _, _, ang) in enumerate(self.ds):
            d = self.d[k]
            if d >= 0.7: continue
            
            abs_angle = wrap(self.th + ang)
            
            # Smart Scan Matching using cosine projection for slight angles
            # If pointing roughly Right (+X)
            if abs(wrap(abs_angle - 0.0)) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - 0.0))
                pred_x = ARENA_W - 0.006 - perp_d - 0.042
                if abs(pred_x - self.x) < 0.05: 
                    self.x = 0.90 * self.x + 0.10 * pred_x
            
            # Roughly Top (+Y)
            elif abs(wrap(abs_angle - (math.pi/2))) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - (math.pi/2)))
                pred_y = ARENA_H - 0.006 - perp_d - 0.042
                if abs(pred_y - self.y) < 0.05: 
                    self.y = 0.90 * self.y + 0.10 * pred_y
                    
            # Roughly Left (-X)
            elif abs(wrap(abs_angle - math.pi)) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - math.pi))
                pred_x = 0.006 + perp_d + 0.042
                if abs(pred_x - self.x) < 0.05:
                    self.x = 0.90 * self.x + 0.10 * pred_x
                    
            # Roughly Bottom (-Y)
            elif abs(wrap(abs_angle - (-math.pi/2))) < 0.2:
                perp_d = d * math.cos(wrap(abs_angle - (-math.pi/2)))
                pred_y = 0.006 + perp_d + 0.042
                if abs(pred_y - self.y) < 0.05:
                    self.y = 0.90 * self.y + 0.10 * pred_y

    def update_map(self):
        cx = int(self.x / CELL) + OFFSET_X
        cy = int(self.y / CELL) + OFFSET_Y
        
        # 1. Sweep a solid physical footprint of contiguous Free Space (radius = 5 cells = 10cm)
        # This prevents the BFS from being choked by thin raycast lines and inflation!
        for dx in range(-5, 6):
            for dy in range(-5, 6):
                if dx*dx + dy*dy <= 25:
                    ix, iy = cx+dx, cy+dy
                    if 0 <= ix < NX and 0 <= iy < NY:
                        if self.map[iy * NX + ix] == 127:
                            self.map[iy * NX + ix] = 0
                            
        # 2. Sensor Raycasting
        for k, (_, sx, sy, ang) in enumerate(self.ds):
            d = self.d[k]
            abs_a = wrap(self.th + ang)
            
            # Max range for raycast clearing
            clear_d = d if d < 0.75 else 0.75
            
            # Bresenham to clear free space
            px_clear = self.x + clear_d * math.cos(abs_a)
            py_clear = self.y + clear_d * math.sin(abs_a)
            
            x0, y0 = cx, cy
            x1 = int(px_clear / CELL) + OFFSET_X
            y1 = int(py_clear / CELL) + OFFSET_Y
            
            dx = abs(x1 - x0)
            dy = abs(y1 - y0)
            sx_step = 1 if x0 < x1 else -1
            sy_step = 1 if y0 < y1 else -1
            err = dx - dy
            
            while True:
                if 0 <= x0 < NX and 0 <= y0 < NY:
                    # Clear free space ONLY if it's currently Unknown (don't erase walls!)
                    if self.map[y0 * NX + x0] == 127:
                        self.map[y0 * NX + x0] = 0 
                if x0 == x1 and y0 == y1:
                    break
                e2 = 2 * err
                if e2 > -dy:
                    err -= dy
                    x0 += sx_step
                if e2 < dx:
                    err += dx
                    y0 += sy_step
                    
            # Mark Obstacle
            if d < 0.75:
                px = self.x + (d + 0.042) * math.cos(abs_a)
                py = self.y + (d + 0.042) * math.sin(abs_a)
                ix = int(px / CELL) + OFFSET_X
                iy = int(py / CELL) + OFFSET_Y
                if 0 <= ix < NX and 0 <= iy < NY:
                    self.map[iy * NX + ix] = 255

    def plan_to_frontier(self):
        """BFS to find the closest reachable frontier cell."""
        cx = int(self.x / CELL) + OFFSET_X
        cy = int(self.y / CELL) + OFFSET_Y
        if not (0 <= cx < NX and 0 <= cy < NY): return
        
        queue = collections.deque([((cx, cy), [])])
        visited = bytearray(NX * NY)
        visited[cy * NX + cx] = 1
        
        while queue:
            (px, py), path = queue.popleft()
            
            # Is it a frontier? (Free space bordering Unknown)
            is_frontier = False
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = px+dx, py+dy
                if 0 <= nx < NX and 0 <= ny < NY:
                    if self.map[ny * NX + nx] == 127:
                        is_frontier = True
                        break
                        
            if is_frontier and len(path) > 2:
                # Target acquired! Convert grid path to real-world coordinates
                self.path = [((ix - OFFSET_X + 0.5)*CELL, (iy - OFFSET_Y + 0.5)*CELL) for ix, iy in path]
                return
                
            # Expand neighbors
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = px+dx, py+dy
                if 0 <= nx < NX and 0 <= ny < NY:
                    nidx = ny * NX + nx
                    if not visited[nidx]:
                        visited[nidx] = 1
                        if self.map[nidx] == 0: # Only route through known Free space
                            
                            # Apply Obstacle Inflation check!
                            blocked = False
                            for idx in [-2, -1, 0, 1, 2]:
                                for idy in [-2, -1, 0, 1, 2]:
                                    if idx*idx + idy*idy <= 5: # 4cm circular clearance
                                        ix, iy = nx+idx, ny+idy
                                        if 0 <= ix < NX and 0 <= iy < NY:
                                            if self.map[iy * NX + ix] == 255:
                                                blocked = True
                                                break
                                if blocked: break
                                
                            if not blocked:
                                queue.append(((nx, ny), path + [(nx, ny)]))
                                
        print("[EXPLORE] MAZE FULLY MAPPED! No frontiers left.")
        self.path = []
                
    def a_star(self):
        """Find fastest path to GOAL avoiding self.map obstacles."""
        start = (int(self.x / CELL) + OFFSET_X, int(self.y / CELL) + OFFSET_Y)
        goal = (int(GOAL[0] / CELL) + OFFSET_X, int(GOAL[1] / CELL) + OFFSET_Y)
        
        # Cap to array bounds
        if not (0 <= start[0] < NX and 0 <= start[1] < NY): return
        
        N = NX * NY
        g = [INF] * N
        parent = [-1] * N
        s_idx = start[1] * NX + start[0]
        g[s_idx] = 0.0
        heap = [(0.0, s_idx)]
        closed = bytearray(N)
        
        moves = [(1,0,1.0), (-1,0,1.0), (0,1,1.0), (0,-1,1.0), 
                 (1,1,1.414), (1,-1,1.414), (-1,1,1.414), (-1,-1,1.414)]
                 
        while heap:
            _, idx = heapq.heappop(heap)
            if closed[idx]: continue
            closed[idx] = 1
            i, j = idx % NX, idx // NX
            
            if i == goal[0] and j == goal[1]:
                out = []
                while idx != -1:
                    out.append((idx % NX, idx // NX))
                    idx = parent[idx]
                out.reverse()
                self.path = [((ix - OFFSET_X + 0.5)*CELL, (iy - OFFSET_Y + 0.5)*CELL) for ix, iy in out]
                return
                
            for di, dj, cost in moves:
                ni, nj = i + di, j + dj
                if ni < 0 or nj < 0 or ni >= NX or nj >= NY: continue
                
                # Check obstacle + circular inflation (radius = 2 cells = 4cm clearance)
                blocked = False
                for dx in [-2, -1, 0, 1, 2]:
                    for dy in [-2, -1, 0, 1, 2]:
                        if dx*dx + dy*dy <= 5: # Circular footprint radius ~2.2
                            if 0 <= ni+dx < NX and 0 <= nj+dy < NY:
                                if self.map[(nj+dy) * NX + (ni+dx)] == 255:
                                    blocked = True
                
                nidx = nj * NX + ni
                if blocked or closed[nidx]: continue
                
                ng = g[idx] + cost
                if ng < g[nidx]:
                    g[nidx] = ng
                    parent[nidx] = idx
                    # Heuristic
                    h = math.hypot(ni - goal[0], nj - goal[1])
                    heapq.heappush(heap, (ng + h, nidx))
                    
        print(f"[A* ERROR] Queue emptied. No path found to GOAL {goal} from START {start}!")
        self.path = []

    def navigate(self):
        """Pure Pursuit Follower to drive along self.path."""
        if not self.path or len(self.path) < 2:
            print("[NAVIGATE] Reached the target cell and navigation stopped. Motors OFF.")
            self.drive_wheels(0, 0)
            return
            
        # Find closest point on path to current robot position
        min_dist = INF
        closest_idx = 0
        for i, p in enumerate(self.path):
            d = math.hypot(p[0] - self.x, p[1] - self.y)
            if d < min_dist:
                min_dist = d
                closest_idx = i
                
        # Lookahead 10 nodes (20cm) to prevent corner cutting
        lookahead = min(closest_idx + 10, len(self.path) - 1)
        target = self.path[lookahead]
        
        # Calculate heading error
        target_angle = math.atan2(target[1] - self.y, target[0] - self.x)
        e = wrap(target_angle - self.th)
        
        # Drive control - Smoother!
        if abs(e) > 0.6: # Sharp turn: spin in place
            v_l = MAX_WHEEL * 0.5 if e < 0 else -MAX_WHEEL * 0.5
            v_r = -MAX_WHEEL * 0.5 if e < 0 else MAX_WHEEL * 0.5
        else: # Smooth proportional curve
            speed = MAX_WHEEL * 0.8  # slower base speed
            turn = e * MAX_WHEEL * 0.8  # much gentler steering gain
            
            # Repulsive Steering: Push away from close corners to prevent clipping!
            if self.d[1] < 0.12: # Front-Left sees corner
                turn -= (0.12 - self.d[1]) * MAX_WHEEL * 3.0
            if self.d[7] < 0.12: # Front-Right sees corner
                turn += (0.12 - self.d[7]) * MAX_WHEEL * 3.0
            
            # Slow down if obstacle is close in front
            if min(self.d[0], self.d[1], self.d[7]) < 0.2:
                speed *= 0.5
                
            v_l = speed - turn
            v_r = speed + turn
        self.drive_wheels(v_l, v_r)
def main():
    global GOAL
    robot = Robot()
    bot = Bot(robot)
    
    print("--- Starting Auto-Calibration Spin ---")
    bot.drive_wheels(MAX_WHEEL, -MAX_WHEEL)
    
    samples = []
    for _ in range(220):
        if not bot.step(): return
        samples.append((bot.th, bot.d[0]))
        
    bot.drive_wheels(0, 0)
    
    result = localise_from_scan(samples)
    if result:
        bot.x, bot.y, _ = result
        print(f"Calibrated: X={bot.x:.3f}, Y={bot.y:.3f}")
    else:
        print("Calibration failed!")
        return

    # Clear map garbage collected during the spin when coordinates were 0,0
    bot.map = bytearray([127] * (NX * NY))
    print("--- Navigating Maze to Goal ---")
    
    plan_counter = 0
    bot.a_star() # Force A* immediately on step 0!
    
    while bot.step():
        if bot.state == "SECTION_1":
            plan_counter += 1
            if plan_counter % 30 == 0:
                bot.a_star()
            bot.navigate()
            
            dist_to_goal = math.hypot(bot.x - GOAL[0], bot.y - GOAL[1])
            if dist_to_goal < 0.05:
                print("[NAVIGATE] Reached the target cell and navigation stopped. Motors OFF.")
                bot.drive_wheels(0, 0)
                bot.state = "TRANSITION"
                
        elif bot.state == "TRANSITION":
            print("--- ENTERING SECTION 2 MAZE & STARTING DFS ---")
            bot.camera.enable(TIME_STEP)
            
            # Maze Column 0 is at X = 0.22m, Row 0 is at Y = 1.455m
            bot.maze_start_x = 0.22
            bot.maze_start_y = 1.455
            
            # Step forward into Cell (0, 0) from the transition chamber
            dist_to_entry = 1.455 - bot.y
            if dist_to_entry > 0.02:
                bot.drive_wheels(MAX_WHEEL * 0.5, MAX_WHEEL * 0.5)
            else:
                bot.drive_wheels(0, 0)
                bot.x = 0.22
                bot.y = 1.455
                bot.th = math.pi / 2 # Facing North
                
                # DFS grid dictionary: (cx, cy) -> {'visited': True, 'walls': {'N':True, 'S':False, ...}}
                bot.dfs_grid = {(0,0): {'visited': True, 'scanned': False, 'walls': {}}}
                bot.dfs_stack = [] # Empty initially! Current position is (cx, cy)
                
                bot.cx = 0
                bot.cy = 0
                bot.target_cx = None
                bot.target_cy = None
                bot.is_finishing = False
                
                bot.state = "DFS_SCAN"
            
        elif bot.state == "DFS_SCAN":
            # 0. Physical Odometry Correction
            # Snap to perfect grid mathematically
            true_cx = bot.maze_start_x + bot.cx * 0.25
            true_cy = bot.maze_start_y + bot.cy * 0.25
            bot.x = true_cx
            bot.y = true_cy
            
            # If there is a wall in front, use it to physically perfect the longitudinal axis!
            if bot.d[0] < 0.20:
                dist_to_wall = bot.d[0] + 0.042
                if abs(wrap(bot.th - 0.0)) < 0.5: bot.x = (true_cx + 0.125) - dist_to_wall
                elif abs(wrap(bot.th - math.pi/2)) < 0.5: bot.y = (true_cy + 0.125) - dist_to_wall
                elif abs(wrap(bot.th - math.pi)) < 0.5: bot.x = (true_cx - 0.125) + dist_to_wall
                elif abs(wrap(bot.th + math.pi/2)) < 0.5: bot.y = (true_cy - 0.125) + dist_to_wall
                
            # 1. Check Camera for Color Tiles
            img = bot.camera.getImageArray()
            if img:
                w, h = bot.camera.getWidth(), bot.camera.getHeight()
                r, g, b = img[w // 2][h // 2]
                detected_color = None
                if b > 40 and b > r * 1.5 and b > g * 1.5: detected_color = "BLUE"
                elif g > 40 and g > r * 1.5 and g > b * 1.5: detected_color = "GREEN"
                elif r > 40 and r > g * 1.5 and r > b * 1.5: detected_color = "RED"
                
            if detected_color:
                cell_coord = (bot.cx, bot.cy)
                if cell_coord not in bot.tile_memory:
                    bot.tile_memory.append(cell_coord)
                    if detected_color == "BLUE":
                        bot.blue_count += 1
                        print(f"[CAMERA] Found BLUE tile at ({bot.cx}, {bot.cy})! Total Blue: {bot.blue_count}")
                    elif detected_color == "GREEN":
                        bot.green_count += 1
                        print(f"[CAMERA] Found GREEN tile at ({bot.cx}, {bot.cy})! Total Green: {bot.green_count}")
                    elif detected_color == "RED":
                        if bot.cx > 0 or bot.cy > 0:
                            bot.final_end_cell = (bot.cx, bot.cy)
                            print(f"[CAMERA] Memorized FINAL RED TILE at discrete {bot.final_end_cell}!")
                            
            # 2. Map Walls (ONLY if entering an unmapped cell for the first time!)
            cell = bot.dfs_grid.setdefault((bot.cx, bot.cy), {'visited': True, 'scanned': False, 'walls': {}})
            cell['visited'] = True
            
            dir_to_offset = {"N": (0,1), "S": (0,-1), "E": (1,0), "W": (-1,0)}
            opp = {"N": "S", "S": "N", "E": "W", "W": "E"}
            
            if not cell.get('scanned', False):
                def angle_to_dir(a):
                    a = wrap(a)
                    if abs(a) < 0.78: return "E"
                    if abs(wrap(a - math.pi/2)) < 0.78: return "N"
                    if abs(wrap(a - math.pi)) < 0.78: return "W"
                    if abs(wrap(a + math.pi/2)) < 0.78: return "S"
                    return "E"
                
                f_dir = angle_to_dir(bot.th)
                l_dir = angle_to_dir(bot.th + math.pi/2)
                r_dir = angle_to_dir(bot.th - math.pi/2)
                b_dir = angle_to_dir(bot.th + math.pi)
                
                # Check sensor readings: A true perpendicular wall reads ~0.075m - 0.083m.
                # Threshold of 0.15m cleanly distinguishes walls (<0.15m) from open corridors (>0.30m).
                cell['walls'][f_dir] = bot.d[0] < 0.15
                cell['walls'][l_dir] = bot.d[2] < 0.15
                cell['walls'][r_dir] = bot.d[6] < 0.15
                cell['walls'][b_dir] = False # The cell we drove from is open
                
                # Physical Arena / Maze Grid Boundary Limits:
                # Columns: cx in [0, 8], Rows: cy in [0, 3]
                for d_str, (dx, dy) in dir_to_offset.items():
                    nx, ny = bot.cx + dx, bot.cy + dy
                    if not (0 <= nx <= 8 and 0 <= ny <= 3):
                        cell['walls'][d_str] = True  # Hard outer maze boundary wall!
                
                # Transition door at (0, 0): Never allow robot to turn South back into Section 1!
                if bot.cx == 0 and bot.cy == 0:
                    cell['walls']["S"] = True
                
                # Bidirectional consistency with neighbors
                for d_str, is_wall in cell['walls'].items():
                    if not is_wall:
                        dx, dy = dir_to_offset[d_str]
                        nx, ny = bot.cx + dx, bot.cy + dy
                        if 0 <= nx <= 8 and 0 <= ny <= 3:
                            nbr = bot.dfs_grid.setdefault((nx, ny), {'visited': False, 'scanned': False, 'walls': {}})
                            nbr['walls'][opp[d_str]] = False
                
                cell['scanned'] = True
            
            # 3. Pick Next Cell
            # Priority Order: North ("N"), South ("S"), West ("W"), East ("E")
            unvisited = []
            for d_str in ["N", "S", "W", "E"]:
                if not cell['walls'].get(d_str, True):
                    dx, dy = dir_to_offset[d_str]
                    nx, ny = bot.cx + dx, bot.cy + dy
                    if 0 <= nx <= 8 and 0 <= ny <= 3:
                        nbr = bot.dfs_grid.get((nx, ny))
                        if not nbr or not nbr.get('visited', False):
                            unvisited.append(((nx, ny), d_str))
                        
            unvisited_cells = [u[0] for u in unvisited]
            if unvisited:
                # Explore forward
                next_cell, direction = unvisited[0]
                bot.dfs_stack.append((bot.cx, bot.cy))
                bot.target_cx, bot.target_cy = next_cell
                bot.dfs_grid.setdefault(next_cell, {'visited': True, 'scanned': False, 'walls': {}})['visited'] = True
                print(f"[NAV] Current: ({bot.cx}, {bot.cy}) | Unvisited: {unvisited_cells} | Next: {next_cell} ({direction})")
                bot.state = "DFS_TURN"
            else:
                # Backtrack
                if not bot.dfs_stack:
                    print("DFS COMPLETE!")
                    bot.state = "FINISH"
                else:
                    bot.target_cx, bot.target_cy = bot.dfs_stack.pop()
                    print(f"[NAV] Current: ({bot.cx}, {bot.cy}) | Unvisited: [] | Backtrack to: {(bot.target_cx, bot.target_cy)}")
                    bot.state = "DFS_TURN"
                    
        elif bot.state == "DFS_TURN":
            # Orient towards target
            dx = bot.target_cx - bot.cx
            dy = bot.target_cy - bot.cy
            target_angle = math.atan2(dy, dx)
            e = wrap(target_angle - bot.th)
            
            if abs(e) > 0.05:
                speed = MAX_WHEEL * 0.4  # SAFER TURN
                bot.drive_wheels(-speed if e > 0 else speed, speed if e > 0 else -speed)
            else:
                bot.drive_wheels(0, 0)
                bot.state = "DFS_FORWARD"
                
        elif bot.state == "DFS_FORWARD":
            dx = bot.target_cx - bot.cx
            dy = bot.target_cy - bot.cy
            target_angle = math.atan2(dy, dx)

            target_x = bot.maze_start_x + bot.target_cx * 0.25
            target_y = bot.maze_start_y + bot.target_cy * 0.25
            
            # Progress along movement direction (immune to cross-track lateral drift)
            remaining = (target_x - bot.x) * dx + (target_y - bot.y) * dy
            
            # Emergency Frontal Crash Stop (Odometry Drift Protection)
            reached = (remaining <= 0.02) or (bot.d[0] < 0.06)
                
            if not reached:
                speed = MAX_WHEEL * 0.6  # Controlled forward drive
                
                # 1. Primary Heading Lock using IMU (guarantees robot stays on straight cardinal line)
                heading_err = wrap(target_angle - bot.th)
                turn = heading_err * 3.0  # Proportional heading correction
                
                # 2. Secondary Corridor Centering: ONLY active when bounded by BOTH parallel walls
                # This prevents the robot from ever falsely veering towards side openings or intersections
                left_wall = bot.d[2] < 0.16
                right_wall = bot.d[6] < 0.16
                if left_wall and right_wall:
                    wall_err = bot.d[2] - bot.d[6]
                    turn += wall_err * 1.5  # Gentle centering nudge
                    
                # Clamp turn to prevent sudden jerking
                turn = max(min(turn, 2.5), -2.5)
                bot.drive_wheels(speed - turn, speed + turn)
            else:
                bot.drive_wheels(0, 0)
                bot.x = target_x
                bot.y = target_y
                bot.cx, bot.cy = bot.target_cx, bot.target_cy
                bot.state = "FINISH_NAV" if bot.is_finishing else "DFS_SCAN"
                
        elif bot.state == "FINISH":
            if not bot.final_end_cell:
                print("[ERROR] Explored whole maze but never found the final Red tile!")
                bot.drive_wheels(0, 0)
                break
                
            # BFS to find shortest path to final_end_cell
            queue = collections.deque([((bot.cx, bot.cy), [])])
            visited = set()
            bot.discrete_path = []
            while queue:
                curr, path = queue.popleft()
                if curr == bot.final_end_cell:
                    bot.discrete_path = path
                    break
                if curr in visited: continue
                visited.add(curr)
                
                cell_walls = bot.dfs_grid.get(curr, {}).get('walls', {})
                for d_str, (dx, dy) in {"N":(0,1), "S":(0,-1), "E":(1,0), "W":(-1,0)}.items():
                    if not cell_walls.get(d_str, True):
                        queue.append(((curr[0]+dx, curr[1]+dy), path + [(curr[0]+dx, curr[1]+dy)]))
            
            if not bot.discrete_path:
                print("[ERROR] Cannot find path to end cell!")
                bot.drive_wheels(0, 0)
                break
                
            bot.is_finishing = True
            bot.state = "FINISH_NAV"
            
        elif bot.state == "FINISH_NAV":
            if not bot.discrete_path:
                bot.drive_wheels(0, 0)
                print("=== MAZE COMPLETED! ===")
                print(f"Final Count -> Blue: {bot.blue_count}, Green: {bot.green_count}")
                if bot.blue_count > bot.green_count: bot.led_blue.set(1)
                elif bot.green_count > bot.blue_count: bot.led_green.set(1)
                else: bot.led_blue.set(1); bot.led_green.set(1)
                break
                
            bot.target_cx, bot.target_cy = bot.discrete_path.pop(0)
            bot.state = "DFS_TURN"

if __name__ == "__main__":
    main()
