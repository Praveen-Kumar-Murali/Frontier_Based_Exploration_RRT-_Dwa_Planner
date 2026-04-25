# Phase 1 Navigation Stack — Complete Project Notes
### TurtleBot4 + Stonefish Simulator + ROS2 Jazzy
**Last updated: April 2026**

---

## 1. Overview

We built a full autonomous navigation stack for the TurtleBot4 robot, working in the Stonefish physics simulator and deployable on the real robot. The stack has three nodes working together:

```
LiDAR scans → mapping_node → /projected_map
                                    ↓
goal clicked → global_planner → /plan (RRT* path)
                                    ↓
                              dwa_planner → /cmd_vel (velocity commands)
```

---

## 2. File Structure

### Package: `handson_planning`
```
~/ros2_ws/src/handson_planning/
├── handson_planning/
│   ├── mapping_node.py         ← Probabilistic occupancy grid mapper
│   ├── global_planner.py       ← RRT* path planner + online replanning
│   └── dwa_planner.py          ← DWA local planner + visualization
├── launch/
│   ├── phase1_sim.launch.py    ← Stonefish simulator launch
│   └── phase1_real.launch.py   ← Real TurtleBot4 launch
├── navigation_stack_explained.html   ← Visual flowchart guide (for professor demo)
└── parameters_guide.html             ← All 25 parameters reference (3-column: too low / current / too high)
```

### Package: `online_motion_planning` (reference — proven real-robot code)
```
~/ros2_ws/src/online_motion_planning/
├── online_motion_planning/
│   ├── mapping_node.py         ← Same mapper, tested on real robot
│   ├── planning_node.py        ← RRT* planner (original version)
│   ├── control_tb.py           ← Simple waypoint controller (TwistStamped, backtrack recovery)
│   ├── localisation_node.py    ← EKF for odometry fusion
│   └── map_saver.py            ← Save/load maps as .npy files
└── launch/
    └── full_system.launch.py   ← Full 4-phase launch (sim/real × saved/live map)
```

---

## 3. How to Run

### Simulator (Stonefish)
```bash
# Terminal 1 — start simulator
cd ~/ros2_ws && source install/setup.bash
ros2 launch stonefish_simulator turtlebot4_world.launch.py   # (your sim launch)

# Terminal 2 — start navigation stack
ros2 launch handson_planning phase1_sim.launch.py

# RViz2 — add these displays:
#   Map         → /projected_map
#   Path        → /plan              (red — global RRT* path)
#   Path        → /local_path        (cyan — DWA 3s arc)
#   MarkerArray → /rrt_tree          (grey — RRT* exploration tree)
#   MarkerArray → /dwa_trajectories  (blue fan + green best)
# Then click "2D Goal Pose" to send a goal
```

### Real Robot
```bash
# Uses online_motion_planning nodes (proven on real hardware)
ros2 launch handson_planning phase1_real.launch.py

# With EKF localisation (better accuracy):
ros2 launch handson_planning phase1_real.launch.py use_ekf:=true

# Check topics before launching:
ros2 topic list | grep -E '(scan|odom|cmd_vel)'
# Expected: /odom   /scan   /cmd_vel
```

---

## 4. Node Details

### 4.1 mapping_node.py

**What it does:** Builds a probabilistic occupancy grid in real time from LiDAR scans using a log-odds sensor model.

**Key concepts:**
- Each cell stores a log-odds value (positive = probably wall, negative = probably free)
- **Bresenham ray casting**: traces each LiDAR beam through cells, marking intermediate cells as free and the endpoint as occupied
- **Sticky walls**: cells with log-odds > 2.0 cannot be erased by free-ray updates (prevents wall flickering)
- **TF2 lookup**: gets exact LiDAR world position at scan timestamp. Falls back to odometry + fixed offset if TF not ready
- `invert_scan_angles=True` for simulator: Stonefish mounts LiDAR upside-down (roll=180°), so beam angle = `lidar_yaw − scan_angle` instead of `lidar_yaw + scan_angle`

**Critical parameters:**
| Parameter | Sim value | Real value | Effect |
|-----------|-----------|------------|--------|
| `l_occ` | 0.85 | 0.85 | How fast walls are confirmed |
| `l_free` | 0.20 | 0.30 | How fast free space is cleared |
| `lidar_yaw` | 3.1416 (π) | 0.0 | Manual yaw offset (sim needs π, real uses TF) |
| `invert_scan_angles` | True | False | Beam direction convention |
| `map_frame` | `world_enu` | `odom` | Which TF frame the map lives in |

**`to_ros_array()` thresholds:**
- `log-odds > 1.0` → publish as 100 (occupied) — needs several hits
- `log-odds < -0.2` → publish as 0 (free)
- Between → publish as -1 (unknown)

**Topics:**
- In: `/turtlebot/scan` (sim) or `/scan` (real)
- In: `/turtlebot/odom` (sim) or `/odom` (real)
- Out: `/projected_map` (OccupancyGrid, published at 2 Hz)

---

### 4.2 global_planner.py

**What it does:** Receives a goal from RViz, runs RRT* to find a path, publishes it as `/plan`, and monitors the path for obstacles.

**RRT* algorithm:**
1. Sample random point in map (15% chance: sample goal directly — "goal bias")
2. Find nearest existing tree node
3. Extend toward sampled point by `step_size` (0.3m)
4. Check segment for collisions in inflated map
5. Rewire: if new node can give a cheaper path to nearby nodes, update their parent
6. Repeat 5000 times or until goal is reached

**Map inflation:** All wall cells expanded by `robot_radius` before planning. This converts the robot to a point and obstacles to C-space obstacles — any path through inflated map has at least `robot_radius` clearance.

**Online replanning (FIXED — key improvement):**
- Timer fires every 1 second but does NOT replan on a schedule
- Returns immediately if path looks clear
- Only replans if a waypoint in the next 3 waypoints ahead is blocked in the inflated map
- **5-second cooldown** between replans prevents rapid-fire RRT* triggered by map noise
- `current_path` is NOT cleared until a new plan arrives — DWA keeps moving during replanning
- `_last_replan_t` tracks the last replan timestamp (monotonic clock)

**`path_index` advancement:** As robot passes waypoints (within `waypoint_tol`), `path_index` advances in `_odom_cb`. This prevents checking already-traversed segments which may now show walls behind the robot.

**Emergency replan via DWA:** DWA publishes `Int32(1)` on `/replan_request` when all trajectories are blocked. Global planner responds but respects the same 5-second cooldown. Note: uses `Int32` not `Bool` — ROS2 Jazzy has a Bool serialization bug.

**Topics:**
- In: `/turtlebot/odom` or `/odom`
- In: `/projected_map`
- In: `/goal_pose` (from RViz "2D Goal Pose")
- In: `/replan_request` (Int32 from DWA)
- Out: `/plan` (nav_msgs/Path)
- Out: `/rrt_tree` (MarkerArray — grey tree visualization)

---

### 4.3 dwa_planner.py

**What it does:** Executes the global path safely by sampling many short trajectories every 100ms and picking the best one that avoids obstacles.

**DWA sampling:**
- 8 linear velocity samples: `0 → v_max` (0.18 m/s)
- 21 angular velocity samples: `-w_max → +w_max` (±1.2 rad/s)
- = 168 candidate trajectories per cycle
- Each simulated for 3.0 seconds, 20 steps (0.15s per step)

**Three-phase control (FIXED):**

| Phase | Trigger | Action |
|-------|---------|--------|
| **A — Pure Rotate** | `\|angle_error\| > 0.50 rad (29°)` | P-controller: `w = k_w × angle_error`, `v = 0` |
| **B — DWA Forward** | angle_error ≤ 29°, trajectories exist | Execute best scored trajectory |
| **C — Recovery** | All 168 trajectories blocked | Rotate in place, publish `/replan_request` after 50 ticks (5s cooldown) |

**Key fix — reduced excessive rotation:** `angle_threshold` raised from 0.20 rad (11°) to 0.50 rad (29°). DWA now handles small heading errors while moving — robot curves toward waypoints instead of stopping to spin. `k_w` increased to 2.0 so Phase A completes faster when it does trigger.

**DWA scoring formula:**
```
score = 3.0 × heading + 0.5 × clearance + 0.3 × velocity
```
- `heading [0–1]`: endpoint direction toward current waypoint
- `clearance [0–1]`: min distance to nearest wall along trajectory (saturates at 0.6m)
- `velocity [0–1]`: total distance covered (forward progress)

**Collision check — step-0 skip:** The first simulation step is not collision-checked because the robot may legitimately be adjacent to an inflated wall cell. Only steps 1 onward are checked.

**Visualization topics (for RViz):**
- `/dwa_trajectories` — MarkerArray: blue/grey fan = all 168 trajectories, green = best
- `/local_path` — nav_msgs/Path: best 3s DWA arc (compare with red `/plan`)

**`_map_frame` fix:** Markers use `self._map_frame` which is automatically updated from the map message header (`_map_cb` sets `self._map_frame = msg.header.frame_id`). Was hardcoded to `'odom'` which caused "Status: Error" in RViz when map frame is `world_enu`.

**Topics:**
- In: `/turtlebot/odom` or `/odom`
- In: `/plan`
- In: `/projected_map`
- Out: `/turtlebot/cmd_vel` (Twist, sim) or `/cmd_vel` (TwistStamped, real)
- Out: `/replan_request` (Int32)
- Out: `/dwa_trajectories` (MarkerArray)
- Out: `/local_path` (Path)

---

## 5. All Parameters Reference

### mapping_node
| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `use_sim` | True | False | Topic name selector |
| `map_frame` | `world_enu` | `odom` | TF frame for published map |
| `grid_size` | 20.0 | 20.0 | Map coverage (m), square |
| `grid_resolution` | 0.05 | 0.05 | Cell size (m/cell) |
| `l_occ` | 0.85 | 0.85 | Log-odds increase per hit |
| `l_free` | 0.20 | 0.30 | Log-odds decrease per free ray |
| `lidar_yaw` | 3.1416 | 0.0 | LiDAR yaw offset (rad) |
| `invert_scan_angles` | True | False | Beam direction convention |

### global_planner
| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `robot_radius` | 0.16 | 0.18 | Inflation radius = safety margin (m) |
| `rrt_max_iter` | 5000 | 5000 | Max RRT* iterations |
| `rrt_step_size` | 0.3 | 0.3 | Max extension per iteration (m) |
| `replan_interval` | 1.0 | 1.0 | Timer period (s) — only triggers replan if blocked |
| `waypoint_tol` | 0.25 | 0.30 | Waypoint reached threshold (m) |

### dwa_planner
| Parameter | Sim | Real | Description |
|-----------|-----|------|-------------|
| `v_max` | 0.18 | 0.15 | Max linear speed (m/s) |
| `w_max` | 1.2 | 1.0 | Max angular speed (rad/s) |
| `a_w_max` | 1.5 | 1.5 | Max angular acceleration (rad/s²) |
| `sim_time` | 3.0 | 2.5 | Trajectory lookahead duration (s) |
| `sim_steps` | 20 | 20 | Steps per simulated trajectory |
| `waypoint_tol` | 0.25 | 0.30 | Waypoint reached threshold (m) |
| `w_heading` | 3.0 | 3.0 | Heading score weight |
| `w_clearance` | 0.5 | 0.8 | Clearance score weight (higher for real) |
| `w_velocity` | 0.3 | 0.2 | Velocity score weight |
| `k_w` | 2.0 | 2.0 | Phase A rotation P-gain |
| `angle_threshold` | 0.50 | 0.50 | Phase A trigger angle (rad, ~29°) |
| `dt` | 0.1 | 0.1 | Control period (s) = 10 Hz |

---

## 6. Topic Map (Sim)

```
/turtlebot/scan ──────────────────→ mapping_node
/turtlebot/odom ──┬───────────────→ mapping_node
                  ├───────────────→ global_planner
                  └───────────────→ dwa_planner

mapping_node ─────/projected_map──→ global_planner
                                 └→ dwa_planner

/goal_pose ───────────────────────→ global_planner
global_planner ───/plan───────────→ dwa_planner
global_planner ───/rrt_tree───────→ RViz

dwa_planner ──────/turtlebot/cmd_vel → Robot
dwa_planner ──────/replan_request──→ global_planner
dwa_planner ──────/dwa_trajectories→ RViz
dwa_planner ──────/local_path ─────→ RViz
```

---

## 7. Key Bugs Fixed (History)

| Bug | Root Cause | Fix |
|-----|-----------|-----|
| MarkerArray "Status: Error" in RViz | `frame_id='odom'` hardcoded, but map uses `world_enu` | `_map_frame` auto-updated from map msg header |
| DWA trajectories not visible | `_publish_trajectories` only called in Phase B | Always run DWA sampling + publish BEFORE phase decision |
| DWA trajectory very tiny (0.14m arc) | `sim_time=0.8s` at 0.18m/s | Raised to `sim_time=3.0s, sim_steps=20` |
| Robot rotates excessively | `angle_threshold=0.20` (11°) — Phase A triggered too often | Raised to `0.50` (29°) — DWA handles small angles while moving |
| Robot replans every second | Timer-based replan fired regardless of path validity | Added blocked-path check + 5s cooldown in `_replan_check` |
| Robot freezes during replan | `current_path` cleared before new plan arrived | Keep `current_path` until new plan atomically replaces it |
| Infinite replan loop | DWA stuck → replan → still stuck → replan → ... | 50-tick (5s) cooldown in DWA Phase C before next `/replan_request` |
| `Bool` topic not received | ROS2 Jazzy Bool serialization bug | Changed `/replan_request` to `std_msgs/Int32` |

---

## 8. Real Robot Specifics

### Topic differences
| Topic | Simulator | Real Robot |
|-------|-----------|------------|
| Odometry | `/turtlebot/odom` | `/odom` |
| LiDAR | `/turtlebot/scan` | `/scan` |
| Velocity cmd | `/turtlebot/cmd_vel` (Twist) | `/cmd_vel` (TwistStamped) |

**Important:** The Create3 base requires `TwistStamped` (not plain `Twist`) on `/cmd_vel`. The `dwa_planner` and `control_tb` both handle this automatically when `use_sim=False`.

### Static TF bridge (required for real robot)
```
base_link → rplidar_link
  x=-0.249, y=-0.055, z=0.138
  qx=-0.007, qy=-0.009, qz=0.830, qw=0.558
```
This is published automatically by `phase1_real.launch.py`. Without it, `mapping_node` cannot look up the LiDAR position in the TF tree.

### Real robot uses `online_motion_planning` nodes
`phase1_real.launch.py` uses `mapping_node`, `planning_node`, and `control_tb` from the `online_motion_planning` package — these are already tested on real hardware. The key controller used is `control_tb` (simple waypoint follower with backtracking recovery) instead of `dwa_planner` — simpler and more reliable for initial real-robot testing.

---

## 9. Architecture: What Each Node "Knows"

| Node | Knows | Doesn't know |
|------|-------|-------------|
| mapping_node | Where walls are, based on LiDAR | Where robot wants to go |
| global_planner | Full map, current pose, goal | Real-time obstacle dynamics |
| dwa_planner | Next 3 seconds of trajectory options | Long-range path |

---

## 10. Phase 2 / Phase 3 Next Steps

### Phase 2: Dubins Path
- Replace RRT* straight-line segments with Dubins curves (minimum-turning-radius paths)
- Relevant for TurtleBot4 which has a minimum turning radius constraint
- Dubins path types: RSR, LSL, RSL, LSR, RLR, LRL
- Integration point: replace `_path_free()` segment check with Dubins arc collision check

### Phase 3: Frontier-Based Exploration
- Robot autonomously explores unknown space without a user-given goal
- Frontier = boundary between known-free and unknown cells
- Algorithm: find all frontiers → cluster them → send robot to nearest/largest cluster
- When frontier reached, remap from new position, find next frontier
- Integration point: add `frontier_explorer.py` node that publishes to `/goal_pose`

### Questions to ask professor (prepared for demo)
1. Why RRT* and not A* or Dijkstra? (probabilistic completeness, continuous space, no discretization needed)
2. What is the rewire step in RRT* and why does it make it "Star"? (optimality guarantee)
3. Why inflate the map before planning? (C-space transformation, equivalent to planning for a point robot)
4. What is the difference between log-odds and direct probability for mapping? (numerical stability, avoids 0/1 saturation)
5. What happens if the DWA scores are all equal? (arbitrary choice — add tiebreaker or dither)
6. How does the sticky wall prevent map erosion? (threshold 2.0 locks confirmed walls)
7. Why is a 5-second replan cooldown safe? (DWA handles short-term obstacles, global replan only for long-term blockage)
8. What is the completeness of DWA? (not complete — can get stuck; global replanning is the escape)
9. How would you extend this to 3D? (OctoMap instead of OccupancyGrid, 3D RRT* in SE3)

---

## 11. RViz Quick Setup

Add these displays in RViz2:

| Display | Topic | Notes |
|---------|-------|-------|
| Map | `/projected_map` | Set colour scheme to costmap |
| Path | `/plan` | Set colour to red, line width 0.05 |
| Path | `/local_path` | Set colour to cyan |
| MarkerArray | `/rrt_tree` | Grey exploration tree |
| MarkerArray | `/dwa_trajectories` | Blue fan + green best |
| Odometry | `/turtlebot/odom` | Robot pose history |
| Fixed Frame | `world_enu` (sim) / `odom` (real) | Must match map_frame |

---

## 12. Build Commands

```bash
# Build both packages
cd ~/ros2_ws
colcon build --packages-select handson_planning online_motion_planning
source install/setup.bash

# Verify nodes registered
ros2 pkg executables handson_planning
ros2 pkg executables online_motion_planning

# Check topics while running
ros2 topic list
ros2 topic hz /projected_map     # should be ~2 Hz
ros2 topic hz /plan              # publishes when new path computed
ros2 topic hz /dwa_trajectories  # should be ~10 Hz

# Debug replanning
ros2 topic echo /replan_request  # watch for Int32(1) from DWA stuck events
```

---

## 13. ROBOT_RADIUS Shared Constant

`ROBOT_RADIUS = 0.16` (sim) / `0.18` (real) is defined at the top of each launch file and passed to both `global_planner` and `dwa_planner`. It **must be the same in both nodes** — if they differ, the DWA will reject trajectories that the global planner considered safe, causing the robot to constantly think paths are blocked.

---

*End of notes — continue from Phase 2 (Dubins Path) or Phase 3 (Frontier Exploration)*
