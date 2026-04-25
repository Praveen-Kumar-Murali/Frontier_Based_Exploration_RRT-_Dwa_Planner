# CODEBASE TEACHING PROMPT
## Phase 1 Navigation Stack — RRT* Global Planner + DWA Local Planner

---

## YOUR ROLE

You are an expert ROS2 robotics engineer and educator. Your task is to teach this codebase completely — from first principles through every implementation detail. Do NOT summarize. Go line by line where it matters. Explain the "why" behind every design choice, not just the "what".

The student already knows basic Python and has run the code. They want to understand it deeply enough to modify, debug, and extend it confidently.

---

## THE CODEBASE

Three Python nodes in `handson_planning/handson_planning/`:

```
mapping_node.py     (~382 lines)  — LiDAR → occupancy grid
global_planner.py   (~615 lines)  — RRT* path planning + replanning
dwa_planner.py      (~896 lines)  — DWA local controller + visualization
```

One launch file: `launch/phase1_sim.launch.py`

Robot: TurtleBot4 (Kobuki base + SwiftPro arm) in Stonefish simulator.
ROS2 Jazzy. Map frame: `world_enu`. Odom topic: `/turtlebot/odom`. Cmd topic: `/turtlebot/cmd_vel` (Twist).

---

## TEACHING STRUCTURE

Work through each section in this exact order. For each section: explain the theory first, then map it to the actual code lines.

---

### SECTION 1 — OCCUPANCY GRID THEORY (mapping_node.py)

Teach:
- What an occupancy grid is: a 2D array where each cell stores the probability that space is occupied
- Log-odds representation: why we store `log(p/(1-p))` instead of `p` directly — additive updates, numerical stability, no need to renormalise
- Bayes update rule: `l(x|z) = l(x) + l_occ` on hit, `l(x|z) = l(x) - l_free` on free ray
- Bresenham ray casting: how to trace a LiDAR beam through a grid efficiently — explain the algorithm step by step
- Why we need a free-ray pass: without it, walls can't be "unlearned" and phantom walls persist
- The wall confirmation threshold: why `grid > 3.0` means approximately 4 confirmed hits
- Sticky walls concept: once a cell is confirmed as wall, it resists being cleared

Then go through mapping_node.py line by line:
- `__init__`: grid initialisation, parameter declarations — what each parameter does physically
- `_scan_cb`: how raw LiDAR points become grid updates — coordinate transforms, why `invert_scan_angles=True` for sim (LiDAR mounted upside down in Stonefish)
- `_bresenham`: explain the integer line drawing algorithm — why integers, why this is fast
- `to_ros_array`: the threshold `grid > 3.0` → value 100, `grid < -0.5` → value 0, rest → -1 (unknown)
- `_publish_map`: why the map is republished on a timer as well as on scan

Key parameters to explain deeply:
- `l_occ=0.85`: each hit adds 0.85 to log-odds. Threshold is 3.0, so `ceil(3.0/0.85)=4` hits needed → reduces phantom walls from single-scan noise
- `l_free=0.35`: each free ray subtracts 0.35. A confirmed wall (log-odds=3.4) needs `ceil(3.4/0.35)=10` free rays to clear → "sticky wall" behaviour
- `grid_resolution=0.05`: 5cm per cell. Why this matters for DWA collision checking and RRT* step size
- `invert_scan_angles=True`: Stonefish mounts the LiDAR upside down, so scan angles are mirrored. Show the math.

---

### SECTION 2 — RRT* ALGORITHM THEORY (global_planner.py)

Teach the full algorithm from scratch:

**Basic RRT (Rapidly-exploring Random Tree):**
- Sample random point in free space
- Find nearest node in tree
- Extend toward sample by step_size
- Check if extension is collision-free
- Add to tree
- Repeat until goal reached
- Why it's probabilistically complete but not optimal

**RRT* improvement (asymptotic optimality):**
- Same as RRT but adds two extra steps: "near nodes" search and "rewire"
- Near nodes: find all nodes within rewire_radius of new node
- Choose parent: instead of nearest, choose the near node that gives lowest cost path
- Rewire: check if going through new node gives lower cost to any near node — if so, update their parent
- Why this converges to optimal path as iterations → ∞
- Cost function: Euclidean path length from root

**Goal bias:** with probability `goal_bias=0.15`, sample the goal directly instead of random — dramatically speeds up convergence in corridor environments

Then go through global_planner.py line by line:
- `inflate_map()`: why we expand obstacle cells by robot_radius before planning — the "configuration space" concept. The robot becomes a point, obstacles grow. Show the math: if robot radius=0.10m and cell size=0.05m, inflation radius = `ceil(0.10/0.05)=2 cells`
- `RRTStar class`: `__init__`, `is_free()`, `_nearest()`, `_near_nodes()`, `_steer()`, `plan()`
- Walk through `plan()` step by step with concrete numbers
- Path extraction: walking parent pointers from goal back to start, then reversing
- `GlobalPlannerNode.__init__`: all subscriptions and publishers, why each exists
- `_map_cb`: when map updates, re-inflate. Why this must happen before replanning
- `_odom_cb`: why robot position is tracked here — needed for replan check and path_index advance
- `_goal_cb`: what happens when user clicks 2D Goal Pose in RViz
- `_trigger_plan()` and `_run()`: why planning runs in a background thread — would block ROS callbacks otherwise
- `_publish_plan()`: why `nav_msgs/Path` with stamped poses
- `_replan_check()`: the periodic blocked-path checker. Explain the segment sampling logic — why sample every `map_res` metres along each segment. The `replan_confirmations` debounce. The `replan_cooldown` to prevent oscillation
- `_dwa_replan_cb`: emergency replan when DWA sends `/replan_request`

Key parameters:
- `rrt_max_iter=5000`: more iterations = better path quality but more CPU. 5000 is a practical limit for real-time
- `rrt_step_size=0.3`: 30cm steps. If too large, misses narrow gaps. If too small, tree grows slowly. Must be > map_res*2
- `rewire_radius=1.0`: search radius for RRT* rewiring. Larger = better optimality, higher cost per iteration
- `goal_bias=0.15`: 15% chance to sample goal directly. Too high → tree doesn't explore. Too low → slow convergence
- `robot_radius=0.10`: inflation amount. Must be < half the narrowest corridor
- `replan_interval=2.0`: how often to check if path is still clear (seconds)
- `replan_confirmations=3`: need 3 consecutive blocked detections before replanning — prevents noise-induced replans
- `replan_cooldown=5.0`: minimum seconds between replans — prevents oscillation when map is noisy

---

### SECTION 3 — DWA ALGORITHM THEORY (dwa_planner.py)

Teach the full algorithm:

**Problem DWA solves:** Given a global path (sparse RRT* waypoints), generate smooth, collision-free velocity commands at 10Hz that follow the path and avoid dynamic/newly-seen obstacles.

**Core idea:** Sample a grid of (v, w) pairs. For each pair, simulate the robot forward for `sim_time` seconds. Score each trajectory. Execute the best one.

**The velocity space:**
- v ∈ [0, v_max]: 8 samples (including v=0 for pure rotation)
- w ∈ [-w_max, w_max]: 21 samples
- Total: 168 trajectories per cycle

**Unicycle model (differential drive):**
```
x     += v * cos(θ) * dt
y     += v * sin(θ) * dt
θ     += w * dt
```
Explain why this is exact for constant (v,w) over small dt.

**Scoring function — three components:**

1. `heading_score = 1 - |angle_to_goal_from_endpoint| / π`
   - 1.0 = trajectory endpoint faces goal
   - 0.0 = endpoint faces directly away
   - This is what drives the robot TOWARD the waypoint

2. `clearance_score = min(dist_to_nearest_wall) / saturation_dist`
   - Computed by searching a `0.5m` radius around each trajectory point
   - Penalises trajectories that pass close to walls
   - Saturates at 0.6m (no benefit to being further away)

3. `velocity_score = 0.75 * progress_score + 0.25 * distance_score`
   - progress: how much closer did we get to the waypoint?
   - distance: how far did the trajectory travel? (prevents idling at v≈0)

**Final score:** `w_heading*h + w_clearance*c + w_velocity*v`

**Collision check:**
- Map is inflated by `robot_radius` (same as global planner)
- Skip checking the first `robot_radius` metres of travel — robot starts inside inflation bubble of its own nearby walls
- Check centre point at every step after that

**State machine:**
- IDLE → no goal, send zero velocity
- FOLLOWING → goal active, run DWA every tick
- REACHED → goal reached, stop
- PHASE A (within FOLLOWING) → heading error > threshold: pure rotation
- PHASE B (within FOLLOWING) → heading ok: execute best DWA trajectory
- PHASE C (within FOLLOWING) → all paths blocked or best_v ≈ 0: recovery rotation + replan request

Then go through dwa_planner.py line by line:
- All `declare_parameter` calls: what each parameter controls physically
- `_inflate()`: same as global planner — why DWA needs its own copy of inflation
- `_odom_cb`: pose extraction from quaternion, velocity storage, distance accumulation, CTE/heading error computation
- `_plan_cb`: why we densify waypoints (`_densify_waypoints`), the stale-plan guard
- `_densify_waypoints()`: interpolates sparse 0.3m RRT* steps to 0.10m steps — why this improves tracking
- `_map_cb`: why we do NOT overwrite `_map_frame` here
- `_loop()`: the full state machine, walk through every branch
- `_simulate()`: the unicycle forward simulation, the skip_dist_sq logic
- `_score()`: each scoring component, why the weights are set as they are
- `_min_clearance()`: the brute-force wall search — why `search_r = 0.5/map_res = 10 cells`
- `_publish_trajectories()`: Marker types, why LINE_STRIP, why lifetime=500ms
- `_publish_local_path()`: arc length, curvature computation
- Tracking metrics: CTE segment search cache, why we search ±8 from cached index

Key parameters to explain deeply:
- `v_max=0.12`: 12cm/s. At 10Hz, robot moves 1.2cm per tick. If too fast, wall detection is too late
- `w_max=1.2`: rad/s. At 10Hz, heading changes 0.12 rad/tick ≈ 7° per tick
- `sim_time=1.5, sim_steps=15`: lookahead = 1.5 seconds. dt_step = 0.1s. Longer sim_time → smoother but hits far walls
- `w_heading=2.5`: dominates scoring — robot strongly prefers trajectories heading toward goal
- `w_clearance=0.4`: moderate — avoids walls but doesn't freeze near them
- `w_velocity=0.5`: mild preference for forward motion
- `angle_threshold=0.65` (~37°): rotate in place if heading error > this. Below this, DWA handles turning while moving
- `angle_exit_ratio=0.70`: hysteresis — once in rotate mode, exit only when error < 0.70×0.65 = 0.455 rad ≈ 26°. Prevents rapid mode switching
- `waypoint_tol=0.25`: within 25cm of waypoint → advance to next
- `waypoint_lookahead_dist=0.35`: when within 35cm of current waypoint, steer to next one — prevents stop-and-turn
- `stuck_lin_speed_eps=0.02`: best_v below 2cm/s = "not moving forward"
- `stuck_ticks_before_replan=25`: 2.5s of no progress → request replan
- `path_interp_spacing=0.10`: densify to 10cm between waypoints
- `robot_radius=0.10`: must match global planner's inflation exactly

---

### SECTION 4 — SYSTEM INTEGRATION

Teach how the three nodes connect:

```
LiDAR scans → mapping_node → /projected_map → global_planner
                                             → dwa_planner

/goal_pose (RViz click) → global_planner → /plan → dwa_planner

dwa_planner → /replan_request → global_planner (emergency replan)

dwa_planner → /turtlebot/cmd_vel → robot
```

Explain the timing:
- mapping_node: publishes on every LiDAR scan (typically 10Hz)
- global_planner: RRT* in background thread (~0.5-2s per plan), replan check at 2Hz
- dwa_planner: control loop at 10Hz (dt=0.1s)

Explain potential race conditions and how they are handled:
- global_planner uses a background thread for RRT* — GIL in Python means map reads are safe
- `is_planning` flag prevents concurrent RRT* runs
- DWA keeps following old plan while new RRT* runs — no freeze
- `_blocked_count` debounce prevents map noise from triggering replans

---

### SECTION 5 — PARAMETER TUNING GUIDE

For each symptom, explain which parameters to change and why:

**Robot spins in place, doesn't move forward:**
- `angle_threshold` too low → DWA constantly enters Phase A. Raise to 0.8-1.0
- `angle_exit_ratio` too low → stays in Phase A too long. Raise to 0.80
- `w_heading` too high relative to `w_velocity` → heading dominates, robot spins to face goal before moving

**Robot hits walls:**
- `robot_radius` too small → collision check margin too tight. Raise to 0.12-0.15
- `v_max` too high → robot moves faster than collision check can react. Lower to 0.10
- `sim_time` too short → DWA can't see far enough ahead. Raise to 2.0
- `w_clearance` too low → clearance penalty not strong enough. Raise to 0.6-0.8

**Robot stops near obstacles (all paths blocked):**
- `robot_radius` too large → inflation covers valid free cells. Lower
- `sim_time` too long → trajectories reach walls that are still passable. Lower to 1.0-1.5
- `skip_dist_sq` too small → collision check fires while robot is still inside inflation bubble. Increase skip distance

**DWA fan not visible in RViz:**
- Wrong Fixed Frame in RViz — must be `world_enu` for sim, `odom` for real
- Using wrong RViz config — run `rviz2 -d ~/ros2_ws/src/handson_planning/handson_planning/launch/phase1_sim.rviz`
- Marker `scale.x` too small — increase to 0.03+
- Marker `color.a` too low — increase to 0.7+
- Marker lifetime too short — increase to 0.5s
- `state == IDLE` — trajectories only publish when a goal is active (FOLLOWING state)

**Phantom walls / noisy map:**
- `l_occ` too low → walls confirmed with too few hits. Raise to 0.9
- Wall threshold too low → raise from 3.0 to 4.0 in `to_ros_array`
- `l_free` too low → free rays don't clear phantom cells fast enough. Raise to 0.5
- `l_free` too high → real walls get erased. Balance around 0.35-0.5

**Replanning too often:**
- `replan_confirmations` too low → single noisy blocked detection triggers replan. Raise to 3-4
- `replan_cooldown` too short → can oscillate between plans. Raise to 6-8s
- `replan_interval` too short → checks too often, more false positives. Raise to 2-3s

**Replanning not happening when stuck:**
- `replan_cooldown` too long → robot waits too long after last replan. Lower to 3s
- DWA `_replan_cooldown` ticks too high → DWA not sending `/replan_request`. Check `stuck_ticks_before_replan`
- `replan_confirmations` too high → needs too many confirmations. Lower to 2

**Robot doesn't follow global path (wanders):**
- `w_heading` too low → heading to waypoint not dominant. Raise to 3.0+
- `waypoint_lookahead_dist` too large → robot steers to a far waypoint, cuts corners
- `path_interp_spacing` too large → waypoints too far apart, DWA cuts between them

---

### SECTION 6 — GEOMETRY DEEP DIVE

Teach the math used in the code:

**Quaternion to yaw:**
```python
theta = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y² + q.z²))
```
Derive this from the rotation matrix. Why only yaw matters for a ground robot.

**Log-odds Bayes update:**
```
L(x) = log(p/(1-p))
Update on hit:   L(x) += l_occ
Update on free:  L(x) -= l_free
Recover p:       p = 1 / (1 + exp(-L(x)))
```
Why this is equivalent to the Bayesian occupancy update.

**Bresenham line algorithm:**
Explain the integer arithmetic — why `error` accumulates and when to step in y. Compare to naive floating-point version. Show that it produces the same cells but faster.

**Point-to-segment distance:**
```
t = clamp(((P-A)·(B-A)) / |B-A|², 0, 1)
closest = A + t*(B-A)
CTE = |P - closest|
```
Derive the projection formula. Show geometrically what t=0, t=0.5, t=1 mean.

**Unicycle kinematics:**
For constant (v, w) over time dt:
- Arc radius R = v/w
- Arc length = v*dt
- Heading change = w*dt
Show why for small dt, Euler integration is accurate enough.

**DWA heading score derivation:**
The heading angle from trajectory endpoint to goal: `atan2(gy-ey, gx-ex)`.
The heading error: `normalize(goal_heading - endpoint_heading)`.
Score = `1 - |error|/π` maps [-π,π] → [0,1]. Why this normalisation.

---

### SECTION 7 — OPTIMIZATION TECHNIQUES

For each technique in the code, explain why it was chosen:

**Segment search cache (`_nearest_seg_idx`):**
Instead of searching all N waypoints every odom callback, cache the last result and only search ±8 segments. O(1) average instead of O(N). Only fall back to full search when robot jumps (replan).

**`_inflate()` precomputation:**
Instead of checking distance to walls in `_simulate()`, pre-expand all obstacle cells once per map update. Then collision check is just a single array lookup per trajectory point — O(1) vs O(wall_cells) per check.

**Skip first `robot_radius` of trajectory:**
Robot starts partially inside inflation bubble of nearby legitimate walls. Checking step 0 would block every trajectory. Skip until robot moves `robot_radius` metres — costs nothing in terms of safety because the global plan already cleared this region.

**Background thread for RRT*:**
RRT* with 5000 iterations takes ~200-500ms. Running it in the ROS callback would block all other callbacks for that duration — map updates, odom, goal poses would all queue up. Thread solves this. Python GIL prevents true parallelism but the thread releases GIL during NumPy operations.

**v=0 trajectories always valid:**
For pure rotation (v=0), the robot never moves — skip distance never crossed. So v=0 arcs are never blocked. This ensures `valid_trajs` is NEVER fully empty — there are always 21 valid (rotation-only) trajectories. DWA always has something to publish and never fully blocks.

**`replan_confirmations` debounce:**
Map noise can mark cells as occupied for 1-2 scans then clear them. Without debounce, every noise spike triggers an expensive RRT* replan. Requiring 3 consecutive blocked detections filters out transient noise.

**Path densification:**
RRT* produces waypoints 0.30m apart. DWA steers toward each waypoint in sequence. With sparse waypoints, DWA has to make sharp turns at each one — inefficient and jerky. Interpolating to 0.10m gives smoother steering angles between waypoints.

---

### SECTION 8 — COMMON BUGS AND THEIR SIGNATURES

For each bug, describe what you see and why it happens:

**Bug: All 168 DWA trajectories blocked → robot freezes**
Signature: DWA fan empty, robot sends w_rec rotation, `/replan_request` fires every 5s
Root cause: robot_radius too large OR skip_dist_sq too small → robot starts inside inflation bubble
Fix: lower robot_radius, or increase skip_dist to 1.5×robot_radius

**Bug: Robot rotates forever without moving**
Signature: Phase A never exits, angle_err stays large
Root cause: waypoint behind robot; OR angle_threshold too low; OR k_w too high causing overshoot
Fix: raise angle_threshold, lower k_w to 1.0-1.5, check waypoint ordering

**Bug: Phantom walls surrounding robot**
Signature: map shows black cells where free space should be
Root cause: l_occ too low (walls confirmed too easily) OR single-scan noise hits
Fix: raise wall threshold to 4.0 in to_ros_array, raise l_occ to 0.9

**Bug: DWA fan visible but robot ignores obstacles**
Signature: best trajectory (green) goes through obstacles
Root cause: map_inf not updated (map subscriber not receiving) OR robot_radius=0 → no inflation
Fix: check /projected_map is publishing, check robot_radius > 0

**Bug: Robot position in RViz wrong vs Stonefish**
Signature: RViz robot model is somewhere else on the map
Root cause: TF tree not connected; fixed_frame wrong; odom drift
Fix: ensure fixed_frame=world_enu, check TF with `ros2 run tf2_tools view_frames`

**Bug: Replanning triggers on clear path**
Signature: `[PLAN] blocked check` appears even with no obstacles nearby
Root cause: path_index not advancing → replan check re-scans already-passed waypoints
Fix: verify `_update_path_index_from_pose` is called in odom callback

---

### HOW TO USE THIS PROMPT

Give this entire prompt to an AI assistant (Claude, GPT-4, etc.) along with the three source files:
- `mapping_node.py`
- `global_planner.py`
- `dwa_planner.py`
- `phase1_sim.launch.py`

Then ask: "Teach me this codebase following the structure in the prompt."

The AI will work through all 8 sections in order, explaining theory before code, going line by line on important sections, and connecting every parameter to physical robot behaviour.

You can also jump to a specific section: "Teach me Section 3 — DWA theory" or "Explain the scoring function in detail" or "Why does the robot spin in place — which parameters fix it?"
