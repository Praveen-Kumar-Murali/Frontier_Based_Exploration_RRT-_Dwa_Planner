#!/usr/bin/env python3
"""
dwa_planner.py  —  DWA Local Planner  (Phase 1)




KEY PARAMETER CHOICES
---------------------
  robot_radius  0.10 m  — same as global planner so inflation is consistent
  inflate_frac  0.5     — inflate map by 0.5 * robot_radius = 0.05 m.
                          Enough clearance without blocking the robot's own cell.
  v range       0..v_max — always full range so robot can start from rest
  w range       constrained by a_w_max from current angular velocity
  sim_time      0.8 s   — short enough to avoid hitting unmapped walls
  sim_steps     8       — 0.1 s per step, matches control period

TOPICS IN
---------
  /plan             nav_msgs/Path         RRT* waypoints from global planner
  /turtlebot/odom   nav_msgs/Odometry     robot pose and velocity
  /projected_map    nav_msgs/OccupancyGrid live occupancy map from mapper

TOPICS OUT
----------
  /turtlebot/cmd_vel  geometry_msgs/Twist  velocity command to robot
  /replan_request     std_msgs/Int32 = 1   emergency replan request
"""

import math
import numpy as np
import rclpy
from rclpy.node        import Node
from geometry_msgs.msg import Twist, TwistStamped, Point
from nav_msgs.msg      import Odometry, Path, OccupancyGrid
from std_msgs.msg      import Int32, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

# ── state machine labels 
IDLE      = 'IDLE'
FOLLOWING = 'FOLLOWING'
REACHED   = 'REACHED'


def _normalize_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _dist_point_to_segment(px, py, ax, ay, bx, by):
    """
    Shortest distance from point P to segment AB.
    Returns (distance, t) where t in [0,1] is the projection parameter.
    """
    dx, dy = bx - ax, by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq < 1e-12:          # degenerate segment
        return math.hypot(px - ax, py - ay), 0.0
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / seg_sq))
    cx, cy = ax + t * dx, ay + t * dy
    return math.hypot(px - cx, py - cy), t


def _inflate(grid, robot_radius, res):
    """
    Dilate every occupied cell (value 100) outward by robot_radius metres.
    Matches the same inflation used by the global planner (RRT*), so DWA
    collision checks are consistent with the planned path.
    Unknown cells (-1) are unchanged.
    """
    radius_m = robot_radius
    r = max(1, int(math.ceil(radius_m / res)))
    out = grid.copy()
    rows, cols = np.where(grid == 100)
    h, w = grid.shape
    for rr, cc in zip(rows, cols):
        out[max(0, rr - r):min(h, rr + r + 1),
            max(0, cc - r):min(w, cc + r + 1)] = 100
    return out



class DWAPlanner(Node):

    def __init__(self):
        super().__init__('dwa_planner')

        # ── declare parameters (match names in launch file exactly) ───────
        self.declare_parameter('use_sim',         True)
        self.declare_parameter('robot_radius',    0.10)   # m
        self.declare_parameter('v_max',           0.18)   # m/s
        self.declare_parameter('w_max',           1.2)    # rad/s
        self.declare_parameter('a_w_max',         1.5)    # rad/s²
        self.declare_parameter('v_samples',       8)
        self.declare_parameter('w_samples',       21)
        self.declare_parameter('sim_time',        3.0)    # s — long enough to see in RViz
        self.declare_parameter('sim_steps',       20)
        self.declare_parameter('waypoint_tol',    0.25)   # m
        self.declare_parameter('w_heading',       3.0)    # score weight
        self.declare_parameter('w_clearance',     0.5)
        self.declare_parameter('w_velocity',      0.3)
        self.declare_parameter('dt',              0.1)    # control period
        self.declare_parameter('k_w',             2.0)    # angular P-gain
        self.declare_parameter('angle_threshold', 0.50)   # rad — only pure-rotate above this
        self.declare_parameter('angle_exit_ratio', 0.70)  # hysteresis: exit rotate below ratio*threshold
        self.declare_parameter('map_frame',       'odom')  # must match mapping_node map_frame
        self.declare_parameter('path_interp_spacing', 0.10)   # densify sparse global path for smoother DWA tracking
        self.declare_parameter('waypoint_lookahead_dist', 0.35)  # if close to current WP, steer to next WP
        self.declare_parameter('stuck_lin_speed_eps', 0.02)      # m/s: near-zero forward speed threshold
        self.declare_parameter('stuck_ticks_before_replan', 25)  # 25 ticks @10Hz = 2.5s no-progress
        self.declare_parameter('plan_max_age', 3.0)   # seconds: ignore stale /plan on startup

        # ── read parameters ────────────────────────────────────────────────
        def g(n): return self.get_parameter(n).value
        self.use_sim       = g('use_sim')
        self.robot_radius  = g('robot_radius')
        self.v_max         = g('v_max')
        self.w_max         = g('w_max')
        self.a_w_max       = g('a_w_max')
        self.v_samples     = int(g('v_samples'))
        self.w_samples     = int(g('w_samples'))
        self.sim_time      = g('sim_time')
        self.sim_steps     = int(g('sim_steps'))
        self.waypoint_tol  = g('waypoint_tol')
        self.w_heading     = g('w_heading')
        self.w_clearance   = g('w_clearance')
        self.w_velocity    = g('w_velocity')
        self.dt            = g('dt')
        self.k_w           = g('k_w')
        self.angle_thresh  = g('angle_threshold')
        self.angle_exit_ratio = g('angle_exit_ratio')
        self._map_frame    = g('map_frame')   # set from param; NOT overwritten by map messages
        self.path_interp_spacing = g('path_interp_spacing')
        self.waypoint_lookahead_dist = g('waypoint_lookahead_dist')
        self.stuck_lin_speed_eps = g('stuck_lin_speed_eps')
        self.stuck_ticks_before_replan = int(g('stuck_ticks_before_replan'))
        self.plan_max_age = g('plan_max_age')

        # ── robot state ───────────────────────────────────────────────────
        self.x      = 0.0
        self.y      = 0.0
        self.theta  = 0.0
        self.vx     = 0.0    # current linear  velocity (from odom)
        self.wz     = 0.0    # current angular velocity (from odom)
        self.odom_ready = False

        # ── path follower state ───────────────────────────────────────────
        self.waypoints = []   # list of (x, y) from global planner
        self.wp_idx    = 0    # index of current target waypoint
        self.state     = IDLE

        # ── map state ─────────────────────────────────────────────────────
        self.map_inf  = None   # inflated int8 numpy array for collision checks
        self.map_res  = 0.05
        self.map_ox   = -10.0
        self.map_oy   = -10.0
        self.map_w    = 400
        self.map_h    = 400

        # ── recovery state ────────────────────────────────────────────────
        self._stuck_ticks     = 0
        self._replan_cooldown = 0   # ticks before next replan allowed
        self._rotate_mode      = False
        self._no_progress_ticks = 0
        self._node_start_time  = None  # set on first odom; used to reject stale plans

        # ── path comparison metrics (for terminal logging) ────────────────
        self._global_length       = 0.0   # total length of current global plan (m)
        self._global_waypoints    = 0     # number of RRT* waypoints
        self._global_avg_seg      = 0.0   # average segment length (m)
        self._local_length        = 0.0   # length of last DWA arc (m)
        self._local_curvature     = 0.0   # heading change / path length (rad/m)
        self._compare_tick        = 0     # counter for periodic comparison print

        # ── real-time tracking metrics ────────────────────────────────────
        self._executed_dist   = 0.0   # total odometry distance travelled (m)
        self._prev_x          = None  # previous odom x for distance accumulation
        self._prev_y          = None  # previous odom y
        self._cte             = 0.0   # cross-track error to global path (m)
        self._heading_err     = 0.0   # heading error vs closest path segment (rad)
        self._nearest_seg_idx = 0     # cache: start searching from this segment index
        self._metrics_tick    = 0     # counter for 10 Hz print throttle

        # ── ROS topics ────────────────────────────────────────────────────
        odom_t = '/turtlebot/odom'   if self.use_sim else '/odom'
        cmd_t  = '/turtlebot/cmd_vel' if self.use_sim else '/cmd_vel'

        self.create_subscription(Odometry,      odom_t,          self._odom_cb, 10)
        self.create_subscription(Path,          '/plan',          self._plan_cb, 10)
        self.create_subscription(OccupancyGrid, '/projected_map', self._map_cb,  10)

        if self.use_sim:
            self.cmd_pub = self.create_publisher(Twist,        cmd_t, 10)
        else:
            self.cmd_pub = self.create_publisher(TwistStamped, cmd_t, 10)

        self.replan_pub     = self.create_publisher(Int32,       '/replan_request',   5)
        self.traj_pub       = self.create_publisher(MarkerArray,'/dwa_trajectories', 5)
        self.local_path_pub = self.create_publisher(Path,       '/local_path',       5)
        self.status_pub     = self.create_publisher(String,     '/dwa_status',       5)
        self._reached_published = False   # publish goal reached only once per goal
        # _map_frame is set from map_frame param above; also auto-updated from map msg header

        self.create_timer(self.dt, self._loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  DWA Local Planner  (Phase 1)')
        self.get_logger().info(
            f'  v_max={self.v_max} m/s  w_max={self.w_max} rad/s  '
            f'dt={self.dt}s')
        self.get_logger().info(
            f'  DWA: {self.v_samples}v × {self.w_samples}w  '
            f'sim={self.sim_time}s/{self.sim_steps}steps')
        self.get_logger().info(
            f'  inflation = robot_radius = {self.robot_radius:.3f} m')
        self.get_logger().info('  waiting for /plan ...')
        self.get_logger().info('=' * 60)


    #  CALLBACKS


    def _odom_cb(self, msg):
        """Store robot pose and velocity from wheel odometry."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x     = p.x
        self.y     = p.y
        self.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.vx    = msg.twist.twist.linear.x
        self.wz    = msg.twist.twist.angular.z
        self.odom_ready = True
        if self._node_start_time is None:
            self._node_start_time = self.get_clock().now().nanoseconds * 1e-9

        # ── accumulate executed distance ──────────────────────────────────
        if self._prev_x is not None:
            self._executed_dist += math.hypot(p.x - self._prev_x,
                                              p.y - self._prev_y)
        self._prev_x, self._prev_y = p.x, p.y

        # ── CTE and heading error (only when following a path) ────────────
        if self.state == FOLLOWING and len(self.waypoints) > 1:
            wps = self.waypoints
            n   = len(wps)
            # Search from cached index ±5 to avoid full-path scan every tick.
            lo  = max(0, self._nearest_seg_idx - 2)
            hi  = min(n - 1, self._nearest_seg_idx + 8)
            best_d, best_t, best_i = 1e9, 0.0, lo
            for i in range(lo, hi):
                d, t = _dist_point_to_segment(
                    self.x, self.y,
                    wps[i][0], wps[i][1],
                    wps[i+1][0], wps[i+1][1])
                if d < best_d:
                    best_d, best_t, best_i = d, t, i
            # If we reached the end of the search window, also check full path
            # once (catches large jumps after a replan).
            if best_i >= hi - 1:
                for i in range(n - 1):
                    d, t = _dist_point_to_segment(
                        self.x, self.y,
                        wps[i][0], wps[i][1],
                        wps[i+1][0], wps[i+1][1])
                    if d < best_d:
                        best_d, best_t, best_i = d, t, i
            self._nearest_seg_idx = best_i
            self._cte = best_d

            # Heading error: robot heading vs segment direction
            sx = wps[best_i+1][0] - wps[best_i][0]
            sy = wps[best_i+1][1] - wps[best_i][1]
            seg_heading = math.atan2(sy, sx)
            self._heading_err = _normalize_angle(self.theta - seg_heading)

    def _plan_cb(self, msg):
        """Receive new waypoint list from global planner."""
        if not msg.poses:
            self.get_logger().warn('[DWA] empty plan — stopping')
            self.waypoints = []
            self.state     = IDLE
            self._send(0.0, 0.0)
            return

        # Reject stale plans that arrive on startup.
        # The global_planner may re-publish the old plan right after restart
        # if the goal persists across launches.  We compare the plan's header
        # stamp to this node's start time: if the plan was stamped BEFORE the
        # node started, it is a leftover — ignore it.
        # Replans triggered after startup (stamp >= node_start) are always accepted.
        if self._node_start_time is not None and self.state == IDLE:
            plan_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if plan_sec < self._node_start_time - 0.5:
                self.get_logger().warn(
                    f'[DWA] ignoring pre-startup /plan (plan_t={plan_sec:.1f} < '
                    f'node_start={self._node_start_time:.1f}) '
                    f'— click 2D Goal Pose to start')
                return

        raw_wps = [(p.pose.position.x, p.pose.position.y)
                   for p in msg.poses]
        self.waypoints = self._densify_waypoints(raw_wps, self.path_interp_spacing)
        self.wp_idx       = 0
        self.state        = FOLLOWING
        self._stuck_ticks = 0
        self._rotate_mode = False
        self._no_progress_ticks   = 0
        self._reached_published   = False   # reset so next goal gets its signal
        self._executed_dist    = 0.0
        self._nearest_seg_idx  = 0
        self._cte              = 0.0
        self._heading_err      = 0.0
        self._metrics_tick     = 0

        # ── compute global path metrics ───────────────────────────────────
        wps = self.waypoints
        seg_lengths = [
            math.hypot(wps[i+1][0] - wps[i][0], wps[i+1][1] - wps[i][1])
            for i in range(len(wps) - 1)
        ] if len(wps) > 1 else [0.0]
        self._global_length    = sum(seg_lengths)
        self._global_waypoints = len(wps)
        self._global_avg_seg   = self._global_length / max(len(seg_lengths), 1)

        # straight-line distance start→goal
        sx, sy = wps[0]
        gx, gy = wps[-1]
        straight = math.hypot(gx - sx, gy - sy)
        efficiency = straight / self._global_length if self._global_length > 0 else 1.0

        self.get_logger().info('─' * 62)
        self.get_logger().info('  GLOBAL PATH  (RRT* — straight-line segments)')
        self.get_logger().info(
            f'  Waypoints     : {self._global_waypoints}')
        self.get_logger().info(
            f'  Total length  : {self._global_length:.3f} m')
        self.get_logger().info(
            f'  Straight dist : {straight:.3f} m')
        self.get_logger().info(
            f'  Path efficiency: {efficiency:.2f}  (1.0 = perfectly straight)')
        self.get_logger().info(
            f'  Avg seg length: {self._global_avg_seg:.3f} m  '
            f'(step_size=0.3m → coarse waypoints)')
        self.get_logger().info(
            f'  Densified WPs : {len(self.waypoints)} '
            f'(spacing≈{self.path_interp_spacing:.2f} m)')
        self.get_logger().info(
            f'  Goal          : ({gx:.2f}, {gy:.2f})')
        self.get_logger().info('─' * 62)

    def _densify_waypoints(self, waypoints, spacing):
        """Insert intermediate points so DWA tracks smoother targets."""
        if len(waypoints) < 2 or spacing <= 1e-3:
            return waypoints

        dense = [waypoints[0]]
        for i in range(len(waypoints) - 1):
            x1, y1 = waypoints[i]
            x2, y2 = waypoints[i + 1]
            seg = math.hypot(x2 - x1, y2 - y1)
            if seg < 1e-9:
                continue
            steps = max(1, int(math.ceil(seg / spacing)))
            for k in range(1, steps + 1):
                t = k / steps
                dense.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
        return dense

    def _map_cb(self, msg):
        """Receive occupancy grid and build inflated version for DWA checks."""
        # Update local map frame from the incoming map message so that any
        # Marker/Path messages use the actual map frame in RViz. The map_frame
        # parameter still provides an initial value before the first map
        # arrives, but once a map is published we should match its header.
        # This prevents "Status: Error" in RViz when the map uses a different
        # TF frame (e.g. 'world_enu' vs 'odom').
        self._map_frame = msg.header.frame_id
        self.map_res = msg.info.resolution
        self.map_w   = msg.info.width
        self.map_h   = msg.info.height
        self.map_ox  = msg.info.origin.position.x
        self.map_oy  = msg.info.origin.position.y
        raw = np.array(msg.data, dtype=np.int8).reshape((self.map_h, self.map_w))
        self.map_inf = _inflate(raw, self.robot_radius, self.map_res)


    #  MAIN CONTROL LOOP  (10 Hz)


    def _loop(self):
        # Always publish DWA fan when odom + map are ready — even without a goal.
        # This lets you verify trajectories are being sampled in RViz immediately.
        if self.odom_ready and self.map_inf is not None and self.state in (IDLE, REACHED):
            dt_step = self.sim_time / self.sim_steps
            _trajs = []
            for _v in np.linspace(0.0, self.v_max, self.v_samples):
                for _w in np.linspace(-self.w_max, self.w_max, self.w_samples):
                    _t = self._simulate(_v, _w, dt_step)
                    if _t is not None:
                        _trajs.append((0.0, _t))
            self._publish_trajectories(_trajs, _trajs[0][1] if _trajs else None)
            self._send(0.0, 0.0)
            return

        # guard: nothing to do
        if not self.odom_ready or self.state in (IDLE, REACHED):
            self._send(0.0, 0.0)
            return
        if not self.waypoints:
            self._send(0.0, 0.0)
            return

        self._replan_cooldown = max(0, self._replan_cooldown - 1)

        # ── advance past already-reached waypoints ────────────────────────
        # (identical to control_tb.py)
        while self.wp_idx < len(self.waypoints) - 1:
            gx, gy = self.waypoints[self.wp_idx]
            if math.hypot(gx - self.x, gy - self.y) < self.waypoint_tol:
                self.wp_idx += 1
                self.get_logger().info(
                    f'[DWA] waypoint {self.wp_idx}/{len(self.waypoints)} reached')
            else:
                break

        # ── check final goal ──────────────────────────────────────────────
        fx, fy = self.waypoints[-1]
        if math.hypot(fx - self.x, fy - self.y) < self.waypoint_tol:
            self.get_logger().info('[DWA] *** GOAL REACHED ***')
            self.state = REACHED
            self._send(0.0, 0.0)
            if not self._reached_published:
                s = String(); s.data = 'goal reached'
                self.status_pub.publish(s)
                self._reached_published = True
            return

        # DWA target point: when we are already near a waypoint, steer to the
        # next one to avoid stop-and-turn behavior on sparse RRT* paths.
        target_idx = self.wp_idx
        if self.wp_idx < len(self.waypoints) - 1:
            cx, cy = self.waypoints[self.wp_idx]
            if math.hypot(cx - self.x, cy - self.y) < self.waypoint_lookahead_dist:
                target_idx = self.wp_idx + 1
        gx, gy = self.waypoints[target_idx]
        d_target = math.hypot(gx - self.x, gy - self.y)

        # ── compute heading error to current waypoint ─────────────────────
        angle_to_wp = math.atan2(gy - self.y, gx - self.x)
        angle_err   = math.atan2(
            math.sin(angle_to_wp - self.theta),
            math.cos(angle_to_wp - self.theta))

        # ───────────
        #  DWA sampling — always run so trajectories are always visible
        #
        #  v: full range 0..v_max
        #  w: full range -w_max..w_max (unconstrained so visualization
        #     shows the complete fan even while rotating in Phase A)
        # ───────────
        vs      = np.linspace(0.0, self.v_max, self.v_samples)
        ws      = np.linspace(-self.w_max, self.w_max, self.w_samples)
        dt_step = self.sim_time / self.sim_steps

        best_score  = -1e9
        best_v      =  0.0
        best_w      =  0.0
        best_traj   =  None
        valid_trajs = []   # (score, traj) for visualization

        for v in vs:
            for w in ws:
                traj = self._simulate(v, w, dt_step)
                if traj is None:
                    continue
                score = self._score(traj, gx, gy)
                valid_trajs.append((score, traj))
                if score > best_score:
                    best_score = score
                    best_v     = v
                    best_w     = w
                    best_traj  = traj

        # Debug: log valid trajectory count every 20 ticks so we can confirm DWA is sampling
        if not hasattr(self, '_dbg_tick'): self._dbg_tick = 0
        self._dbg_tick += 1
        if self._dbg_tick % 20 == 0:
            self.get_logger().info(
                f'[DWA] valid_trajs={len(valid_trajs)}/168  best_v={best_v:.3f}  '
                f'best_score={best_score:.2f}  robot=({self.x:.2f},{self.y:.2f})')

        # Always publish visualization so RViz always shows the fan
        self._publish_trajectories(valid_trajs, best_traj)

        # ───────────
        #  PHASE A: large heading error — rotate using P-controller
        #
        #  Only trigger pure-rotation when the error is large (>angle_thresh,
        #  default 0.50 rad ≈ 29°).  For smaller errors DWA handles turning
        #  while moving — this eliminates the excessive spin-in-place behaviour.
        #
        #  The robot exits Phase A as soon as it is within angle_thresh of the
        #  waypoint direction, then DWA takes over.  k_w=2.0 makes rotation
        #  fast enough that Phase A is short.
        # ───────────
        enter_thresh = self.angle_thresh
        exit_thresh = max(0.20, self.angle_thresh * self.angle_exit_ratio)
        if self._rotate_mode:
            if abs(angle_err) < exit_thresh:
                self._rotate_mode = False
        else:
            if abs(angle_err) > enter_thresh:
                self._rotate_mode = True

        if self._rotate_mode:
            self._no_progress_ticks = 0
            w = max(-self.w_max, min(self.w_max, self.k_w * angle_err))
            # small forward nudge when almost aligned helps stability
            v_nudge = 0.0
            self._send(v_nudge, w)
            return

        # ───────────
        #  PHASE C: all forward trajectories blocked — recovery
        #
        #  Triggers when:
        #    a) ALL 168 trajectories were collision-blocked (best_score=-1e9), OR
        #    b) DWA found valid trajectories but best picks near-zero forward speed
        #       AND robot is not yet at the waypoint — happens at corners where high
        #       clearance penalty crushes all forward-moving arcs
        # ───────────
        if best_score <= -1e9 or (best_v < self.stuck_lin_speed_eps
                                   and d_target > self.waypoint_tol * 1.5):
            self._stuck_ticks    += 1
            self._no_progress_ticks += 1

            if self._replan_cooldown == 0:
                self.get_logger().warn(
                    f'[DWA] all paths blocked '
                    f'(stuck {self._stuck_ticks} ticks) — requesting replan')
                req       = Int32()
                req.data  = 1
                self.replan_pub.publish(req)
                self._replan_cooldown = 50          # 5 s cooldown

            # Find which direction has the most clear space and rotate there.
            # This breaks the deadlock where rotating toward a blocked waypoint
            # keeps the robot spinning in one direction forever.
            best_w_rec = self.k_w * angle_err   # default: toward waypoint
            best_clear = -1.0
            for w_try in [self.w_max, -self.w_max, self.w_max * 0.5, -self.w_max * 0.5]:
                t_try = self._simulate(0.0, w_try, dt_step)
                if t_try is not None:
                    clr = self._min_clearance(t_try)
                    if clr > best_clear:
                        best_clear = clr
                        best_w_rec = w_try
            w_rec = max(-self.w_max, min(self.w_max, best_w_rec))
            self._send(0.0, w_rec)
            return

        # ───────────
        #  PHASE B: execute best DWA trajectory
        # ───────────
        self._stuck_ticks = 0

        # No-progress watchdog: if DWA keeps selecting almost-zero forward
        # velocity away from the target for several seconds, request replan.
        if best_v < self.stuck_lin_speed_eps and d_target > (self.waypoint_tol * 1.5):
            self._no_progress_ticks += 1
        else:
            self._no_progress_ticks = 0

        if self._no_progress_ticks >= self.stuck_ticks_before_replan and self._replan_cooldown == 0:
            self.get_logger().warn(
                f'[DWA] no forward progress for {self._no_progress_ticks} ticks '
                f'(best_v={best_v:.3f}) — requesting replan')
            req = Int32()
            req.data = 1
            self.replan_pub.publish(req)
            self._replan_cooldown = 50
            self._no_progress_ticks = 0

        # ── tracking metrics printout (every tick = 10 Hz) ───────────────
        self._metrics_tick += 1
        if self._metrics_tick % 10 == 0:   # print once per second
            eff = (self._executed_dist / self._global_length
                   if self._global_length > 1e-6 else 0.0)
            self.get_logger().info(
                f'\n'
                f'  ┌─ TRACKING METRICS ──────────────────────┐\n'
                f'  │  Cross-track error : {self._cte:6.3f} m            │\n'
                f'  │  Heading error     : {self._heading_err:+6.3f} rad          │\n'
                f'  │  Executed dist     : {self._executed_dist:6.3f} m            │\n'
                f'  │  Efficiency        : {eff:6.3f}               │\n'
                f'  └─────────────────────────────────────────┘'
            )

        self._send(best_v, best_w)
        if best_traj is not None:
            self._publish_local_path(best_traj)


    #  TRAJECTORY SIMULATION


    def _simulate(self, v, w, dt_step):
        """
        Simulate constant (v, w) for sim_steps × dt_step seconds.

        Unicycle model:
          x   += v * cos(theta) * dt
          y   += v * sin(theta) * dt
          theta += w * dt

        Collision check:
          - Uses INFLATED map (obstacles expanded by robot_radius, same as RRT*)
          - Step 0 is SKIPPED — the robot may be adjacent to a wall in a valid
            position; checking step 0 would block all forward trajectories.
          - Checks steps 1 onward.  If any point falls in an occupied cell or
            outside the map, the trajectory is rejected (return None).

        Returns list of (x, y, theta) if safe, or None if blocked.
        """
        if self.map_inf is None:
            # No map yet — allow all trajectories (robot stays still anyway)
            return self._simulate_no_map(v, w, dt_step)

        px, py, pth = self.x, self.y, self.theta
        traj = []
        # Skip first few steps to clear the robot's own inflated cell.
        # robot_radius=0.05m → 1-cell inflation.  At v_max=0.12, dt=0.1s:
        # each step moves 0.012m.  4 steps = 0.048m > 0.05m cell → clear.
        # Using 4 keeps enough lookahead (11 of 15 steps checked).
        SKIP_STEPS = 4

        for step in range(self.sim_steps):
            px  += v * math.cos(pth) * dt_step
            py  += v * math.sin(pth) * dt_step
            pth  = math.atan2(math.sin(pth + w * dt_step),
                              math.cos(pth + w * dt_step))
            traj.append((px, py, pth))

            if step < SKIP_STEPS:
                continue

            col = int((px - self.map_ox) / self.map_res)
            row = int((py - self.map_oy) / self.map_res)

            if not (0 <= col < self.map_w and 0 <= row < self.map_h):
                return None

            if self.map_inf[row, col] == 100:
                return None

        return traj   # collision-free

    def _simulate_no_map(self, v, w, dt_step):
        """Simulate with no collision check (map not yet available)."""
        px, py, pth = self.x, self.y, self.theta
        traj = []
        for _ in range(self.sim_steps):
            px  += v * math.cos(pth) * dt_step
            py  += v * math.sin(pth) * dt_step
            pth  = math.atan2(math.sin(pth + w * dt_step),
                              math.cos(pth + w * dt_step))
            traj.append((px, py, pth))
        return traj


    #  TRAJECTORY SCORING


    def _score(self, traj, gx, gy):
        """
        Score a collision-free trajectory on three normalised criteria.

        heading  [0–1]: endpoint points toward the current waypoint
        clearance[0–1]: minimum distance to nearest wall along trajectory
        velocity [0–1]: mix of progress-to-waypoint and travel distance
        """
        ex, ey, eth = traj[-1]

        # heading score
        ang_to_goal = math.atan2(gy - ey, gx - ex)
        head_err    = abs(math.atan2(math.sin(ang_to_goal - eth),
                                     math.cos(ang_to_goal - eth)))
        h_sc = 1.0 - head_err / math.pi   # 1.0 = perfectly aligned

        # clearance score
        min_d = self._min_clearance(traj)
        c_sc  = min(min_d / 0.6, 1.0)    # saturates at 0.6 m

        # progress score: did endpoint get closer to waypoint?
        d0 = math.hypot(gx - self.x, gy - self.y)
        d1 = math.hypot(gx - ex, gy - ey)
        progress = max(0.0, d0 - d1)
        prog_sc = min(progress / max(self.v_max * self.sim_time, 1e-9), 1.0)

        # distance term keeps a mild preference for not idling at v≈0
        dist = math.hypot(ex - self.x, ey - self.y)
        dist_sc = min(dist / max(self.v_max * self.sim_time, 1e-9), 1.0)
        v_sc = 0.75 * prog_sc + 0.25 * dist_sc

        return self.w_heading * h_sc + self.w_clearance * c_sc + self.w_velocity * v_sc

    def _min_clearance(self, traj):
        """
        Minimum distance from any sampled trajectory point to the nearest
        wall cell in the INFLATED map, searched within 0.5 m radius.
        Returns 999.0 if no map or no wall found.
        """
        if self.map_inf is None:
            return 999.0
        search_r = int(0.5 / self.map_res)   # 10 cells at 0.05 m/cell
        min_d    = 999.0

        # sample every other trajectory point for speed
        for (px, py, _) in traj[::2]:
            col = int((px - self.map_ox) / self.map_res)
            row = int((py - self.map_oy) / self.map_res)
            for dc in range(-search_r, search_r + 1):
                for dr in range(-search_r, search_r + 1):
                    nc, nr = col + dc, row + dr
                    if not (0 <= nc < self.map_w and 0 <= nr < self.map_h):
                        continue
                    if self.map_inf[nr, nc] != 100:
                        continue
                    wx = self.map_ox + (nc + 0.5) * self.map_res
                    wy = self.map_oy + (nr + 0.5) * self.map_res
                    d  = math.hypot(px - wx, py - wy)
                    if d < min_d:
                        min_d = d
        return min_d


    #  DWA TRAJECTORY VISUALIZATION


    def _publish_local_path(self, traj):
        """
        Publish the chosen DWA trajectory as a nav_msgs/Path on /local_path.
        In RViz add Path display → topic /local_path (cyan colour).
        Compare with /plan (global RRT* path, red colour) to see the difference:
          /plan       = long-range RRT* route through the whole map
          /local_path = short DWA arc (3 s ahead) the robot is currently executing
        Also computes local metrics and prints a comparison table every ~3 s.
        """
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._map_frame
        for (px, py, pth) in traj:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.position.z = 0.0
            # encode heading as quaternion (yaw only)
            ps.pose.orientation.z = math.sin(pth * 0.5)
            ps.pose.orientation.w = math.cos(pth * 0.5)
            msg.poses.append(ps)
        self.local_path_pub.publish(msg)

        # ── compute local path metrics ────────────────────────────────────
        # Arc length: sum of Euclidean distances between consecutive points
        arc_len = 0.0
        for i in range(len(traj) - 1):
            dx = traj[i+1][0] - traj[i][0]
            dy = traj[i+1][1] - traj[i][1]
            arc_len += math.hypot(dx, dy)

        # Curvature: total heading change / arc length (rad/m)
        #   High curvature → tight curve.  Low → nearly straight arc.
        total_heading_change = 0.0
        for i in range(len(traj) - 1):
            dth = traj[i+1][2] - traj[i][2]
            # wrap to [-pi, pi]
            dth = math.atan2(math.sin(dth), math.cos(dth))
            total_heading_change += abs(dth)
        curvature = total_heading_change / arc_len if arc_len > 1e-6 else 0.0

        self._local_length    = arc_len
        self._local_curvature = curvature

        # ── periodic comparison print (every 30 ticks ≈ 3 s at 10 Hz) ───
        self._compare_tick += 1
        if self._compare_tick % 30 == 0:
            g_len  = self._global_length
            g_wps  = self._global_waypoints
            g_seg  = self._global_avg_seg
            l_len  = self._local_length
            l_curv = self._local_curvature
            # remaining global path from current waypoint
            wps = self.waypoints
            rem_len = sum(
                math.hypot(wps[i+1][0] - wps[i][0], wps[i+1][1] - wps[i][1])
                for i in range(self.wp_idx, len(wps) - 1)
            ) if self.wp_idx < len(wps) - 1 else 0.0
            wp_done = self.wp_idx
            self.get_logger().info('━' * 62)
            self.get_logger().info('  PATH COMPARISON  (global RRT* vs local DWA arc)')
            self.get_logger().info('━' * 62)
            self.get_logger().info('  GLOBAL (RRT*)  — coarse long-range route')
            self.get_logger().info(f'    Total waypoints   : {g_wps}')
            self.get_logger().info(f'    Waypoints done    : {wp_done}  /  remaining: {g_wps - wp_done - 1}')
            self.get_logger().info(f'    Total path length : {g_len:.3f} m')
            self.get_logger().info(f'    Remaining length  : {rem_len:.3f} m')
            self.get_logger().info(f'    Avg segment       : {g_seg:.3f} m  (step_size=0.3 m)')
            self.get_logger().info('  LOCAL  (DWA)   — smooth 3 s arc robot executes NOW')
            self.get_logger().info(f'    Arc length        : {l_len:.3f} m  (covers ~{self.sim_time:.1f} s lookahead)')
            self.get_logger().info(f'    Curvature         : {l_curv:.3f} rad/m'
                                   f'  (0=straight, >1=sharp turn)')
            self.get_logger().info(f'    Arc / global seg  : {l_len/g_seg:.2f}x'
                                   f'  (DWA arc is this fraction of one RRT* step)'
                                   if g_seg > 1e-6 else '    Arc / global seg  : n/a')
            self.get_logger().info('  KEY DIFFERENCE')
            self.get_logger().info(
                f'    Global plans {g_len:.2f} m once.  '
                f'DWA re-samples {self.v_samples*self.w_samples} arcs @ 10 Hz.')
            self.get_logger().info('━' * 62)

    def _publish_trajectories(self, valid_trajs, best_traj):
        """
        Publish two MarkerArray topics for RViz:
          - All valid (collision-free) trajectories: thin grey lines
          - Best chosen trajectory: thick green line
        Visualises what DWA is sampling every control cycle.
        """
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ── 1. All valid trajectories (blue, thin) ───────────────────────
        # Always prepend the robot's current position as the first point.
        # Without this, v=0 rotation arcs have all points at the same location
        # and LINE_STRIP renders as an invisible dot in RViz.
        for i, (score, traj) in enumerate(valid_trajs):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp    = now
            m.ns              = 'dwa_candidates'
            m.id              = i
            m.type            = Marker.LINE_STRIP
            m.action          = Marker.ADD
            m.scale.x         = 0.03
            m.color.r = 0.2
            m.color.g = 0.4
            m.color.b = 1.0
            m.color.a = 0.7
            m.pose.orientation.w = 1.0
            m.lifetime.sec  = 0
            m.lifetime.nanosec = int(0.5e9)
            origin = Point(); origin.x = self.x; origin.y = self.y; origin.z = 0.02
            m.points.append(origin)
            for (px, py, _) in traj:
                p = Point(); p.x = px; p.y = py; p.z = 0.02
                m.points.append(p)
            ma.markers.append(m)

        # ── 2. Best trajectory (bright green, thick) ─────────────────────
        if best_traj is not None:
            m = Marker()
            m.header.frame_id = self._map_frame
            m.header.stamp    = now
            m.ns              = 'dwa_best'
            m.id              = 0
            m.type            = Marker.LINE_STRIP
            m.action          = Marker.ADD
            m.scale.x         = 0.06
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            m.pose.orientation.w = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = int(0.5e9)
            origin = Point(); origin.x = self.x; origin.y = self.y; origin.z = 0.04
            m.points.append(origin)
            for (px, py, _) in best_traj:
                p = Point(); p.x = px; p.y = py; p.z = 0.04
                m.points.append(p)
            ma.markers.append(m)

        self.traj_pub.publish(ma)


    #  VELOCITY PUBLISHER


    def _send(self, v, w):
        if self.use_sim:
            msg = Twist()
            msg.linear.x  = float(v)
            msg.angular.z = float(w)
        else:
            msg = TwistStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.linear.x  = float(v)
            msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def destroy_node(self):
        try:
            self._send(0.0, 0.0)
        except Exception:
            pass
        super().destroy_node()


# ============================
def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
