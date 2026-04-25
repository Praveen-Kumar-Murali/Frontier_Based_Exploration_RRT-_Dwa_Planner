#!/usr/bin/env python3
"""
planning_node.py — RRT* Motion Planner with Online Replanning


This node implements sampling-based motion planning using the RRT* algorithm
and performs periodic path validity checking against the current occupancy map.

ARCHITECTURE:
  The node subscribes to the live occupancy grid (/projected_map or a saved
  file), the robot odometry (/odom), and user-specified goals (/goal_pose).
  Upon receiving a goal, RRT* is executed in a background thread to avoid
  blocking the ROS callback executor. The resulting path is published to
  /plan for the waypoint controller.

MAP INFLATION:
  Prior to planning, all occupied cells are expanded by robot_radius using
  the inflate_map() function. This converts point-obstacle cells into
  configuration-space obstacles, ensuring that any path through the inflated
  map maintains at least robot_radius clearance from all known walls.
  Unknown cells (-1) are not inflated, allowing the planner to route through
  unexplored regions during live-mapping phases.

ONLINE REPLANNING:
  A ROS timer fires every replan_interval seconds and checks whether each
  segment of the current path remains free in the latest inflated map.
  If any segment is blocked, the robot stops and a new plan is computed.
  Path validity is assessed by sampling interpolated points along each
  segment and querying the inflated occupancy grid.

Parameters:
  use_sim            — selects simulator or real-robot topic names
  use_saved_map      — load map from file (Phase 1/3) or subscribe live
  robot_radius       — inflation radius in metres (default 0.10 m)
  rrt_max_iter       — maximum RRT* iterations per planning call
  rrt_step_size      — maximum extension distance per iteration (m)
  replan_interval    — period of path validity check (seconds)
  waypoint_tol       — waypoint reached distance threshold (m)
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import threading
import os
import random

from nav_msgs.msg           import OccupancyGrid, Odometry, Path
from geometry_msgs.msg      import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg           import Int32


_MAPS        = os.path.expanduser('~/ros2_ws/maps')
MAP_FILE     = os.path.join(_MAPS, 'lab_map_BEST.npy')
META_FILE    = os.path.join(_MAPS, 'lab_map_BEST_meta.npy')
MAP_FILE_FB  = os.path.join(_MAPS, 'lab_map.npy')
META_FILE_FB = os.path.join(_MAPS, 'lab_map_meta.npy')



def inflate_map(grid, robot_radius, resolution):
    """
    Inflate every occupied cell by robot_radius to create a safety buffer.
    Unknown cells (-1) are NOT inflated — robot can plan through unexplored space.
    """
    inflated = grid.copy()
    r = int(math.ceil(robot_radius / resolution))
    h, w = grid.shape
    occ_rows, occ_cols = np.where(grid == 100)
    for row, col in zip(occ_rows, occ_cols):
        inflated[max(0,row-r):min(h,row+r+1),
                 max(0,col-r):min(w,col+r+1)] = 100
    return inflated



class RRTStarPlanner:


    def __init__(self, grid, resolution, origin,
                 robot_radius=0.10, max_iter=5000, step_size=0.3,
                 rewire_radius=1.0, goal_bias=0.15, logger=None):
        self.grid          = grid
        self.resolution    = resolution
        self.origin        = np.array(origin)
        self.robot_radius  = robot_radius
        self.max_iter      = max_iter
        self.step_size     = step_size
        self.rewire_radius = rewire_radius
        self.goal_bias     = goal_bias
        self.logger        = logger
        self.height, self.width = grid.shape
        self.last_tree     = {}

    def _w2c(self, x, y):
        col = int(np.floor((x - self.origin[0]) / self.resolution))
        row = int(np.floor((y - self.origin[1]) / self.resolution))
        return col, row

    def is_free(self, x, y):
        """Single cell check — map already inflated by robot_radius."""
        col, row = self._w2c(x, y)
        if col < 0 or col >= self.width or row < 0 or row >= self.height:
            return False
        return self.grid[row, col] != 100

    def _path_free(self, p1, p2, n=15):
        for i in range(n + 1):
            t = i / n
            if not self.is_free(p1[0]+t*(p2[0]-p1[0]),
                                 p1[1]+t*(p2[1]-p1[1])):
                return False
        return True

    def plan(self, start, goal):
        log = self.logger.info if self.logger else print
        log(f'[RRT*] start=({start[0]:.2f},{start[1]:.2f}) '
            f'goal=({goal[0]:.2f},{goal[1]:.2f})')

        if not self.is_free(*start):
            if self.logger: self.logger.error('[RRT*] Start in obstacle!')
            return None
        if not self.is_free(*goal):
            if self.logger: self.logger.error('[RRT*] Goal in obstacle!')
            return None

        tree = {0: (start, -1, 0.0)}

        for it in range(self.max_iter):
            if it % 500 == 0 and self.logger:
                self.logger.info(
                    f'[RRT*] iter {it}/{self.max_iter} nodes={len(tree)}')

            rp = goal if random.random() < self.goal_bias else (
                self.origin[0] + random.uniform(0, self.width  * self.resolution),
                self.origin[1] + random.uniform(0, self.height * self.resolution)
            )

            nid  = min(tree, key=lambda n: np.linalg.norm(
                np.array(tree[n][0]) - np.array(rp)))
            npos = tree[nid][0]
            diff = np.array(rp) - np.array(npos)
            dist = np.linalg.norm(diff)
            if dist < 1e-6: continue
            new_pos = tuple(np.array(npos) + diff/dist * min(self.step_size, dist))

            if not self._path_free(npos, new_pos): continue

            new_id    = len(tree)
            best_cost = tree[nid][2] + np.linalg.norm(
                np.array(new_pos) - np.array(npos))
            best_par  = nid

            for i, (p, _, c) in tree.items():
                d = np.linalg.norm(np.array(new_pos) - np.array(p))
                if d < self.rewire_radius:
                    cand = c + d
                    if cand < best_cost and self._path_free(p, new_pos):
                        best_cost = cand; best_par = i

            tree[new_id] = (new_pos, best_par, best_cost)

            for i, (p, par, c) in list(tree.items()):
                if i == new_id: continue
                d = np.linalg.norm(np.array(new_pos) - np.array(p))
                if d < self.rewire_radius:
                    nc = best_cost + d
                    if nc < c and self._path_free(new_pos, p):
                        tree[i] = (p, new_id, nc)

            if np.linalg.norm(np.array(new_pos) - np.array(goal)) < self.step_size:
                gid = len(tree)
                gcost = best_cost + np.linalg.norm(
                    np.array(goal) - np.array(new_pos))
                tree[gid] = (goal, new_id, gcost)
                self.last_tree = tree
                path = self._trace(tree, gid)
                if self.logger:
                    self.logger.info(
                        f'[RRT*]  path found iters={it} '
                        f'waypoints={len(path)} cost={gcost:.2f}m')
                return path

        self.last_tree = tree
        if self.logger:
            self.logger.error(f'[RRT*]  no path after {self.max_iter} iters')
        return None

    def _trace(self, tree, nid):
        path = []
        cur  = nid
        while cur != -1:
            path.append(tree[cur][0])
            cur = tree[cur][1]
        return list(reversed(path))

    def tree_markers(self, frame_id):
        ma = MarkerArray()
        if not self.last_tree: return ma
        m = Marker()
        m.header.frame_id = frame_id
        m.type  = Marker.LINE_LIST
        m.action= Marker.ADD
        m.id    = 0
        m.color.r = 0.4; m.color.g = 0.4; m.color.b = 0.4; m.color.a = 0.6
        m.scale.x = 0.02
        for nid, (pos, par, _) in self.last_tree.items():
            if par == -1 or par not in self.last_tree: continue
            pp = self.last_tree[par][0]
            m.points.append(Point(x=float(pos[0]), y=float(pos[1]), z=0.01))
            m.points.append(Point(x=float(pp[0]),  y=float(pp[1]),  z=0.01))
        ma.markers.append(m)
        return ma



class PlanningNode(Node):


    def __init__(self):
        super().__init__('global_planner')

        self.declare_parameter('use_sim',         True)
        self.declare_parameter('use_saved_map',   False)
        self.declare_parameter('rrt_max_iter',    5000)
        self.declare_parameter('rrt_step_size',   0.3)
        self.declare_parameter('robot_radius',    0.10)  # real robot: 0.10m
        self.declare_parameter('replan_interval', 0.5)
        self.declare_parameter('waypoint_tol',    0.25)
        self.declare_parameter('replan_confirmations', 3)
        self.declare_parameter('replan_cooldown', 8.0)

        use_sim              = self.get_parameter('use_sim').value
        self.use_sim         = use_sim
        self.use_saved_map   = self.get_parameter('use_saved_map').value
        self.rrt_max_iter    = self.get_parameter('rrt_max_iter').value
        self.rrt_step_size   = self.get_parameter('rrt_step_size').value
        self.robot_radius    = self.get_parameter('robot_radius').value
        self.replan_interval = self.get_parameter('replan_interval').value
        self.waypoint_tol    = self.get_parameter('waypoint_tol').value
        self.replan_confirmations = int(self.get_parameter('replan_confirmations').value)
        self.replan_cooldown = float(self.get_parameter('replan_cooldown').value)

        self.map_frame  = 'world_enu' if use_sim else 'odom'
        self.odom_topic = '/turtlebot/odom' if use_sim else '/odom'

        self.map_array    = None
        self.map_inflated = None
        self.map_res      = 0.05
        self.map_w        = 400
        self.map_h        = 400
        self.map_origin_x = -10.0
        self.map_origin_y = -10.0

        self.rx = 0.0; self.ry = 0.0; self.rtheta = 0.0
        self.goal_x = None; self.goal_y = None

        self.current_path    = []
        self.path_index      = 0
        self.is_planning     = False
        self.goal_reached    = False
        self._retry_count    = 0
        self._last_replan_t  = 0.0   # wall-clock time of last replan trigger
        self._blocked_count  = 0
        self._blocked_reason = ''

        if self.use_saved_map:
            self._load_saved_map()

        self.create_subscription(Odometry,    self.odom_topic, self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose',    self._goal_cb, 10)
        self.create_subscription(Int32, '/replan_request', self._dwa_replan_cb, 5)
        if not self.use_saved_map:
            self.create_subscription(OccupancyGrid, '/projected_map',
                                     self._map_cb, 10)

        self.plan_pub = self.create_publisher(Path,          '/plan',          10)
        self.map_pub  = self.create_publisher(OccupancyGrid, '/projected_map', 10)
        self.tree_pub = self.create_publisher(MarkerArray,   '/rrt_tree',      10)

        self.create_timer(self.replan_interval, self._replan_check)
        if self.use_saved_map:
            self.create_timer(1.0, self._republish_map)

        self.get_logger().info('=' * 58)
        self.get_logger().info('✅  PlanningNode ready')
        self.get_logger().info(
            f'   mode  : {"SIM" if use_sim else "REAL"}  |  '
            f'map: {"SAVED FILE" if self.use_saved_map else "LIVE BUILD"}')
        self.get_logger().info(
            f'   frame : {self.map_frame}  |  odom: {self.odom_topic}')
        self.get_logger().info(
            f'   RRT*  : iter={self.rrt_max_iter}  '
            f'step={self.rrt_step_size}m  r_robot={self.robot_radius}m')
        self.get_logger().info(
            f'   replan: every {self.replan_interval}s')
        self.get_logger().info(
            f'   MAP INFLATION: {self.robot_radius}m around walls')
        self.get_logger().info('=' * 58)

    # ── map helpers ────────────────────────────────────────────────────────

    def _do_inflate(self):
        if self.map_array is None:
            return
        self.map_inflated = inflate_map(
            self.map_array, self.robot_radius, self.map_res)

    def _load_saved_map(self):
        mf = MAP_FILE  if os.path.exists(MAP_FILE)  else MAP_FILE_FB
        ef = META_FILE if os.path.exists(META_FILE) else META_FILE_FB
        if not os.path.exists(mf):
            self.get_logger().error(f' saved map not found: {mf}')
            return
        self.map_array = np.load(mf)
        meta = np.load(ef)
        self.map_res      = float(meta[0])
        self.map_w        = int(meta[1])
        self.map_h        = int(meta[2])
        self.map_origin_x = float(meta[3])
        self.map_origin_y = float(meta[4])
        self._do_inflate()
        occ  = (self.map_array == 100).sum()
        free = (self.map_array == 0).sum()
        self.get_logger().info(
            f'✅ map loaded {self.map_w}×{self.map_h} occ={occ} free={free}')

    def _republish_map(self):
        if self.map_array is None: return
        msg = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = self.map_frame
        msg.info.resolution           = self.map_res
        msg.info.width                = self.map_w
        msg.info.height               = self.map_h
        msg.info.origin.position.x    = self.map_origin_x
        msg.info.origin.position.y    = self.map_origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.map_array.flatten().tolist()
        self.map_pub.publish(msg)

    # ── callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rtheta = math.atan2(
            2.0*(q.w*q.z + q.x*q.y),
            1.0 - 2.0*(q.y*q.y + q.z*q.z))
        # Advance path_index as robot passes waypoints so _replan_check
        # never checks already-traversed segments (which may now have walls).
        while (self.current_path and
               self.path_index < len(self.current_path) - 1):
            wx, wy = self.current_path[self.path_index]
            if math.hypot(wx - self.rx, wy - self.ry) < self.waypoint_tol:
                self.path_index += 1
            else:
                break

    def _map_cb(self, msg):
        self.map_res      = msg.info.resolution
        self.map_w        = msg.info.width
        self.map_h        = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_array    = np.array(msg.data, dtype=np.int8).reshape(
            (self.map_h, self.map_w))
        self._do_inflate()

    def _goal_cb(self, msg):
        self.goal_x       = msg.pose.position.x
        self.goal_y       = msg.pose.position.y
        self.goal_reached = False
        self.current_path = []
        self.path_index   = 0
        self._retry_count = 0
        self.get_logger().info(
            f'goal received ({self.goal_x:.2f},{self.goal_y:.2f})')
        self._trigger_plan()

    # ── collision check ────────────────────────────────────────────────────

    def _cell_free(self, wx, wy):
        """Check single cell in inflated map."""
        if self.map_inflated is None:
            return True
        cc = int((wx - self.map_origin_x) / self.map_res)
        rc = int((wy - self.map_origin_y) / self.map_res)
        if not (0 <= cc < self.map_w and 0 <= rc < self.map_h):
            return False
        return self.map_inflated[rc, cc] != 100

    def _find_free_nearby(self, wx, wy, max_m=1.0):
        """Find nearest free cell in inflated map, searching outward."""
        if self.map_inflated is None:
            return wx, wy
        steps = int(max_m / self.map_res)
        for r in range(steps + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    tx = wx + dx * self.map_res
                    ty = wy + dy * self.map_res
                    if self._cell_free(tx, ty):
                        if r > 0:
                            self.get_logger().info(
                                f'[PLAN] pos adjusted '
                                f'({wx:.2f},{wy:.2f})→({tx:.2f},{ty:.2f})')
                        return tx, ty
        return None, None

    # ── planning ───────────────────────────────────────────────────────────

    def _trigger_plan(self):
        if self.is_planning: return
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        if self.goal_x is None: return
        if self.map_inflated is None:
            self.get_logger().warn('[PLAN] no map yet')
            return
        self.is_planning = True

        sx, sy = self._find_free_nearby(self.rx,     self.ry)
        gx, gy = self._find_free_nearby(self.goal_x, self.goal_y)

        if sx is None:
            self.get_logger().error('[PLAN] no free cell near start')
            self.is_planning = False
            return
        if gx is None:
            self.get_logger().error('[PLAN] no free cell near goal')
            self.is_planning = False
            return

        planner = RRTStarPlanner(
            grid         = self.map_inflated.copy(),
            resolution   = self.map_res,
            origin       = [self.map_origin_x, self.map_origin_y],
            robot_radius = self.robot_radius,
            max_iter     = self.rrt_max_iter,
            step_size    = self.rrt_step_size,
            rewire_radius= 1.0,
            goal_bias    = 0.15,
            logger       = self.get_logger(),
        )

        path = planner.plan((sx, sy), (gx, gy))

        if path:
            # Atomically replace path — DWA switches to new plan immediately
            self.current_path = path
            self.path_index   = 0
            self._retry_count = 0
            import time as _time
            self._last_replan_t = _time.monotonic()
            self._pub_plan(path)
            self._pub_tree(planner)
            self.get_logger().info(
                f' /plan published {len(path)} waypoints')
        else:
            self._retry_count += 1
            if self._retry_count < 3:
                self.get_logger().error(
                    f'[PLAN] no path — retry {self._retry_count}/3 in 2s')
                self.is_planning = False
                threading.Timer(2.0, self._trigger_plan).start()
                return
            else:
                self.get_logger().error(
                    '[PLAN] no path after 3 retries — click a new goal')
                self._retry_count = 0
                self._stop()

        self.is_planning = False

    # ── DWA emergency replan ───────────────────────────────────────────────

    def _dwa_replan_cb(self, msg):
        """DWA local planner sends Int32(1) when all trajectories are blocked."""
        import time as _time
        now = _time.monotonic()
        if msg.data == 1 and not self.is_planning and self.goal_x is not None:
            if now - self._last_replan_t < self.replan_cooldown:
                return   # cooldown — ignore rapid-fire requests
            self.get_logger().warn('[Global] DWA stuck — emergency replan')
            self._last_replan_t = now
            self._blocked_count = 0
            # Keep current_path — DWA keeps rotating while RRT* runs
            self._trigger_plan()

    # ── online replanning ──────────────────────────────────────────────────

    def _replan_check(self):
        """
        Path validity check — fires on a timer but only replans when the
        current path is ACTUALLY blocked.

        Design principles:
          1. If nothing is blocked → do nothing (robot keeps following path).
          2. If a waypoint ahead is blocked → trigger RRT* in background.
          3. Do NOT clear current_path before new plan arrives — DWA continues
             following the old plan while RRT* runs, preventing freeze/spin.
          4. Minimum 5 s between successive replans to prevent rapid looping
             caused by map noise.
          5. Only checks the next 3 waypoints ahead (not the whole path) to
             avoid false positives on far-future still-unmapped segments.
        """
        if self.is_planning or self.goal_reached or self.goal_x is None:
            self._blocked_count = 0
            return
        if self.map_inflated is None:
            return
        if not self.current_path:
            return

        # ── minimum cooldown between replans ────────────────────────────
        import time as _time
        now = _time.monotonic()
        if now - self._last_replan_t < self.replan_cooldown:
            return

        # ── check next 3 waypoints (not current position) ───────────────
        blocked_reason = None

        # Check next 3 waypoints — sample every map cell along each segment
        # so no obstacle is missed between sparse RRT* waypoints (0.3m apart).
        sx, sy = self.rx, self.ry
        for idx in range(self.path_index, min(self.path_index + 3, len(self.current_path))):
            nx, ny = self.current_path[idx]
            seg_len = math.hypot(nx - sx, ny - sy)
            n_steps = max(2, int(math.ceil(seg_len / self.map_res)))
            for k in range(1, n_steps + 1):
                t  = k / n_steps
                cx = sx + t * (nx - sx)
                cy = sy + t * (ny - sy)
                if not self._cell_free(cx, cy):
                    blocked_reason = f'wp[{idx}] segment blocked at ({cx:.2f},{cy:.2f})'
                    break
            if blocked_reason:
                break
            sx, sy = nx, ny

        if blocked_reason is None:
            if self._blocked_count > 0:
                self.get_logger().info('[PLAN] path clear — blockage counter reset')
            self._blocked_count = 0
            self._blocked_reason = ''
            return   # path is clear — do NOT replan

        # Require repeated confirmation to avoid map-noise replans.
        if blocked_reason == self._blocked_reason:
            self._blocked_count += 1
        else:
            self._blocked_reason = blocked_reason
            self._blocked_count = 1

        self.get_logger().warn(
            f'[PLAN] blocked check {self._blocked_count}/{self.replan_confirmations}: {blocked_reason}')

        if self._blocked_count < self.replan_confirmations:
            return

        # ── path is blocked — trigger background RRT* ────────────────────
        self.get_logger().warn(f'🚧 {blocked_reason} — replanning!')
        self._last_replan_t = now
        self._blocked_count = 0
        self._blocked_reason = ''
        # Keep current_path alive so DWA doesn't freeze while planning runs.
        # _run() will replace current_path atomically when new plan is ready.
        self._trigger_plan()

    def _stop(self):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        self.plan_pub.publish(msg)

    # ── publish ────────────────────────────────────────────────────────────

    def _pub_plan(self, path):
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        for x, y in path:
            ps = PoseStamped()
            ps.header.frame_id    = self.map_frame
            ps.pose.position.x    = float(x)
            ps.pose.position.y    = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.plan_pub.publish(msg)
        self.get_logger().info(f'📍 /plan published {len(path)} waypoints')

    def _pub_tree(self, planner):
        ma = planner.tree_markers(frame_id=self.map_frame)
        for m in ma.markers:
            m.header.stamp = self.get_clock().now().to_msg()
        self.tree_pub.publish(ma)



def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()