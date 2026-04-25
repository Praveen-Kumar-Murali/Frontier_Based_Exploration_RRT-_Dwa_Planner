#!/usr/bin/env python3
"""
frontier_explorer.py  —  Frontier-Based Autonomous Exploration
==============================================================

Ported from Sazid669/Autonomus-Exploration (ROS1) to ROS2 Jazzy.
Core algorithms preserved exactly:
  - DBSCAN clustering  (eps=3 cells, min_samples=4)
  - Maximum Information Gain via Shannon entropy window
  - Score = 0.6 × cluster_length + 0.4 × min_distance
  - Fallback: next point in same cluster → next cluster → replan

StateValidityChecker is reimplemented inline (map_to_position +
is_valid_frontier) — no external dependency needed.

INTEGRATION
-----------
  /projected_map   ──→  frontier_explorer  ──→  /goal_pose  ──→  global_planner
  /turtlebot/odom  ──→  frontier_explorer
  /dwa_status      ──→  frontier_explorer  (goal reached signal from dwa_planner)
  /controller_status ─→ frontier_explorer  (goal reached from control_tb)

RViz topics:
  /frontier_points   PoseArray    all raw frontier cells
  /frontier_clusters MarkerArray  coloured clusters + labels
  /frontiers         MarkerArray  chosen goal marker

START: ros2 topic pub /explorer_start std_msgs/msg/Bool "data: true"  --once
STOP:  ros2 topic pub /explorer_start std_msgs/msg/Bool "data: false" --once

PARAMETERS
----------
  use_sim             bool    True = /turtlebot/odom, False = /odom
  distance_threshold  float   validity check radius in metres (default 0.3)
  map_update_interval float   min seconds between map callbacks processed
  explore_interval    float   seconds between explore loop ticks
  length_weight       float   cluster size score weight  (default 0.6)
  distance_weight     float   distance score weight      (default 0.4)
  entropy_radius      int     window radius in cells for information gain
  dbscan_eps          float   DBSCAN neighbourhood radius in cells
  dbscan_min_samples  int     DBSCAN min points to form cluster
  goal_tolerance      float   metres — distance to consider goal reached
  max_nav_attempts    int     attempts per cluster before moving to next
"""

import math
import rclpy
import numpy as np
import scipy.stats as stats
from collections import deque
from sklearn.cluster import DBSCAN

from rclpy.node        import Node
from nav_msgs.msg      import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point
from std_msgs.msg      import String, Bool
from visualization_msgs.msg import Marker, MarkerArray


# ─────────────────────────────────────────────────────────────────────────────
class StateValidityChecker:
    """
    Inline reimplementation of utils_lib.StateValidityChecker.

    The original repo stores the map TRANSPOSED (.T), so grid indices are
    (col, row) — i.e. point = (i, j) where i is the column index.
    We keep the SAME convention here so all algorithm logic is identical.

    self.map  shape: (width, height)  — matches original .T convention
    """

    def __init__(self, distance_threshold: float = 0.3):
        self.map        = None
        self.resolution = None
        self.origin     = None          # [ox, oy] world coords of cell (0,0)
        self._thresh    = distance_threshold

    def set(self, env_transposed: np.ndarray, resolution: float, origin: list):
        """
        env_transposed: occupancy grid ALREADY transposed (.T), shape (W, H)
        resolution: metres per cell
        origin: [ox, oy] world position of cell (0, 0)
        """
        self.map        = env_transposed
        self.resolution = resolution
        self.origin     = origin

    def map_to_position(self, point) -> np.ndarray:
        """
        Convert (col, row) grid index → world (x, y).
        col = point[0], row = point[1]   (matches transposed convention)
        """
        x = self.origin[0] + point[0] * self.resolution
        y = self.origin[1] + point[1] * self.resolution
        return np.array([x, y])

    def position_to_map(self, world_xy) -> tuple:
        """World (x, y) → (col, row) in transposed map."""
        col = int((world_xy[0] - self.origin[0]) / self.resolution)
        row = int((world_xy[1] - self.origin[1]) / self.resolution)
        return col, row

    def is_valid_frontier(self, world_xy) -> bool:
        """
        True if the world point maps to a FREE cell (value 0).
        Unknown (-1) and occupied (100) are both invalid navigation targets.
        """
        if self.map is None:
            return False
        col, row = self.position_to_map(world_xy)
        w, h = self.map.shape
        if not (0 <= col < w and 0 <= row < h):
            return False
        return self.map[col, row] == 0


# ─────────────────────────────────────────────────────────────────────────────
class FrontierExplorer(Node):
# ─────────────────────────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('frontier_explorer')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('use_sim',             True)
        self.declare_parameter('distance_threshold',  0.3)
        self.declare_parameter('map_update_interval', 1.0)   # s
        self.declare_parameter('explore_interval',    1.0)   # s
        self.declare_parameter('length_weight',       0.6)
        self.declare_parameter('distance_weight',     0.4)
        self.declare_parameter('entropy_radius',      5)     # cells
        self.declare_parameter('dbscan_eps',          3.0)   # cells
        self.declare_parameter('dbscan_min_samples',  4)
        self.declare_parameter('goal_tolerance',      0.40)  # m
        self.declare_parameter('max_nav_attempts',    10)

        def g(n): return self.get_parameter(n).value
        self.use_sim             = g('use_sim')
        dist_thresh              = g('distance_threshold')
        self.map_update_interval = g('map_update_interval')
        self.explore_interval    = g('explore_interval')
        self.length_weight       = g('length_weight')
        self.distance_weight     = g('distance_weight')
        self.entropy_radius      = int(g('entropy_radius'))
        self.dbscan_eps          = g('dbscan_eps')
        self.dbscan_min_samples  = int(g('dbscan_min_samples'))
        self.goal_tolerance      = g('goal_tolerance')
        self.max_nav_attempts    = int(g('max_nav_attempts'))

        # ── state validity checker (inline) ───────────────────────────────
        self.svc = StateValidityChecker(dist_thresh)

        # ── robot / map state ─────────────────────────────────────────────
        self.current_pose  = None          # np.array [x, y, yaw]
        self.map           = None          # transposed grid (W, H) int8
        self.resolution    = None
        self.origin        = None
        self.last_map_time = 0.0           # monotonic float

        # ── exploration state ─────────────────────────────────────────────
        self.goal_reached  = True          # True = ready for next goal
        self.goal          = None          # current (gx, gy) world target
        self.clusters      = {}            # {label: [grid_points]}
        self.sorted_clusters = None
        self.exploring     = False

        # ── ROS topics ────────────────────────────────────────────────────
        odom_t = '/turtlebot/odom' if self.use_sim else '/odom'

        self.create_subscription(OccupancyGrid, '/projected_map',     self._map_cb,    10)
        self.create_subscription(Odometry,      odom_t,               self._odom_cb,   10)
        self.create_subscription(String,        '/dwa_status',        self._status_cb, 10)
        self.create_subscription(String,        '/controller_status', self._status_cb, 10)
        self.create_subscription(Bool,          '/explorer_start',    self._start_cb,  10)

        self.goal_pub      = self.create_publisher(PoseStamped, '/goal_pose',           10)
        self.frontier_pub  = self.create_publisher(PoseArray,   '/frontier_points',     10)
        self.cluster_pub   = self.create_publisher(MarkerArray, '/frontier_clusters',   10)
        self.marker_pub    = self.create_publisher(Marker,      '/frontier_best_marker',10)
        self.status_pub    = self.create_publisher(String,      '/explorer_status',     10)

        self.create_timer(self.explore_interval, self._explore_tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🧭  FrontierExplorer ready (DBSCAN + InfoGain)')
        self.get_logger().info(f'   DBSCAN eps={self.dbscan_eps} cells  '
                               f'min_samples={self.dbscan_min_samples}')
        self.get_logger().info(f'   score weights: length={self.length_weight}  '
                               f'dist={self.distance_weight}')
        self.get_logger().info(f'   entropy radius={self.entropy_radius} cells')
        self.get_logger().info('   → publish Bool(true) to /explorer_start to begin')
        self.get_logger().info('=' * 60)

    # ── callbacks ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                         1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.current_pose = np.array([p.x, p.y, yaw])

    def _map_cb(self, msg):
        """
        Process map at most once per map_update_interval seconds.
        Transposes the grid to match the original repo's (W,H) convention.
        """
        import time as _time
        now = _time.monotonic()
        if now - self.last_map_time < self.map_update_interval:
            return
        self.last_map_time = now

        raw = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        # Transpose to (W, H) — same as original repo's env.T convention
        self.map       = raw.T
        self.origin    = [msg.info.origin.position.x,
                          msg.info.origin.position.y]
        self.resolution = msg.info.resolution
        self.svc.set(self.map, self.resolution, self.origin)

    def _status_cb(self, msg: String):
        """Handle goal-reached signals from dwa_planner or control_tb."""
        txt = msg.data.lower()
        if 'goal reached' in txt or 'reached' in txt:
            if not self.goal_reached:
                self.get_logger().info('[EXPLORE] goal reached signal received')
                self.goal_reached = True

    def _start_cb(self, msg: Bool):
        if msg.data:
            self.exploring    = True
            self.goal_reached = True   # kick off immediately
            self.get_logger().info('[EXPLORE] ▶ exploration STARTED')
            self._pub_status('exploring')
        else:
            self.exploring = False
            self.goal      = None
            self.get_logger().info('[EXPLORE] ⏸ exploration PAUSED')
            self._pub_status('paused')

    # ── main tick ──────────────────────────────────────────────────────────

    def _explore_tick(self):
        """
        Called at explore_interval Hz.
        Only acts when exploring=True AND goal_reached=True (ready for next goal).
        Also handles distance-based goal-reached detection as a fallback.
        """
        if not self.exploring:
            return
        if self.current_pose is None or self.map is None:
            return

        # distance-based goal reached fallback
        if self.goal is not None and not self.goal_reached:
            dist = math.hypot(self.goal[0] - self.current_pose[0],
                              self.goal[1] - self.current_pose[1])
            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f'[EXPLORE] goal reached by distance ({dist:.2f}m)')
                self.goal_reached = True

        if not self.goal_reached:
            return   # still driving to current goal

        # ready — find frontiers and send next goal
        frontiers = self._get_frontiers(self.map)

        if not frontiers:
            self.get_logger().info(
                '[EXPLORE] ✅ No frontiers remaining — exploration complete!')
            self._pub_status('complete')
            self.exploring = False
            return

        self._frontier_publish(frontiers)
        clusters = self._cluster_frontiers(frontiers)
        self._publish_clusters(clusters)
        self._explore(clusters)

    # ── frontier detection ─────────────────────────────────────────────────

    def _get_frontiers(self, gridmap: np.ndarray) -> list:
        """
        Find all frontier cells.
        gridmap shape: (W, H) — transposed convention from original repo.
        A cell is a frontier if it is FREE (0) and has at least one
        UNKNOWN (-1) neighbour in 8-connectivity.
        Returns list of (col, row) grid indices.
        """
        frontiers = []
        W, H = gridmap.shape

        for i in range(W):
            for j in range(H):
                if gridmap[i, j] != 0:
                    continue
                # check 8 neighbours
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        ni, nj = i + di, j + dj
                        if 0 <= ni < W and 0 <= nj < H:
                            if gridmap[ni, nj] == -1:
                                frontiers.append((i, j))
                                break
                    else:
                        continue
                    break
        return frontiers

    # ── clustering (DBSCAN — from original repo) ──────────────────────────

    def _cluster_frontiers(self, frontier_points: list) -> dict:
        """
        DBSCAN clustering — identical parameters to original repo.
        eps=3 cells, min_samples=4.
        Returns {label: [list of (col,row) points]}, noise points excluded.
        """
        if not frontier_points:
            return {}

        pts    = np.array(frontier_points)
        dbscan = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
        dbscan.fit(pts)
        labels = dbscan.labels_

        clusters = {}
        for label, point in zip(labels, pts):
            if label == -1:
                continue   # noise
            clusters.setdefault(int(label), []).append(tuple(point))
        return clusters

    # ── exploration logic ──────────────────────────────────────────────────

    def _explore(self, clusters: dict):
        if not clusters:
            self.get_logger().warn('[EXPLORE] no clusters — reclustering')
            self._recluster_and_explore()
            return

        self.clusters = clusters
        min_distances = self._calculate_min_distances(clusters)
        max_length    = self._calculate_max_lengths(clusters)
        target_label, target_points = self._score_and_select(
            min_distances, max_length, clusters)

        if target_points:
            best_vp = self._maximum_information_gain(target_points)
            if best_vp is not None:
                self.get_logger().info(f'[EXPLORE] best viewpoint grid={best_vp}')
                self._navigate_to_viewpoint(best_vp, target_label)
            else:
                self.get_logger().warn('[EXPLORE] no valid viewpoint in cluster')
        else:
            self.get_logger().warn('[EXPLORE] target cluster empty')
            self._recluster_and_explore()

    def _recluster_and_explore(self):
        frontiers = self._get_frontiers(self.map)
        if not frontiers:
            self.get_logger().info('[EXPLORE] no new frontiers — complete')
            self._pub_status('complete')
            self.exploring = False
            return
        clusters = self._cluster_frontiers(frontiers)
        if clusters:
            self._publish_clusters(clusters)
            self._explore(clusters)
        else:
            self.get_logger().info('[EXPLORE] clustering failed')

    # ── scoring (from original repo) ──────────────────────────────────────

    def _calculate_max_lengths(self, clusters: dict) -> dict:
        return {label: len(pts) for label, pts in clusters.items()}

    def _calculate_travel_cost(self, pos_a, pos_b) -> float:
        return float(np.linalg.norm(np.array(pos_a) - np.array(pos_b)))

    def _calculate_min_distances(self, clusters: dict) -> dict:
        if self.current_pose is None:
            return {}
        robot_xy  = self.current_pose[:2]
        min_dists = {}
        for label, pts in clusters.items():
            min_d = np.inf
            for pt in pts:
                world = self.svc.map_to_position(pt)
                d     = self._calculate_travel_cost(robot_xy, world)
                if d < min_d:
                    min_d = d
            min_dists[label] = min_d
        return min_dists

    def _score_and_select(self, min_distances, max_length, clusters):
        """
        Score = length_weight × cluster_size + distance_weight × min_distance
        Select cluster with MAXIMUM score.
        (Identical to original repo's calculate_scores_and_select_target)
        """
        if not clusters or not min_distances or not max_length:
            return None, []

        scores = {}
        for label, pts in clusters.items():
            if pts and label in min_distances and label in max_length:
                scores[label] = (max_length[label]      * self.length_weight +
                                 min_distances[label]   * self.distance_weight)

        if not scores:
            return None, []

        self.sorted_clusters = sorted(scores.items(),
                                      key=lambda x: x[1], reverse=True)
        best_label  = max(scores, key=scores.get)
        best_points = self.clusters.get(best_label, [])
        self.get_logger().info(
            f'[EXPLORE] {len(scores)} clusters scored — '
            f'best label={best_label} size={len(best_points)} '
            f'score={scores[best_label]:.2f}')
        return best_label, best_points

    # ── maximum information gain (from original repo) ─────────────────────

    def _calculate_entropy(self, grid: np.ndarray, x: int, y: int,
                           radius: int) -> float:
        """
        Shannon entropy of free/occupied/unknown cell counts in a
        (2r+1)×(2r+1) window around (x, y).
        grid shape: (W, H) transposed convention.
        """
        counts = {'free': 0, 'occupied': 0, 'unknown': 0}
        W, H  = grid.shape
        x_min, x_max = max(0, x - radius), min(W, x + radius + 1)
        y_min, y_max = max(0, y - radius), min(H, y + radius + 1)

        total = 0
        for i in range(x_min, x_max):
            for j in range(y_min, y_max):
                v = grid[i, j]
                if   v == -1: counts['unknown']  += 1
                elif v ==  0: counts['free']      += 1
                else:         counts['occupied']  += 1
                total += 1

        if total == 0:
            return 0.0
        probs = [c / total for c in counts.values()]
        return float(stats.entropy(probs))

    def _maximum_information_gain(self, cluster_points: list):
        """
        Return the grid point (col, row) with the highest local entropy.
        Identical to original repo's maximum_information_gain().
        """
        if self.map is None or self.resolution is None:
            return None

        r          = int(self.svc._thresh / self.resolution)
        max_ent    = -np.inf
        best_point = None

        for pt in cluster_points:
            x, y    = int(pt[0]), int(pt[1])
            entropy = self._calculate_entropy(self.map, x, y, r)
            if entropy > max_ent:
                max_ent    = entropy
                best_point = pt

        if best_point is None:
            self.get_logger().warn('[EXPLORE] no valid frontier — exploration complete')
        return best_point

    # ── navigation (from original repo) ───────────────────────────────────

    def _navigate_to_viewpoint(self, grid_point, cluster_id: int,
                               attempt: int = 0):
        """
        Convert grid_point → world, validate, publish goal.
        Falls back to next point in same cluster, then next cluster.
        """
        world_pt = self.svc.map_to_position(grid_point)

        if not self.svc.is_valid_frontier(world_pt):
            self.get_logger().warn(
                f'[EXPLORE] attempt {attempt+1}: '
                f'({world_pt[0]:.2f},{world_pt[1]:.2f}) invalid — trying next')

            if attempt < self.max_nav_attempts:
                nxt = self._select_next_target_point(cluster_id, grid_point)
                if nxt is not None:
                    self._navigate_to_viewpoint(nxt, cluster_id, attempt + 1)
                    return
                self.get_logger().warn('[EXPLORE] cluster exhausted')
            else:
                self.get_logger().warn('[EXPLORE] max attempts — next cluster')

            # move to next cluster
            nxt_pt, nxt_label = self._select_next_cluster_viewpoint(cluster_id)
            if nxt_pt is not None:
                self._navigate_to_viewpoint(nxt_pt, nxt_label)
            else:
                self.get_logger().error('[EXPLORE] no more clusters')
            return

        # valid point — publish goal
        self.get_logger().info(
            f'[EXPLORE] ➡  goal ({world_pt[0]:.2f},{world_pt[1]:.2f})')
        self._publish_goal(world_pt)
        self._publish_best_viewpoint_marker(world_pt)

        # remove used cluster so it isn't re-chosen
        if cluster_id in self.clusters:
            del self.clusters[cluster_id]

    def _select_next_target_point(self, cluster_id: int, bad_point):
        """Remove bad_point from cluster, re-run info gain on remainder."""
        pts = list(self.clusters.get(cluster_id, []))
        pts = [p for p in pts if not np.array_equal(p, bad_point)]
        self.clusters[cluster_id] = pts
        if not pts:
            return None
        return self._maximum_information_gain(pts)

    def _select_next_cluster_viewpoint(self, current_label: int):
        """Remove current cluster from sorted list, pick next best."""
        if self.sorted_clusters is None:
            return None, None
        self.sorted_clusters = [c for c in self.sorted_clusters
                                 if c[0] != current_label]
        if not self.sorted_clusters:
            self.get_logger().warn('[EXPLORE] all clusters visited')
            return None, None
        nxt_label = self.sorted_clusters[0][0]
        nxt_pts   = self.clusters.get(nxt_label, [])
        if not nxt_pts:
            return None, None
        return self._maximum_information_gain(nxt_pts), nxt_label

    # ── publish helpers ────────────────────────────────────────────────────

    def _publish_goal(self, world_xy):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        msg.pose.position.x    = float(world_xy[0])
        msg.pose.position.y    = float(world_xy[1])
        msg.pose.orientation.w = 1.0
        self.goal          = (float(world_xy[0]), float(world_xy[1]))
        self.goal_reached  = False
        self.goal_pub.publish(msg)

    def _frontier_publish(self, frontier_points: list):
        """Publish all frontier cells as PoseArray."""
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        for pt in frontier_points:
            world = self.svc.map_to_position(pt)
            p = Pose()
            p.position.x = float(world[0])
            p.position.y = float(world[1])
            pa.poses.append(p)
        self.frontier_pub.publish(pa)

    def _publish_clusters(self, clusters: dict):
        """
        Publish DBSCAN clusters as coloured MarkerArray.
        Each cluster gets a random colour + text label — identical to
        the original repo's publish_clusters().
        """
        ma  = MarkerArray()
        now = self.get_clock().now().to_msg()

        # delete old
        del_m = Marker()
        del_m.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        del_m.header.stamp    = now
        del_m.action          = Marker.DELETEALL
        del_m.id              = 0
        ma.markers.append(del_m)

        rng = np.random.default_rng(seed=42)   # fixed seed → stable colours

        for idx, (label, pts) in enumerate(clusters.items()):
            r_, g_, b_ = rng.random(3).tolist()
            frame      = 'world_enu' if self.use_sim else 'odom'

            # coloured POINTS marker
            pm = Marker()
            pm.header.frame_id = frame
            pm.header.stamp    = now
            pm.ns              = 'frontier_clusters'
            pm.id              = label
            pm.type            = Marker.POINTS
            pm.action          = Marker.ADD
            pm.scale.x         = 0.08
            pm.scale.y         = 0.08
            pm.color.r         = r_
            pm.color.g         = g_
            pm.color.b         = b_
            pm.color.a         = 1.0
            pm.pose.orientation.w = 1.0
            # lifetime = 0 → permanent in RViz

            world_pts = [self.svc.map_to_position(p) for p in pts]
            for wp in world_pts:
                pt = Point()
                pt.x = float(wp[0]); pt.y = float(wp[1]); pt.z = 0.0
                pm.points.append(pt)

            avg_x = float(np.mean([w[0] for w in world_pts]))
            avg_y = float(np.mean([w[1] for w in world_pts]))

            # text label
            tm = Marker()
            tm.header.frame_id = frame
            tm.header.stamp    = now
            tm.ns              = 'frontier_cluster_labels'
            tm.id              = label
            tm.type            = Marker.TEXT_VIEW_FACING
            tm.action          = Marker.ADD
            tm.pose.position.x = avg_x
            tm.pose.position.y = avg_y + 0.3
            tm.pose.position.z = 0.3
            tm.pose.orientation.w = 1.0
            tm.scale.z         = 0.25
            tm.color.r         = 0.0
            tm.color.g         = 0.0
            tm.color.b         = 1.0
            tm.color.a         = 1.0
            tm.text            = f'C{label}({len(pts)})'
            # lifetime = 0 → permanent in RViz

            ma.markers.append(pm)
            ma.markers.append(tm)

        self.cluster_pub.publish(ma)

    def _publish_best_viewpoint_marker(self, world_xy):
        """Green sphere + 'Best Viewpoint' text — from original repo."""
        frame = 'world_enu' if self.use_sim else 'odom'
        now   = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header.frame_id = frame
        sphere.header.stamp    = now
        sphere.ns              = 'viewpoint_markers'
        sphere.id              = 0
        sphere.type            = Marker.SPHERE
        sphere.action          = Marker.ADD
        sphere.pose.position.x = float(world_xy[0])
        sphere.pose.position.y = float(world_xy[1])
        sphere.pose.position.z = 0.0
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.2; sphere.scale.y = 0.2; sphere.scale.z = 0.2
        sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0
        sphere.color.a = 1.0
        # lifetime = 0 → permanent in RViz
        self.marker_pub.publish(sphere)

        txt = Marker()
        txt.header.frame_id = frame
        txt.header.stamp    = now
        txt.ns              = 'viewpoint_markers'
        txt.id              = 1
        txt.type            = Marker.TEXT_VIEW_FACING
        txt.action          = Marker.ADD
        txt.pose.position.x = float(world_xy[0])
        txt.pose.position.y = float(world_xy[1]) + 0.4
        txt.pose.position.z = 0.5
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.3
        txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 0.0
        txt.color.a = 1.0
        txt.text    = 'Best Viewpoint'
        txt.lifetime.nanosec = int(5e9)
        self.marker_pub.publish(txt)

    def _clear_clusters_and_frontiers(self):
        self._clear_clusters()
        self._clear_frontiers()

    def _clear_clusters(self):
        ma = MarkerArray()
        m  = Marker()
        m.action = Marker.DELETEALL
        m.id     = 0
        ma.markers.append(m)
        self.cluster_pub.publish(ma)
        self.clusters.clear()

    def _clear_frontiers(self):
        pa = PoseArray()
        pa.header.stamp    = self.get_clock().now().to_msg()
        pa.header.frame_id = 'world_enu' if self.use_sim else 'odom'
        self.frontier_pub.publish(pa)

    def _pub_status(self, text: str):
        m = String(); m.data = text
        self.status_pub.publish(m)


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
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
