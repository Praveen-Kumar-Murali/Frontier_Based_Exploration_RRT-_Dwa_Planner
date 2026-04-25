#!/usr/bin/env python3
"""
mapping_node.py — Probabilistic Occupancy Grid Mapping


This node builds a 2-D occupancy grid in real time from LiDAR range
measurements using a log-odds sensor model.

LIDAR POSE ESTIMATION:
  The world-frame position of the LiDAR at each scan timestamp is obtained
  via a TF tree lookup: lookup_transform(map_frame, laser_frame, scan_time).
  When the EKF localisation node is active, the TF tree reflects the
  filtered robot pose, producing more accurate scan registration than raw
  odometry. If the TF lookup fails (e.g. at startup), the node falls back
  to odometry plus a fixed sensor offset.

BEAM ANGLE CONVENTION:
  The scan angle direction depends on the physical LiDAR mounting:
    Stonefish simulator (upside-down mount):
      invert_scan_angles:=true  →  beam_angle = l_yaw − scan_angle
    Real TurtleBot4 (upright mount):
      invert_scan_angles:=false →  beam_angle = l_yaw + scan_angle

OCCUPANCY UPDATE:
  Each cell stores a log-odds value updated by inverse sensor model rules:
    Beam endpoint (occupied hit): log-odds += l_occ  (+0.85)
    Intermediate cells (free ray): log-odds += l_free (−0.40)
  Cells confirmed as walls (log-odds > 2.0) are protected from erasure
  by subsequent free-ray updates.

Parameters:
  use_sim             — topic name selection (simulator vs real robot)
  map_frame           — coordinate frame for published map
  lidar_yaw           — LiDAR yaw mount offset in radians
  invert_scan_angles  — beam angle sign: true=simulator, false=real robot
  grid_size           — map coverage in metres (square)
  grid_resolution     — cell size in metres per cell
  l_occ, l_free       — log-odds update magnitudes
"""

import math
import rclpy
import rclpy.time
import rclpy.duration
import numpy as np
import tf2_ros

from rclpy.node       import Node
from nav_msgs.msg     import Odometry, OccupancyGrid
from sensor_msgs.msg  import LaserScan



class GridMap:

    """
    Probabilistic occupancy grid using log-odds.
    Sticky walls: cells with log-odds > 2.0 cannot be erased by free rays.
    """

    LMAX =  6.91
    LMIN = -6.91

    def __init__(self, cell_size=0.05, map_size=20.0):
        self.cell_size = cell_size
        n = int(map_size / cell_size)
        self.grid   = np.zeros((n, n), dtype=np.float32)
        self.origin = np.array([-map_size / 2.0, -map_size / 2.0])
        self.height, self.width = self.grid.shape

    def world_to_cell(self, wx, wy):
        col = int((wx - self.origin[0]) / self.cell_size)
        row = int((wy - self.origin[1]) / self.cell_size)
        if 0 <= col < self.width and 0 <= row < self.height:
            return col, row
        return None

    def update_cell(self, col, row, delta):
        if 0 <= col < self.width and 0 <= row < self.height:
            self.grid[row, col] = np.clip(
                self.grid[row, col] + delta,
                self.LMIN, self.LMAX
            )

    @staticmethod
    def bresenham(c0, r0, c1, r1):
        cells = []
        dc = abs(c1 - c0); dr = abs(r1 - r0)
        sc = 1 if c1 > c0 else -1
        sr = 1 if r1 > r0 else -1
        err = dc - dr
        c, r = c0, r0
        while True:
            cells.append((c, r))
            if c == c1 and r == r1:
                break
            e2 = 2 * err
            if e2 > -dr: err -= dr; c += sc
            if e2 <  dc: err += dc; r += sr
        return cells

    def cast_ray(self, lx, ly, angle, dist, l_occ, l_free, is_hit):
        start = self.world_to_cell(lx, ly)
        end   = self.world_to_cell(
            lx + dist * math.cos(angle),
            ly + dist * math.sin(angle)
        )
        if start is None:
            return
        self.update_cell(start[0], start[1], l_free)
        if end is None or start == end:
            return
        cells = self.bresenham(start[0], start[1], end[0], end[1])
        for c, r in cells[1:-1]:
            if 0 <= c < self.width and 0 <= r < self.height:
                if self.grid[r, c] < 2.0:
                    self.update_cell(c, r, l_free)
        if cells:
            c, r = cells[-1]
            if (c, r) != start:
                if is_hit:
                    self.update_cell(c, r, l_occ)
                else:
                    if 0 <= c < self.width and 0 <= r < self.height:
                        if self.grid[r, c] < 2.0:
                            self.update_cell(c, r, l_free)

    def to_ros_array(self):
        out = np.full(self.grid.shape, -1, dtype=np.int8)
        out[self.grid >  3.0] = 100   # wall: needs 4+ hits to confirm — reduces phantom walls
        out[self.grid < -0.5] = 0     # free: needs strong evidence
        return out

    def get_origin(self):
        return self.origin.copy()



class OccupancyGridNode(Node):


    def __init__(self):
        super().__init__('mapping_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('use_sim',          True)
        self.declare_parameter('map_frame',        'world_enu')
        self.declare_parameter('grid_size',         20.0)
        self.declare_parameter('grid_resolution',    0.05)
        self.declare_parameter('l_occ',              0.85)
        self.declare_parameter('l_free',             0.40)
        self.declare_parameter('lidar_yaw',           3.1416)
        self.declare_parameter('invert_scan_angles', True)

        use_sim        = self.get_parameter('use_sim').value
        self.map_frame = self.get_parameter('map_frame').value
        gsize          = self.get_parameter('grid_size').value
        res            = self.get_parameter('grid_resolution').value
        self.l_occ        =  abs(self.get_parameter('l_occ').value)
        self.l_free       = -abs(self.get_parameter('l_free').value)
        self.lidar_yaw     = self.get_parameter('lidar_yaw').value
        self.invert_angles = self.get_parameter('invert_scan_angles').value

        if use_sim:
            self.odom_topic = '/turtlebot/odom'
            self.scan_topic = '/turtlebot/scan'
        else:
            self.odom_topic = '/odom'
            self.scan_topic = '/scan'

        # ── grid ──────────────────────────────────────────────────────────
        self.grid_map = GridMap(cell_size=res, map_size=gsize)

        # ── robot state (odom fallback) ────────────────────────────────────
        self.lidar_x   = -0.003
        self.lidar_y   =  0.0
        self.rx = 0.0; self.ry = 0.0; self.rtheta = 0.0
        self.odom_ready = False

        # ── scan state ────────────────────────────────────────────────────
        self.latest_scan = None
        self.scan_ready  = False
        self.scan_count  = 0

        # ── TF buffer (30s cache so bag timestamps don't expire) ──────────
        # When localisation_node is running, TF lookup returns EKF-filtered
        # lidar pose at scan time — this eliminates drift during replanning
        self.tf_buffer   = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=30)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(Odometry,  self.odom_topic, self.odom_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        # ── publisher ─────────────────────────────────────────────────────
        self.map_pub = self.create_publisher(OccupancyGrid, '/projected_map', 10)
        self.create_timer(0.5, self.loop)  # 10Hz — faster wall detection

        self.get_logger().info('=' * 58)
        self.get_logger().info(' OccupancyGridNode started')
        self.get_logger().info(
            f'   Map: {gsize}x{gsize}m @ {res}m/cell '
            f'({int(gsize/res)}x{int(gsize/res)} cells)'
        )
        self.get_logger().info(f'   Frame: {self.map_frame}')
        self.get_logger().info(
            f'   Topics: {self.odom_topic} + {self.scan_topic}'
        )
        self.get_logger().info(
            '   Beam angles: TF lookup at scan time'
        )
        self.get_logger().info(
            f'   invert_scan_angles: {self.invert_angles}  '
            + ('(use l_yaw - angle, Stonefish ENU)' if self.invert_angles
               else '(use l_yaw + angle, real robot odom)')
        )
        self.get_logger().info(
            '   Drift fix: EKF TF used when localisation_node active'
        )
        self.get_logger().info('=' * 58)

    # ── callbacks ──────────────────────────────────────────────────────────

    def odom_cb(self, msg):
        """Store latest odometry as fallback when TF lookup fails."""
        self.rx = msg.pose.pose.position.x
        self.ry = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.rtheta = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.odom_ready = True

    def scan_cb(self, msg):
        """Store latest scan for processing in main loop."""
        self.latest_scan = msg
        self.scan_ready  = True

    # ── main loop ──────────────────────────────────────────────────────────

    def loop(self):
        if self.scan_ready and self.odom_ready and self.latest_scan is not None:
            self.scan_ready = False
            self.process_scan(self.latest_scan)
        self.publish_map()

    # ── scan processing ────────────────────────────────────────────────────

    def process_scan(self, msg):
        """
        Register one LaserScan message into the occupancy grid.

        The LiDAR world position is obtained by querying the TF tree at the
        scan timestamp. If the transform is unavailable, the method falls back
        to odometry plus a fixed sensor offset. The beam direction is computed
        from the robot heading and the lidar_yaw parameter; the sign convention
        is selected by invert_scan_angles.

        Note: the minimum range threshold is set to 0.05 m internally rather
        than using msg.range_min, which the Stonefish simulator reports as
        0.20 m — a value that would discard valid close-range returns near walls.
        """
        scan_time   = rclpy.time.Time.from_msg(msg.header.stamp)
        laser_frame = msg.header.frame_id   # e.g. 'turtlebot/rplidar'

        # ── get lidar world pose ───────────────────────────────────────────
        tf_ok = False
        lx    = self.rx
        ly    = self.ry
        l_yaw = self.rtheta

        try:
            # Use TF for lidar POSITION only — more accurate than odom+offset
            # When EKF is active, this gives filtered position at scan time
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                laser_frame,
                scan_time,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            lx    = transform.transform.translation.x
            ly    = transform.transform.translation.y
            tf_ok = True

        except tf2_ros.TransformException:
            # TF not ready — use odom + fixed offset as fallback
            cos_r = math.cos(self.rtheta)
            sin_r = math.sin(self.rtheta)
            lx    = self.rx + cos_r * self.lidar_x - sin_r * self.lidar_y
            ly    = self.ry + sin_r * self.lidar_x + cos_r * self.lidar_y

        # Always compute beam angle from odometry + lidar_yaw parameter
        # This is reliable: Stonefish lidar has roll=-180 which makes
        # yaw extraction from TF quaternion unreliable
        l_yaw = self.rtheta + self.lidar_yaw

        # ── cast rays ─────────────────────────────────────────────────────
        rmax = msg.range_max
        rmin = 0.05   # NOT msg.range_min (which is 0.20m in Stonefish)

        hits = 0; free_rays = 0; skipped = 0

        for i, r in enumerate(msg.ranges):
            scan_angle = msg.angle_min + i * msg.angle_increment

            # Beam angle — use parameter to control convention
            # Stonefish (invert_scan_angles=true):  lidar upside-down → subtract
            # Real robot (invert_scan_angles=false): normal mount → add
            if self.invert_angles:
                beam_world = l_yaw - scan_angle   # ENU / Stonefish
            else:
                beam_world = l_yaw + scan_angle   # odom / real robot

            if math.isnan(r):
                skipped += 1
                continue
            elif math.isinf(r) or r >= rmax * 0.99:
                free_rays += 1
                self.grid_map.cast_ray(
                    lx, ly, beam_world,
                    rmax * 0.95,
                    self.l_occ, self.l_free,
                    is_hit=False
                )
            elif r < rmin:
                skipped += 1
            else:
                hits += 1
                self.grid_map.cast_ray(
                    lx, ly, beam_world, r,
                    self.l_occ, self.l_free,
                    is_hit=True
                )

        self.scan_count += 1

        if self.scan_count % 30 == 0:
            occ  = int((self.grid_map.grid >  1.0).sum())
            free = int((self.grid_map.grid < -0.2).sum())
            src  = 'TF' if tf_ok else 'odom'
            self.get_logger().info(
                f'Scan #{self.scan_count:4d} | {src} | '
                f'lidar=({lx:6.2f},{ly:6.2f}) '
                f'θ={math.degrees(l_yaw):6.1f}° | '
                f'hits={hits:3d} free={free_rays:3d} skip={skipped:2d} | '
                f'MAP occ={occ:5d} free={free:6d}'
            )

    # ── publish ────────────────────────────────────────────────────────────

    def publish_map(self):
        data = self.grid_map.to_ros_array()
        msg  = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = self.map_frame
        msg.info.resolution           = self.grid_map.cell_size
        msg.info.width                = self.grid_map.width
        msg.info.height               = self.grid_map.height
        msg.info.origin.position.x    = float(self.grid_map.origin[0])
        msg.info.origin.position.y    = float(self.grid_map.origin[1])
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data.flatten().tolist()
        self.map_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()