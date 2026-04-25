#!/usr/bin/env python3
"""
phase1_real.launch.py  —  Phase 1 on Real TurtleBot4  (Full DWA Stack)
=======================================================================

Runs the complete Phase 1 simulation stack on the real TurtleBot4:
  - mapping_node    from online_motion_planning  (TF-based LiDAR mapper)
  - planning_node   from online_motion_planning  (RRT* global planner)
  - dwa_planner     from handson_planning        (DWA local planner — same as sim)
  - static TF publishers for world_enu->odom and base_link->rplidar_link
  - EKF localisation node (optional)

This gives the full simulation experience on the real robot:
  /dwa_trajectories  — blue fan of candidate trajectories (RViz MarkerArray)
  /local_path        — green best DWA arc (RViz Path)
  /plan              — red RRT* global path (RViz Path)
  /rrt_tree          — grey RRT* tree (RViz MarkerArray)

HOW TO RUN
----------
  Step 1 — source workspace:
    cd ~/ros2_ws && source install/setup.bash

  Step 2 — launch:
    ros2 launch handson_planning phase1_real.launch.py

  Step 3 — RViz2 setup (Fixed Frame = odom):
    Map           -> /projected_map
    Path          -> /plan                 (red  — RRT* global path)
    Path          -> /local_path           (cyan — DWA 3s arc)
    MarkerArray   -> /rrt_tree             (grey — RRT* tree)
    MarkerArray   -> /dwa_trajectories     (blue fan + green best)
    Odometry      -> /odom

  Step 4 — click "2D Goal Pose" in RViz to send a goal.
           Wait ~5-10s after launch before clicking so the map has time to build.

KEY DIFFERENCES FROM SIMULATOR LAUNCH
--------------------------------------
  use_sim=False             -> /odom, /scan, TwistStamped /cmd_vel
  map_frame='odom'          -> real TF uses odom (not world_enu)
  invert_scan_angles=False  -> real RPLidar is upright (not inverted like Stonefish)
  lidar_yaw=0.0             -> TF handles LiDAR orientation (no manual offset)
  v_max=0.15 m/s            -> conservative for real corridors
  robot_radius=0.10 m       -> inflation radius (must match between planner + DWA)
  w_clearance=0.8           -> higher clearance weight for real obstacles

TROUBLESHOOTING
---------------
  Robot not moving?
    ros2 topic echo /cmd_vel          -> should see TwistStamped at 10 Hz
    ros2 topic echo /plan --no-arr    -> should have waypoints after goal set

  Map is empty?
    ros2 topic hz /projected_map      -> should be ~2 Hz
    ros2 topic hz /scan               -> should be ~7-10 Hz

  DWA all blocked / robot stuck?
    -> Robot rotates and publishes /replan_request
    -> planning_node replans automatically (5s cooldown)

  TF error in mapping_node?
    ros2 run tf2_tools view_frames
"""

from launch               import LaunchDescription
from launch.actions       import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions    import IfCondition
from launch_ros.actions   import Node

# IMPORTANT: robot_radius must be identical in planning_node and dwa_planner.
# If they differ, DWA rejects trajectories the global planner thinks are safe.
ROBOT_RADIUS = 0.08   # m — inflation radius for both planner and DWA


def generate_launch_description():

    # ── LAUNCH ARGUMENTS ─────────────────────────────────────────────────
    arg_use_ekf = DeclareLaunchArgument(
        'use_ekf',
        default_value='false',
        description='true = run EKF localisation for better odometry accuracy'
    )
    arg_lidar_yaw = DeclareLaunchArgument(
        'lidar_yaw',
        default_value='0.0',
        description='LiDAR yaw offset (rad). 0.0 = let TF handle it.'
    )

    use_ekf   = LaunchConfiguration('use_ekf')
    lidar_yaw = LaunchConfiguration('lidar_yaw')

    # ── STATIC TF: world_enu -> odom (identity) ──────────────────────────
    # Maps world_enu to odom so any node that expects world_enu still works.
    static_tf_world = Node(
        package    = 'tf2_ros',
        executable = 'static_transform_publisher',
        name       = 'world_enu_to_odom',
        arguments  = ['0', '0', '0', '0', '0', '0', 'world_enu', 'odom'],
    )

    # ── STATIC TF: base_link -> rplidar_link ─────────────────────────────
    # Provides the TF chain that mapping_node uses to look up LiDAR world pose.
    # The robot's URDF already broadcasts this as a zero-offset static TF.
    # Publishing here with the measured physical offset gives more accurate mapping.
    # x=-0.249m y=-0.055m z=0.138m, ~56deg yaw rotation quaternion.
    static_tf_lidar = Node(
        package    = 'tf2_ros',
        executable = 'static_transform_publisher',
        name       = 'base_to_rplidar',
        arguments  = [
            '--x',  '-0.249',
            '--y',  '-0.055',
            '--z',   '0.138',
            '--qx', '-0.007',
            '--qy', '-0.009',
            '--qz',  '0.830',
            '--qw',  '0.558',
            '--frame-id',       'base_link',
            '--child-frame-id', 'rplidar_link',
        ],
    )

    # ── EKF LOCALISATION (optional) ───────────────────────────────────────
    # Fuses wheel odometry + IMU for a smoother pose estimate.
    # Only launched when use_ekf:=true is passed on the command line.
    ekf_node = Node(
        package    = 'online_motion_planning',
        executable = 'localisation_node',
        name       = 'localisation_node',
        output     = 'screen',
        parameters = [{
            'odom_frame':             'odom',
            'base_frame':             'base_link',
            'wheel_left_joint_name':  'wheel_left_joint',
            'wheel_right_joint_name': 'wheel_right_joint',
            'use_sim':                False,
        }],
        condition  = IfCondition(use_ekf),
    )

    # ── MAPPING NODE ──────────────────────────────────────────────────────
    # Probabilistic occupancy grid from LiDAR scans using log-odds sensor model.
    # Publishes /projected_map at 2 Hz.
    mapping_node = Node(
        package    = 'online_motion_planning',
        executable = 'mapping_node',
        name       = 'mapping_node',
        output     = 'screen',
        parameters = [{
            'use_sim':            False,
            'map_frame':          'odom',   # real robot TF frame
            'grid_size':          20.0,     # 20x20 m map
            'grid_resolution':    0.05,     # 5 cm/cell
            'l_occ':              0.85,     # log-odds hit increment
            'l_free':             0.30,     # log-odds miss decrement
            'lidar_yaw':          lidar_yaw,
            'invert_scan_angles': False,    # real RPLidar: upright, not inverted
        }],
    )

    # ── PLANNING NODE (RRT*) ──────────────────────────────────────────────
    # Runs RRT* in background thread, checks path validity every 1s,
    # replans if a waypoint in the path is blocked by the live map.
    planning_node = Node(
        package    = 'online_motion_planning',
        executable = 'planning_node',
        name       = 'planning_node',
        output     = 'screen',
        parameters = [{
            'use_sim':          False,
            'use_saved_map':    False,      # live map from mapping_node
            'robot_radius':     ROBOT_RADIUS,
            'rrt_max_iter':     5000,
            'rrt_step_size':    0.3,        # m per extension step
            'replan_interval':  2.0,        # check every 2s, replan only if truly blocked
      'waypoint_tol':     0.25,
        }],
    )

    # ── DWA LOCAL PLANNER ─────────────────────────────────────────────────
    # Same DWA node as in simulation. Samples 8x21=168 trajectories at 10 Hz,
    # scores them on heading/clearance/velocity, publishes TwistStamped to /cmd_vel.
    # Publishes /dwa_trajectories and /local_path for RViz visualisation.
    # Sends /replan_request (Int32=1) to planning_node when all paths blocked.
    dwa_node = Node(
        package    = 'handson_planning',
        executable = 'dwa_planner',
        name       = 'dwa_planner',
        output     = 'screen',
        parameters = [{
            'use_sim':         False,       # -> TwistStamped on /cmd_vel
            'map_frame':       'odom',      # so markers use odom frame from the start
            'robot_radius':    0.05,  # DWA uses smaller inflation than global planner (0.08m)
                                    # so it can navigate near walls the global plan approved
            'v_max':           0.15,        # conservative: 15 cm/s
            'w_max':           1.0,         # rad/s
            'a_w_max':         1.5,
            'v_samples':       8,
            'w_samples':       21,
            'sim_time':        3.0,         # 3s lookahead arc
            'sim_steps':       20,
      'waypoint_tol':    0.25,
      'w_heading':       2.5,   # heading is primary — go toward waypoint
      'w_clearance':     0.3,   # lower: corners drag all scores down at high weight
      'w_velocity':      0.6,
            'dt':              0.1,         # 10 Hz control loop
            'k_w':             2.0,         # Phase A rotation P-gain
            'angle_threshold': 0.50,        # rad (29deg) — only pure-rotate above this
      'angle_exit_ratio': 0.70,       # hysteresis to avoid rotate/move oscillation
      'path_interp_spacing': 0.10,    # densify sparse RRT* waypoints for smoother tracking
      'waypoint_lookahead_dist': 0.35,
      'stuck_lin_speed_eps':    0.02,   # m/s: forward speed below this = "not moving"
      'stuck_ticks_before_replan': 25,  # 25 ticks @ 10Hz = 2.5s of no progress → replan
        }],
    )

    return LaunchDescription([
        arg_use_ekf,
        arg_lidar_yaw,
        static_tf_world,
        static_tf_lidar,
        ekf_node,
        mapping_node,
        planning_node,
        dwa_node,
    ])
