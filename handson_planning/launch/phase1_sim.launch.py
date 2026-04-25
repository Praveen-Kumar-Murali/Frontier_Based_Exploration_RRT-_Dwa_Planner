#!/usr/bin/env python3
"""
phase1_sim.launch.py  —  Phase 1: Live Mapping + RRT* Global + DWA Local


HOW TO RUN
----------
  Terminal 1 — start simulator:
    cd ~/ros2_ws && source install/setup.bash
    ros2 launch turtlebot_simulation turtlebot_hoi_circuit1.launch.py

  Terminal 2 — start planning stack:
    cd ~/ros2_ws && source install/setup.bash
    ros2 launch handson_planning phase1_sim.launch.py

  RViz2: click "2D Goal Pose" anywhere inside the circuit.

NODES
-----
  mapping_node    — live LiDAR occupancy grid → /projected_map
  global_planner  — RRT* path planner → /plan  (replans when blocked)
  dwa_planner     — DWA local controller → /turtlebot/cmd_vel
"""

from launch import LaunchDescription
from launch_ros.actions import Node

# Circuit corridor analysis from mesh files:
#   Straight corridor width : ~1.17 m  (segments spaced 1.18m, wall 0.008m thick)
#   S-bend inner width      : ~0.418 m  ← narrowest passage in the circuit
#   Robot base width        : ~0.35 m  (wheel span ±0.115m + body overhang)
#   Max safe clearance each side: (0.418 - 0.35) / 2 = 0.034 m
#
# ROBOT_RADIUS must be < 0.034m to allow RRT* to plan through the S-bend.
# We use 0.10m for open corridors (conservative safe margin) but RRT* will
# fail on S-bend. If S-bend is needed, reduce to 0.03m.
# NOTE: arm hitting walls is a separate problem — the arm swings in the
# horizontal plane above the LiDAR scan height so the map cannot see it.
# Solution: fold/retract the arm before navigating, or avoid tight corners.
ROBOT_RADIUS = 0.17   # safe for wide corridors; reduce to 0.03 for S-bend


def generate_launch_description():

    # ── NODE 1: mapping_node 
    mapping_node = Node(
        package    = 'handson_planning',
        executable = 'mapping_node',
        name       = 'mapping_node',
        output     = 'screen',
        parameters = [{
            'use_sim':            True,
            'map_frame':          'world_enu',
            'grid_size':          20.0,
            'grid_resolution':    0.05,
            'l_occ':              0.85,   # log-odds increase per hit — needs 4 hits to confirm wall
            'l_free':             0.35,   # stronger free-ray erasure — cleans phantom walls faster
            'lidar_yaw':          3.1416,
            'invert_scan_angles': True,
        }],
    )

    # ── NODE 2: global_planner — RRT* + periodic replanning ──────────────
    global_planner_node = Node(
        package    = 'handson_planning',
        executable = 'global_planner',
        name       = 'global_planner',
        output     = 'screen',
        parameters = [{
            'use_sim':         True,
            'use_saved_map':   False,
            'robot_radius':    ROBOT_RADIUS,
            'rrt_max_iter':    5000,
            'rrt_step_size':   0.3,
            'replan_interval':      2.0,
            'waypoint_tol':         0.25,
            'replan_confirmations': 3,
            'replan_cooldown':      5.0,
        }],
    )

    # ── NODE 3: dwa_planner — DWA local planner
    dwa_planner_node = Node(
        package    = 'handson_planning',
        executable = 'dwa_planner',
        name       = 'dwa_planner',
        output     = 'screen',
        parameters = [{
            'use_sim':         True,
            'map_frame':       'world_enu',
            'robot_radius':    0.07,   # MUST be < global planner (0.10m).
                                       # Global planner inflates 2 cells (0.10m) — robot sits inside that zone.
                                       # If DWA also inflates 2 cells it blocks its own position.
                                       # DWA at 1 cell (0.05m) can still move in the approved corridor.
            'v_max':           0.12,   # reduced: 0.18 too fast near walls, body clips before
                                       # DWA can react in one 0.1s control cycle
            'w_max':           1.2,
            'a_w_max':         1.5,
            'v_samples':       8,
            'w_samples':       21,
            'sim_time':        1.5,    # short lookahead — long arcs hit far walls
            'sim_steps':       15,     # dt_step = 0.1s per step
            'waypoint_tol':    0.25,
            'w_heading':       2.5,    # heading is the primary driver
            'w_clearance':     0.4,    # moderate: penalise wall proximity without freezing
            'w_velocity':      0.5,
            'dt':              0.1,
            'k_w':             1.4,
            'angle_threshold': 0.65,   # sim: slightly more tolerant than real (0.50)
            'angle_exit_ratio': 0.70,  # hysteresis: exit rotate-mode below 0.70×threshold
            'path_interp_spacing':     0.10,   # densify RRT* waypoints → smoother tracking
            'waypoint_lookahead_dist': 0.35,   # steer to next WP when this close to current
            'stuck_lin_speed_eps':     0.02,   # m/s: below this = "not moving forward"
            'stuck_ticks_before_replan': 25,   # 2.5s no progress → request replan
            'plan_max_age':    3.0,    # ignore /plan messages older than this many seconds
        }],
    )

    # ── NODE 4: frontier_explorer — autonomous exploration ────────────────
    # START: publish Bool(True) to /explorer_start  to begin exploration
    # STOP:  publish Bool(False) to /explorer_start to pause
    # Or just manually click "2D Goal Pose" in RViz for manual goals.
    frontier_explorer_node = Node(
        package    = 'handson_planning',
        executable = 'frontier_explorer',
        name       = 'frontier_explorer',
        output     = 'screen',
        parameters = [{
            'use_sim':              True,
            'distance_threshold':   0.3,   # validity check radius (m)
            'map_update_interval':  1.0,   # process map at most every 1s
            'explore_interval':     1.0,   # frontier scan rate (s)
            'length_weight':        0.6,   # score weight: cluster size
            'distance_weight':      0.4,   # score weight: distance to cluster
            'entropy_radius':       5,     # info-gain window radius (cells)
            'dbscan_eps':           3.0,   # DBSCAN neighbourhood (cells)
            'dbscan_min_samples':   4,     # DBSCAN min points per cluster
            'goal_tolerance':       0.40,  # goal reached distance (m)
            'max_nav_attempts':     10,    # attempts before skipping cluster
        }],
    )

    return LaunchDescription([
        mapping_node,
        global_planner_node,
        dwa_planner_node,
        frontier_explorer_node,
    ])
