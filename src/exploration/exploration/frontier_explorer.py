#!/usr/bin/env python3
"""
High-performance 2D frontier-based autonomous exploration for ROS2 Humble.
Optimized for Jetson Orin NX (<8GB RAM).

Architecture:
  SLAM Toolbox (/map) -> FrontierExplorer (frontier detection + goal selection)
                          -> Nav2 NavigateToPose action (path planning + control)
                          -> /cmd_vel (motor)

State machine:
  WAITING -> INITIAL_SPIN -> EXPLORING <-> NAVIGATING -> RETURNING -> SAVING_MAP
    -> RANDOM_NAV -> RESETTING (if more cycles) -> WAITING ...
    -> IDLE (after all cycles complete)

Topics subscribed:
  /map              (nav_msgs/OccupancyGrid)   - from SLAM Toolbox
  /exploration/goal (geometry_msgs/PoseStamped) - external goals in IDLE mode

Topics published:
  /cmd_vel                    (geometry_msgs/Twist)      - only during initial spin
  /exploration/frontiers      (visualization_msgs/MarkerArray)
  /exploration/status         (std_msgs/String)
  /sim/reset                  (std_msgs/Empty)           - reset simulator between cycles

Action client:
  navigate_to_pose  (nav2_msgs/NavigateToPose)  - Nav2 goal execution
"""

import math
import os
import subprocess
import time
from collections import deque
from enum import IntEnum
from tempfile import NamedTemporaryFile

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, ColorRGBA, Empty
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException

try:
    import cv2
    _HAS_CV2 = True
except ImportError:
    _HAS_CV2 = False


class State(IntEnum):
    WAITING = 0       # Waiting for map / TF / Nav2
    INITIAL_SPIN = 1  # Optional in-place scan to seed the map
    EXPLORING = 2     # Picking next frontier
    NAVIGATING = 3    # Nav2 driving toward a frontier
    RETURNING = 4     # Navigating back to start position
    SAVING_MAP = 5    # Writing map to disk
    RANDOM_NAV = 6    # Navigating to random waypoints after exploration
    RESETTING = 7     # Resetting simulator for next cycle
    IDLE = 8          # Done; accepts external /exploration/goal


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        # ======================== Parameters ========================
        # Frontier detection
        self.declare_parameter('min_frontier_size', 5)       # min cells per cluster
        self.declare_parameter('robot_radius', 0.15)         # safety clearance (m)
        self.declare_parameter('min_frontier_dist', 0.3)     # ignore frontiers closer than this (m)

        # Frontier scoring
        self.declare_parameter('cost_weight', 1.0)           # distance penalty weight
        self.declare_parameter('gain_weight', 1.5)           # information gain weight
        self.declare_parameter('heading_weight', 0.9)        # penalty for large heading changes
        self.declare_parameter('u_turn_penalty', 1.2)        # extra penalty for near-180 deg goals
        self.declare_parameter('u_turn_angle', 2.1)          # radians, above this counts as U-turn
        self.declare_parameter('preferred_heading_limit', 1.2)  # prefer goals within this heading error
        self.declare_parameter('hard_heading_limit', 1.75)      # reject goals beyond this unless no fallback

        # Blacklist
        self.declare_parameter('blacklist_radius', 0.6)      # (m) merge radius
        self.declare_parameter('blacklist_clear_interval', 120.0)  # (s) auto-clear

        # Navigationmin_explored_cells
        self.declare_parameter('goal_timeout', 90.0)         # (s) per goal
        self.declare_parameter('replan_interval', 2.5)       # (s) re-evaluate while navigating
        self.declare_parameter('post_goal_pause', 0.3)       # (s) pause after reaching goal
        self.declare_parameter('enable_rescue_goals', True)
        self.declare_parameter('rescue_goal_radius', 0.6)
        self.declare_parameter('max_rescue_goals', 4)

        # Completion
        self.declare_parameter('no_frontier_threshold', 5)   # consecutive empty checks
        self.declare_parameter('min_explored_cells', 100)    # don't finish if map too small
        self.declare_parameter('enable_return_home', True)
        self.declare_parameter('map_save_path', '~/order_exploration/src/exploration/map/exploration_map')
        self.declare_parameter('map_cache_save_interval', 2.0)

        # Startup map seeding motion
        self.declare_parameter('allow_initial_spin', False)   # Keep false on real robot unless explicitly needed
        self.declare_parameter('initial_spin_duration', 8.0)  # (s)
        self.declare_parameter('spin_angular_vel', 0.4)       # (rad/s)
        self.declare_parameter('allow_initial_forward', True) # Seed map with short forward motion
        self.declare_parameter('initial_forward_duration', 1.5) # (s)
        self.declare_parameter('initial_forward_vel', 0.08)   # (m/s)

        # Multi-cycle
        self.declare_parameter('num_cycles', 3)              # exploration cycles
        self.declare_parameter('num_random_goals', 5)        # random waypoints per cycle

        # ROS interface
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # Read all
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.min_frontier_dist = float(self.get_parameter('min_frontier_dist').value)
        self.cost_weight = float(self.get_parameter('cost_weight').value)
        self.gain_weight = float(self.get_parameter('gain_weight').value)
        self.heading_weight = float(self.get_parameter('heading_weight').value)
        self.u_turn_penalty = float(self.get_parameter('u_turn_penalty').value)
        self.u_turn_angle = float(self.get_parameter('u_turn_angle').value)
        self.preferred_heading_limit = float(self.get_parameter('preferred_heading_limit').value)
        self.hard_heading_limit = float(self.get_parameter('hard_heading_limit').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius').value)
        self.blacklist_clear_interval = float(self.get_parameter('blacklist_clear_interval').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.replan_interval = float(self.get_parameter('replan_interval').value)
        self.post_goal_pause = float(self.get_parameter('post_goal_pause').value)
        self.enable_rescue_goals = bool(self.get_parameter('enable_rescue_goals').value)
        self.rescue_goal_radius = float(self.get_parameter('rescue_goal_radius').value)
        self.max_rescue_goals = int(self.get_parameter('max_rescue_goals').value)
        self.no_frontier_threshold = int(self.get_parameter('no_frontier_threshold').value)
        self.min_explored_cells = int(self.get_parameter('min_explored_cells').value)
        self.enable_return_home = bool(self.get_parameter('enable_return_home').value)
        self.map_save_path = str(self.get_parameter('map_save_path').value)
        self.map_cache_save_interval = float(self.get_parameter('map_cache_save_interval').value)
        self.allow_initial_spin = bool(self.get_parameter('allow_initial_spin').value)
        self.initial_spin_duration = float(self.get_parameter('initial_spin_duration').value)
        self.spin_angular_vel = float(self.get_parameter('spin_angular_vel').value)
        self.allow_initial_forward = bool(self.get_parameter('allow_initial_forward').value)
        self.initial_forward_duration = float(self.get_parameter('initial_forward_duration').value)
        self.initial_forward_vel = float(self.get_parameter('initial_forward_vel').value)
        self.num_cycles = int(self.get_parameter('num_cycles').value)
        self.num_random_goals = int(self.get_parameter('num_random_goals').value)
        nav_action = str(self.get_parameter('nav_action_name').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        # ======================== State ========================
        self.state = State.WAITING
        self.current_map: OccupancyGrid | None = None
        self._cached_map_msg: OccupancyGrid | None = None
        self._last_cache_save_time = 0.0
        self._last_cache_map_stamp_ns = -1
        self.start_pos: tuple | None = None
        self.current_goal_pos: tuple | None = None
        self.current_goal_score: float = 0.0  # score when goal was selected
        self.goal_handle = None
        self.blacklist: list = []                # [(x, y, t), ...]
        self.spin_start_time = 0.0
        self.goal_send_time = 0.0
        self.last_frontier_eval = 0.0
        self.last_bl_clear = time.time()
        self.no_frontier_count = 0
        self.pause_until = 0.0
        self.exploration_start = 0.0
        self.goals_sent = 0
        self.goals_ok = 0
        self.goals_fail = 0
        self._goal_pending = False  # True between send_goal_async and _on_resp
        self._goal_gen = 0          # Goal generation counter (detects stale callbacks)
        self._from_idle = False     # True when navigating from external goal in IDLE
        self.cycle = 1              # Current exploration cycle (1-based)
        self.random_goals = []      # Random waypoints for post-exploration navigation
        self.random_goal_idx = 0    # Current random goal index
        self.rescue_goals = deque() # Temporary detour goals after failures
        self._active_rescue = False

        # ======================== TF ========================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ======================== Comms ========================
        action_grp = ReentrantCallbackGroup()
        timer_grp = MutuallyExclusiveCallbackGroup()

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/exploration/goal', self._on_ext_goal, 10)

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, '/exploration/frontiers', 10)
        self.status_pub = self.create_publisher(
            String, '/exploration/status', 10)
        self.reset_pub = self.create_publisher(Empty, '/sim/reset', 10)

        self.nav_client = ActionClient(
            self, NavigateToPose, nav_action, callback_group=action_grp)

        self.timer = self.create_timer(
            0.5, self._tick, callback_group=timer_grp)

        self.get_logger().info(
            f'FrontierExplorer ready | cycles={self.num_cycles} '
            f'random_goals={self.num_random_goals} '
            f'min_size={self.min_frontier_size} '
            f'radius={self.robot_radius} timeout={self.goal_timeout}s '
            f'cv2={_HAS_CV2}')

    # ───────────────────── helpers ─────────────────────
    def _pose(self):
        """(x, y, yaw) in map frame, or None."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                             1 - 2*(q.y*q.y + q.z*q.z))
            return (t.x, t.y, yaw)
        except TransformException:
            return None

    def _on_map(self, msg):
        self.current_map = msg
        self._cached_map_msg = msg
        self._maybe_persist_map_cache(msg)

    def _map_stamp_ns(self, msg: OccupancyGrid) -> int:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _maybe_persist_map_cache(self, msg: OccupancyGrid) -> None:
        now = time.time()
        stamp_ns = self._map_stamp_ns(msg)
        if stamp_ns == self._last_cache_map_stamp_ns:
            return
        if now - self._last_cache_save_time < self.map_cache_save_interval:
            return
        try:
            self._write_map_snapshot(os.path.expanduser(self.map_save_path), msg)
            self._last_cache_save_time = now
            self._last_cache_map_stamp_ns = stamp_ns
        except Exception as e:
            self.get_logger().warn(f'Map cache save skipped: {e}')

    def _write_atomic_text(self, path: str, content: str) -> None:
        directory = os.path.dirname(path) or '.'
        with NamedTemporaryFile('w', dir=directory, delete=False, encoding='utf-8') as tmp:
            tmp.write(content)
            tmp.flush()
            os.fsync(tmp.fileno())
            temp_path = tmp.name
        os.replace(temp_path, path)

    def _write_atomic_bytes(self, path: str, content: bytes) -> None:
        directory = os.path.dirname(path) or '.'
        with NamedTemporaryFile('wb', dir=directory, delete=False) as tmp:
            tmp.write(content)
            tmp.flush()
            os.fsync(tmp.fileno())
            temp_path = tmp.name
        os.replace(temp_path, path)

    def _write_map_snapshot(self, path: str, map_msg: OccupancyGrid) -> None:
        data = np.asarray(map_msg.data, dtype=np.int16)
        width = int(map_msg.info.width)
        height = int(map_msg.info.height)
        if width <= 0 or height <= 0:
            raise ValueError('map size is empty')
        if data.size != width * height:
            raise ValueError(
                f'map data size mismatch: expected {width * height}, got {data.size}')

        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)

        grid = data.reshape((height, width))
        image = np.full((height, width), 205, dtype=np.uint8)
        image[grid == 0] = 254
        image[grid >= 65] = 0
        image = np.flipud(image)

        pgm_path = f'{path}.pgm'
        yaml_path = f'{path}.yaml'
        pgm_header = f'P5\n# CREATOR: frontier_explorer cache\n{width} {height}\n255\n'.encode('ascii')
        self._write_atomic_bytes(pgm_path, pgm_header + image.tobytes())

        origin = map_msg.info.origin
        yaw = 2.0 * math.atan2(origin.orientation.z, origin.orientation.w)
        yaml_content = (
            f'image: {os.path.basename(pgm_path)}\n'
            f'mode: trinary\n'
            f'resolution: {map_msg.info.resolution:.12g}\n'
            f'origin: [{origin.position.x:.12g}, {origin.position.y:.12g}, {yaw:.12g}]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n'
        )
        self._write_atomic_text(yaml_path, yaml_content)

    def _on_ext_goal(self, msg):
        if self.state != State.IDLE:
            self.get_logger().warn('Ignoring goal: exploration active')
            return
        x, y = msg.pose.position.x, msg.pose.position.y
        self.get_logger().info(f'External goal ({x:.2f}, {y:.2f})')
        self._from_idle = True
        self._send_goal(x, y)
        self.state = State.NAVIGATING

    # ───────────────────── main loop ─────────────────────
    def _tick(self):
        st = String()
        st.data = (f'{State(self.state).name} cycle={self.cycle}/{self.num_cycles} '
                   f'sent={self.goals_sent} ok={self.goals_ok} fail={self.goals_fail}')
        self.status_pub.publish(st)

        if time.time() < self.pause_until:
            return

        handler = {
            State.WAITING:      self._s_waiting,
            State.INITIAL_SPIN: self._s_spin,
            State.EXPLORING:    self._s_exploring,
            State.NAVIGATING:   self._s_navigating,
            State.RETURNING:    self._s_returning,
            State.SAVING_MAP:   self._s_saving,
            State.RANDOM_NAV:   self._s_random_nav,
            State.RESETTING:    self._s_resetting,
            State.IDLE:         lambda: None,
        }.get(self.state)
        if handler:
            handler()

    # ─── WAITING ───
    def _s_waiting(self):
        if not self.current_map:
            self.get_logger().info('Waiting /map...', throttle_duration_sec=5.0)
            return
        p = self._pose()
        if not p:
            self.get_logger().info('Waiting TF...', throttle_duration_sec=5.0)
            return
        if not self.nav_client.wait_for_server(timeout_sec=0.5):
            self.get_logger().info('Waiting Nav2...', throttle_duration_sec=5.0)
            return
        self.start_pos = (p[0], p[1])
        self.exploration_start = time.time()
        next_state = State.INITIAL_SPIN if (self.allow_initial_spin or self.allow_initial_forward) else State.EXPLORING
        self.get_logger().info(
            f'[Cycle {self.cycle}/{self.num_cycles}] '
            f'Start ({p[0]:.2f}, {p[1]:.2f}) -> {State(next_state).name}')
        self.state = next_state
        if self.allow_initial_spin or self.allow_initial_forward:
            self.spin_start_time = time.time()

    # ─── INITIAL SPIN ───
    def _s_spin(self):
        if not self.allow_initial_spin and not self.allow_initial_forward:
            self.state = State.EXPLORING
            return
        if self.allow_initial_spin and time.time() - self.spin_start_time < self.initial_spin_duration:
            cmd = Twist()
            cmd.angular.z = self.spin_angular_vel
            self.cmd_pub.publish(cmd)
            return
        if self.allow_initial_forward and time.time() - self.spin_start_time < self.initial_forward_duration:
            cmd = Twist()
            cmd.linear.x = self.initial_forward_vel
            self.cmd_pub.publish(cmd)
            return

        self.cmd_pub.publish(Twist())
        if self.allow_initial_spin:
            self.get_logger().info('Scan done -> EXPLORING')
        elif self.allow_initial_forward:
            self.get_logger().info('Initial forward done -> EXPLORING')
        else:
            self.get_logger().info('Initial seed motion done -> EXPLORING')
        self.state = State.EXPLORING

    # ─── EXPLORING ───
    def _s_exploring(self):
        if not self.current_map:
            return
        p = self._pose()
        if not p:
            return
        rx, ry = p[0], p[1]

        # auto-clear old blacklist
        now = time.time()
        if now - self.last_bl_clear > self.blacklist_clear_interval and self.blacklist:
            n = len(self.blacklist)
            self.blacklist.clear()
            self.last_bl_clear = now
            self.get_logger().info(f'Cleared {n} blacklisted goals')

        frontiers = self._detect_frontiers()
        self._viz(frontiers, rx, ry)

        if self.rescue_goals:
            gx, gy = self.rescue_goals.popleft()
            self.get_logger().info(
                f'Rescue goal -> ({gx:.2f},{gy:.2f}) '
                f'remaining={len(self.rescue_goals)}')
            self._active_rescue = True
            self.current_goal_score = -1e9
            self._send_goal(gx, gy)
            self.state = State.NAVIGATING
            return

        if not frontiers:
            self.no_frontier_count += 1
            self.get_logger().info(
                f'No frontiers ({self.no_frontier_count}/{self.no_frontier_threshold})')
            if self.no_frontier_count >= self.no_frontier_threshold:
                grid = np.array(self.current_map.data, dtype=np.int8)
                free = int(np.sum((grid >= 0) & (grid <= 50)))
                if free < self.min_explored_cells:
                    if self.allow_initial_spin:
                        self.get_logger().warn(f'Map small ({free} cells), re-spin')
                        self.state = State.INITIAL_SPIN
                        self.spin_start_time = time.time()
                    else:
                        self.get_logger().warn(
                            f'Map small ({free} cells), keep waiting for more observations')
                        self.pause_until = time.time() + 1.0
                        self.state = State.EXPLORING
                    self.no_frontier_count = 0
                    return
                dt = time.time() - self.exploration_start
                self.get_logger().info(
                    f'*** EXPLORATION COMPLETE (Cycle {self.cycle}/{self.num_cycles}) *** '
                    f'{dt:.0f}s free={free} '
                    f'sent={self.goals_sent} ok={self.goals_ok} fail={self.goals_fail}')
                if self.enable_return_home and self.start_pos:
                    self.get_logger().info(f'Returning home')
                    self._send_goal(*self.start_pos)
                    self.state = State.RETURNING
                else:
                    self.state = State.SAVING_MAP
            return

        best = self._pick(frontiers, rx, ry, p[2])
        if not best:
            self.no_frontier_count += 1
            self.get_logger().warn(
                f'All blacklisted ({self.no_frontier_count}/{self.no_frontier_threshold})')
            if self.no_frontier_count >= self.no_frontier_threshold:
                self.get_logger().info('All frontiers unreachable -> clear and retry once')
                self.blacklist.clear()
                self.no_frontier_count = 0
            return
        self.no_frontier_count = 0
        self._active_rescue = False
        self.get_logger().info(
            f'-> ({best["x"]:.2f},{best["y"]:.2f}) sz={best["size"]} '
            f'd={best["dist"]:.2f} sc={best["score"]:.2f}')
        self.current_goal_score = best['score']
        self._send_goal(best['x'], best['y'])
        self.state = State.NAVIGATING

    # ─── NAVIGATING ───
    def _s_navigating(self):
        # Still waiting for goal acceptance/rejection — do nothing
        if self._goal_pending:
            return
        # result callback already fired
        if self.goal_handle is None and self.current_goal_pos is None:
            if self._from_idle:
                self._from_idle = False
                self.state = State.IDLE
            else:
                self.state = State.EXPLORING
            return
        # goal rejected
        if self.goal_handle is None and self.current_goal_pos is not None:
            self._bl_current()
            if self._from_idle:
                self._from_idle = False
                self.state = State.IDLE
            else:
                self.state = State.EXPLORING
            return
        # timeout
        if time.time() - self.goal_send_time > self.goal_timeout:
            self.get_logger().warn('Goal timeout')
            self._cancel()
            self._bl_current()
            if self._from_idle:
                self._from_idle = False
                self.state = State.IDLE
            else:
                self.state = State.EXPLORING
            return
        # Skip re-evaluation for external (IDLE) goals
        if self._from_idle:
            return
        # Fast check every tick: is the goal area still a frontier?
        # If the surrounding area is fully known, cancel immediately.
        if self.current_goal_pos and not self._goal_still_frontier():
            self.get_logger().info('Goal area fully explored, replanning')
            self._cancel()
            self.current_goal_pos = None
            self.goal_handle = None
            self.state = State.EXPLORING
            return
        # Periodic re-evaluation: switch to a better frontier as soon as one appears
        now = time.time()
        if now - self.last_frontier_eval > self.replan_interval:
            self.last_frontier_eval = now
            p = self._pose()
            if p and self.current_goal_pos:
                fs = self._detect_frontiers()
                if fs:
                    b = self._pick(fs, p[0], p[1], p[2])
                    # Switch if a different frontier now scores higher
                    # (small margin of 0.3 prevents oscillation between equal ones)
                    if b and b['score'] > self.current_goal_score + 0.3:
                        bd = math.hypot(b['x'] - self.current_goal_pos[0],
                                        b['y'] - self.current_goal_pos[1])
                        # Only switch if it's actually a different point
                        if bd > self.blacklist_radius:
                            self.get_logger().info(
                                f'Better frontier found -> switch '
                                f'({b["x"]:.2f},{b["y"]:.2f}) '
                                f'score={b["score"]:.1f} > cur={self.current_goal_score:.1f}')
                            self._cancel()
                            self.current_goal_pos = None
                            self.goal_handle = None
                            self.state = State.EXPLORING

    # ─── RETURNING ───
    def _s_returning(self):
        if self.goal_handle is None:
            self.state = State.SAVING_MAP
            return
        if time.time() - self.goal_send_time > 90.0:
            self.get_logger().warn('Return timeout')
            self._cancel()
            self.goal_handle = None
            self.state = State.SAVING_MAP

    # ─── SAVING MAP ───
    def _s_saving(self):
        self._save_map()
        # Generate random waypoints for post-exploration navigation
        self.random_goals = self._generate_random_goals(self.num_random_goals)
        self.random_goal_idx = 0
        if self.random_goals:
            self.get_logger().info(
                f'Map saved. Starting random navigation with '
                f'{len(self.random_goals)} waypoints.')
            self.state = State.RANDOM_NAV
        else:
            self.get_logger().info('Map saved. No valid waypoints generated.')
            if self.cycle < self.num_cycles:
                self.cycle += 1
                self.state = State.RESETTING
            else:
                self.get_logger().info('*** ALL CYCLES COMPLETE ***')
                self.state = State.IDLE

    # ─── RANDOM NAV ───
    def _s_random_nav(self):
        if self._goal_pending:
            return
        # Goal rejected
        if self.goal_handle is None and self.current_goal_pos is not None:
            self.get_logger().warn('Random nav goal rejected, skipping')
            self.current_goal_pos = None
            self.random_goal_idx += 1
            return
        # Goal still running — check timeout
        if self.goal_handle is not None:
            if time.time() - self.goal_send_time > self.goal_timeout:
                self.get_logger().warn('Random nav goal timeout, skipping')
                self._cancel()
                self.goal_handle = None
                self.current_goal_pos = None
                self.random_goal_idx += 1
            return
        # Send next random goal or finish
        if self.random_goal_idx < len(self.random_goals):
            gx, gy = self.random_goals[self.random_goal_idx]
            self.get_logger().info(
                f'[Cycle {self.cycle}] Random goal '
                f'{self.random_goal_idx + 1}/{len(self.random_goals)}: '
                f'({gx:.2f}, {gy:.2f})')
            self._send_goal(gx, gy)
        else:
            self.get_logger().info(
                f'*** Cycle {self.cycle}/{self.num_cycles} complete ***')
            if self.cycle < self.num_cycles:
                self.cycle += 1
                self.state = State.RESETTING
            else:
                self.get_logger().info('*** ALL CYCLES COMPLETE ***')
                self.state = State.IDLE

    # ─── RESETTING ───
    def _s_resetting(self):
        self.get_logger().info(
            f'Resetting for cycle {self.cycle}/{self.num_cycles}...')
        self._cancel()
        self.reset_pub.publish(Empty())
        # Clear exploration state
        self.current_map = None
        self.blacklist.clear()
        self.no_frontier_count = 0
        self.rescue_goals.clear()
        self._active_rescue = False
        self.current_goal_pos = None
        self.goal_handle = None
        self._goal_pending = False
        self.exploration_start = 0.0
        self.goals_sent = 0
        self.goals_ok = 0
        self.goals_fail = 0
        self.random_goals = []
        self.random_goal_idx = 0
        # Wait for sim to regenerate map
        self.pause_until = time.time() + 2.0
        self.state = State.WAITING

    # ═══════════════════ Frontier Detection ═══════════════════
    def _detect_frontiers(self):
        info = self.current_map.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        grid = np.array(self.current_map.data, dtype=np.int8).reshape(h, w)

        free = (grid >= 0) & (grid <= 50)
        unk = (grid == -1)
        occ = grid > 65

        # free cell adjacent to unknown = frontier
        adj = np.zeros((h, w), dtype=bool)
        for dy, dx in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            # use slicing (not roll) to avoid wrap-around artifacts
            sr = (max(0, dy), h + min(0, dy))
            sc = (max(0, dx), w + min(0, dx))
            tr = (max(0, -dy), h + min(0, -dy))
            tc = (max(0, -dx), w + min(0, -dx))
            adj[tr[0]:tr[1], tc[0]:tc[1]] |= unk[sr[0]:sr[1], sc[0]:sc[1]]

        frontier = free & adj

        # inflate obstacles for safety
        safe_c = max(1, int(math.ceil(self.robot_radius / res)))
        occ_buf = np.zeros((h, w), dtype=bool)
        for dy in range(-safe_c, safe_c + 1):
            for dx in range(-safe_c, safe_c + 1):
                if dy*dy + dx*dx > safe_c*safe_c:
                    continue
                sr = (max(0, dy), h + min(0, dy))
                sc = (max(0, dx), w + min(0, dx))
                tr = (max(0, -dy), h + min(0, -dy))
                tc = (max(0, -dx), w + min(0, -dx))
                occ_buf[tr[0]:tr[1], tc[0]:tc[1]] |= occ[sr[0]:sr[1], sc[0]:sc[1]]
        frontier &= ~occ_buf

        if not np.any(frontier):
            return []

        # cluster
        if _HAS_CV2:
            nl, lb = cv2.connectedComponents(frontier.astype(np.uint8), connectivity=8)
            out = []
            for i in range(1, nl):
                c = np.argwhere(lb == i)
                if len(c) < self.min_frontier_size:
                    continue
                cy, cx = c.mean(axis=0)
                out.append({'x': ox+cx*res, 'y': oy+cy*res, 'size': len(c)})
        else:
            out = self._cluster_bfs(frontier, h, w, res, ox, oy)
        return out

    def _cluster_bfs(self, mask, h, w, res, ox, oy):
        vis = np.zeros((h, w), dtype=bool)
        result = []
        for py, px in np.argwhere(mask):
            if vis[py, px]:
                continue
            q = deque([(py, px)])
            vis[py, px] = True
            cells = []
            while q:
                cy, cx = q.popleft()
                cells.append((cy, cx))
                for dy, dx in ((-1,0),(1,0),(0,-1),(0,1)):
                    ny, nx = cy+dy, cx+dx
                    if 0<=ny<h and 0<=nx<w and not vis[ny,nx] and mask[ny,nx]:
                        vis[ny, nx] = True
                        q.append((ny, nx))
            if len(cells) < self.min_frontier_size:
                continue
            a = np.array(cells, dtype=float)
            cy, cx = a.mean(axis=0)
            result.append({'x': ox+cx*res, 'y': oy+cy*res, 'size': len(cells)})
        return result

    # ═══════════════════ Frontier Selection ═══════════════════
    def _pick(self, frontiers, rx, ry, ryaw):
        best_pref, bs_pref = None, -1e9
        best_fallback, bs_fallback = None, -1e9
        for f in frontiers:
            if self._in_bl(f['x'], f['y']):
                continue
            d = math.hypot(f['x']-rx, f['y']-ry)
            if d < self.min_frontier_dist:
                continue
            heading = math.atan2(f['y'] - ry, f['x'] - rx)
            heading_err = abs(self._angle_diff(ryaw, heading))
            heading_pen = self.heading_weight * heading_err
            if heading_err > self.u_turn_angle:
                heading_pen += self.u_turn_penalty
            f['dist'] = d
            f['heading_err'] = heading_err
            s = (self.gain_weight * math.log1p(f['size'])
                 - self.cost_weight * d
                 - heading_pen)
            f['score'] = s
            if heading_err <= self.preferred_heading_limit:
                if s > bs_pref:
                    bs_pref = s
                    best_pref = f
                continue
            if heading_err <= self.hard_heading_limit and s > bs_fallback:
                bs_fallback = s
                best_fallback = f
        return best_pref if best_pref is not None else best_fallback

    @staticmethod
    def _angle_diff(a, b):
        d = b - a
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return d

    def _in_bl(self, x, y):
        return any(math.hypot(x-bx, y-by) < self.blacklist_radius
                   for bx, by, _ in self.blacklist)

    def _bl_current(self):
        if self.current_goal_pos:
            if not self._active_rescue:
                self.blacklist.append((*self.current_goal_pos, time.time()))
                self.get_logger().info(
                    f'Blacklisted ({self.current_goal_pos[0]:.2f},'
                    f'{self.current_goal_pos[1]:.2f})')
            else:
                self.get_logger().info(
                    f'Rescue goal failed ({self.current_goal_pos[0]:.2f},'
                    f'{self.current_goal_pos[1]:.2f})')
            self.goals_fail += 1
        self.current_goal_pos = None
        self.goal_handle = None
        self._active_rescue = False

    def _goal_still_frontier(self):
        """Quick check: are there still unknown cells near the current goal?

        Runs every tick (0.5s) during navigation. If the area around the goal
        is fully explored, the goal is stale and we should replan immediately.
        This is O(patch_size) on the raw map data — very cheap, no numpy needed.
        """
        if not self.current_goal_pos or not self.current_map:
            return True  # can't verify, assume still valid
        info = self.current_map.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx = int((self.current_goal_pos[0] - ox) / res)
        gy = int((self.current_goal_pos[1] - oy) / res)
        data = self.current_map.data
        # Check ~0.5 m radius around the goal for any unknown cell
        check_r = max(3, int(0.5 / res))
        for dy in range(-check_r, check_r + 1):
            for dx in range(-check_r, check_r + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if data[ny * w + nx] == -1:
                        return True
        return False

    # ═══════════════════ Random Goal Generation ═══════════════════
    def _generate_random_goals(self, n):
        """Generate N random navigable positions from the current explored map."""
        if not self.current_map:
            return []
        info = self.current_map.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        grid = np.array(self.current_map.data, dtype=np.int8).reshape(h, w)

        free = (grid >= 0) & (grid <= 50)
        occ = grid > 65

        # Inflate obstacles for safety
        safe_c = max(1, int(math.ceil(self.robot_radius * 2 / res)))
        occ_buf = np.zeros((h, w), dtype=bool)
        for dy in range(-safe_c, safe_c + 1):
            for dx in range(-safe_c, safe_c + 1):
                if dy * dy + dx * dx > safe_c * safe_c:
                    continue
                sr = (max(0, dy), h + min(0, dy))
                sc = (max(0, dx), w + min(0, dx))
                tr = (max(0, -dy), h + min(0, -dy))
                tc = (max(0, -dx), w + min(0, -dx))
                occ_buf[tr[0]:tr[1], tc[0]:tc[1]] |= occ[sr[0]:sr[1], sc[0]:sc[1]]

        safe_free = free & ~occ_buf
        candidates = np.argwhere(safe_free)

        if len(candidates) == 0:
            return []

        rng = np.random.default_rng()
        indices = rng.choice(
            len(candidates), size=min(n * 5, len(candidates)), replace=False)

        goals = []
        for idx in indices:
            if len(goals) >= n:
                break
            cy, cx = candidates[idx]
            wx = ox + cx * res
            wy = oy + cy * res
            # Ensure minimum distance from other goals and start
            too_close = False
            for gx, gy in goals:
                if math.hypot(wx - gx, wy - gy) < 1.0:
                    too_close = True
                    break
            if self.start_pos and math.hypot(
                    wx - self.start_pos[0], wy - self.start_pos[1]) < 0.5:
                too_close = True
            if not too_close:
                goals.append((wx, wy))

        return goals

    def _queue_rescue_goals(self):
        if not self.enable_rescue_goals or not self.current_map:
            return
        pose = self._pose()
        if not pose:
            return
        rx, ry, ryaw = pose
        candidates = []
        for dist in (self.rescue_goal_radius, self.rescue_goal_radius * 0.7):
            for ang in (0.0, 0.55, -0.55, 1.1, -1.1):
                heading = ryaw + ang
                gx = rx + dist * math.cos(heading)
                gy = ry + dist * math.sin(heading)
                if self._map_cell_is_free(gx, gy):
                    candidates.append((gx, gy))

        queued = 0
        seen = set()
        for gx, gy in candidates:
            key = (round(gx, 2), round(gy, 2))
            if key in seen:
                continue
            seen.add(key)
            self.rescue_goals.append((gx, gy))
            queued += 1
            if queued >= self.max_rescue_goals:
                break
        if queued:
            self.get_logger().info(f'Queued {queued} rescue goals after failure')

    def _map_cell_is_free(self, wx, wy):
        if not self.current_map:
            return False
        info = self.current_map.info
        w, h, res = info.width, info.height, info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        safe_c = max(1, int(math.ceil(self.robot_radius / res)))
        if not (0 <= gx < w and 0 <= gy < h):
            return False
        data = self.current_map.data
        for dy in range(-safe_c, safe_c + 1):
            for dx in range(-safe_c, safe_c + 1):
                if dx * dx + dy * dy > safe_c * safe_c:
                    continue
                nx, ny = gx + dx, gy + dy
                if not (0 <= nx < w and 0 <= ny < h):
                    return False
                v = data[ny * w + nx]
                if v < 0 or v > 50:
                    return False
        return True

    # ═══════════════════ Nav2 Action ═══════════════════
    def _send_goal(self, x, y):
        self._goal_gen += 1
        gen = self._goal_gen
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.w = 1.0
        self.current_goal_pos = (x, y)
        self.goal_send_time = time.time()
        self.goals_sent += 1
        self._goal_pending = True
        fut = self.nav_client.send_goal_async(g, feedback_callback=self._fb)
        fut.add_done_callback(lambda f: self._on_resp(f, gen))

    def _on_resp(self, fut, gen):
        self._goal_pending = False
        if gen != self._goal_gen:
            return  # stale response from a canceled goal
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_handle = None
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._on_result(f, gen))

    def _on_result(self, fut, gen):
        if gen != self._goal_gen:
            return  # stale result from a canceled goal — ignore
        st = fut.result().status
        if st == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal reached')
            self.goals_ok += 1
            self.pause_until = time.time() + self.post_goal_pause
            self._active_rescue = False
        else:
            self.get_logger().warn(f'Goal failed (st={st})')
            if self.state == State.NAVIGATING and not self._active_rescue:
                self._queue_rescue_goals()
            # Don't blacklist random nav goals, rescue goals, or canceled goals
            if self.state != State.RANDOM_NAV and self.current_goal_pos and not self._active_rescue:
                self.blacklist.append((*self.current_goal_pos, time.time()))
            self.goals_fail += 1
            self._active_rescue = False
        self.goal_handle = None
        self.current_goal_pos = None
        if self.state == State.NAVIGATING:
            if self._from_idle:
                self._from_idle = False
                self.state = State.IDLE
            else:
                self.state = State.EXPLORING
        elif self.state == State.RETURNING:
            self.state = State.SAVING_MAP
        elif self.state == State.RANDOM_NAV:
            self.random_goal_idx += 1

    def _fb(self, _):
        pass

    def _cancel(self):
        if self.goal_handle:
            self.get_logger().info('Cancel goal')
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    # ═══════════════════ Map Saving ═══════════════════
    def _save_map(self, prefer_cache: bool = False):
        path = os.path.expanduser(self.map_save_path)
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        cached_map = self.current_map or self._cached_map_msg

        if prefer_cache and cached_map is not None:
            try:
                self.get_logger().info(f'Saving cached map -> {path}')
                self._write_map_snapshot(path, cached_map)
                self.get_logger().info('Cached map saved OK')
                return
            except Exception as e:
                self.get_logger().warn(f'Cached map save failed, fallback to map_saver_cli: {e}')

        try:
            self.get_logger().info(f'Saving map -> {path}')
            subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', path, '--ros-args', '-p', 'save_map_timeout:=10.0'],
                timeout=20.0, check=True,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info('Map saved OK')
            if cached_map is not None:
                self._last_cache_save_time = time.time()
                self._last_cache_map_stamp_ns = self._map_stamp_ns(cached_map)
        except Exception as e:
            if cached_map is None:
                self.get_logger().error(f'Map save error: {e}')
                return
            try:
                self.get_logger().warn(f'map_saver_cli failed, using cached map: {e}')
                self._write_map_snapshot(path, cached_map)
                self.get_logger().info('Cached map saved OK')
            except Exception as cache_error:
                self.get_logger().error(f'Map save error: {cache_error}')

    # ═══════════════════ Visualization ═══════════════════
    def _viz(self, frontiers, rx, ry):
        ma = MarkerArray()
        s = self.get_clock().now().to_msg()
        # delete all
        d = Marker(); d.action = Marker.DELETEALL
        ma.markers.append(d)
        # start
        if self.start_pos:
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = s; m.id = 1
            m.type = Marker.CYLINDER; m.action = Marker.ADD
            m.pose.position.x = self.start_pos[0]
            m.pose.position.y = self.start_pos[1]
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3; m.scale.z = 0.02
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            ma.markers.append(m)
        # frontiers
        for i, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = s; m.id = 100+i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = f['x']; m.pose.position.y = f['y']
            m.pose.position.z = 0.1; m.pose.orientation.w = 1.0
            sc = min(0.3, max(0.06, f['size']/80.0))
            m.scale.x = m.scale.y = m.scale.z = sc
            m.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8)
            ma.markers.append(m)
        # current goal
        if self.current_goal_pos:
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = s; m.id = 9999
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = self.current_goal_pos[0]
            m.pose.position.y = self.current_goal_pos[1]
            m.pose.position.z = 0.25; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
            ma.markers.append(m)
        # random nav goals (show upcoming waypoints)
        if self.state == State.RANDOM_NAV:
            for i, (gx, gy) in enumerate(self.random_goals):
                m = Marker()
                m.header.frame_id = 'map'; m.header.stamp = s; m.id = 7000 + i
                m.type = Marker.CYLINDER; m.action = Marker.ADD
                m.pose.position.x = gx; m.pose.position.y = gy
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = 0.2; m.scale.z = 0.02
                if i < self.random_goal_idx:
                    # already visited — green
                    m.color = ColorRGBA(r=0.0, g=0.8, b=0.2, a=0.5)
                elif i == self.random_goal_idx:
                    # current target — yellow
                    m.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=1.0)
                else:
                    # upcoming — magenta
                    m.color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.6)
                ma.markers.append(m)
        # blacklist
        for i, (bx, by, _) in enumerate(self.blacklist):
            m = Marker()
            m.header.frame_id = 'map'; m.header.stamp = s; m.id = 5000+i
            m.type = Marker.CUBE; m.action = Marker.ADD
            m.pose.position.x = bx; m.pose.position.y = by
            m.pose.position.z = 0.1; m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            ma.markers.append(m)
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._cancel()
        node._save_map(prefer_cache=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
