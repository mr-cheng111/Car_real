"""Controllers for switching between mapping and navigation launch files.

Usage example:
    from ros2_launch_controllers import ExplorationFlowController

    flow = ExplorationFlowController()
    flow.start_mapping()
    # ...
    flow.start_navigation()
    flow.send_navigation_goal(x=1.2, y=-0.3, yaw=1.57)
    flow.wait_navigation()
    # ...
    flow.stop_all()
"""

from __future__ import annotations

import math
import os
import re
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


@dataclass
class ProcessStatus:
    name: str
    running: bool
    pid: Optional[int]
    started_at: Optional[float]
    mode: str
    last_error: Optional[str]


class LaunchProcessController:
    """Base process controller for a single ros2 launch command."""

    def __init__(
        self,
        name: str,
        package: str,
        launch_file: str,
        default_args: Optional[Dict[str, str]] = None,
        log_dir: str = "./logs",
    ) -> None:
        self.name = name
        self.package = package
        self.launch_file = launch_file
        self.default_args = default_args or {}
        self.log_dir = os.path.abspath(log_dir)

        self._process: Optional[subprocess.Popen] = None
        self._log_fp = None
        self._started_at: Optional[float] = None
        self._last_error: Optional[str] = None
        self._lock = threading.Lock()

    def _pid_file_path(self) -> str:
        return os.path.join(self.log_dir, f"{self.name}.pid")

    def _write_pid_file(self, pid: int) -> None:
        with open(self._pid_file_path(), "w", encoding="utf-8") as fp:
            fp.write(str(pid))

    def _read_pid_file(self) -> Optional[int]:
        try:
            with open(self._pid_file_path(), "r", encoding="utf-8") as fp:
                value = fp.read().strip()
            return int(value) if value else None
        except Exception:
            return None

    def _clear_pid_file(self) -> None:
        try:
            os.remove(self._pid_file_path())
        except FileNotFoundError:
            pass
        except Exception:
            pass

    @staticmethod
    def _is_pid_running(pid: int) -> bool:
        if pid <= 0:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def _stop_by_pid(self, pid: int, timeout_sec: float) -> bool:
        try:
            if os.name == "nt":
                os.kill(pid, signal.CTRL_BREAK_EVENT)
            else:
                os.killpg(pid, signal.SIGINT)

            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                if not self._is_pid_running(pid):
                    self._clear_pid_file()
                    return True
                time.sleep(0.2)

            if os.name == "nt":
                os.kill(pid, signal.SIGTERM)
            else:
                os.killpg(pid, signal.SIGTERM)

            time.sleep(0.5)
            if not self._is_pid_running(pid):
                self._clear_pid_file()
                return True

            if os.name == "nt":
                os.kill(pid, signal.SIGKILL)
            else:
                os.killpg(pid, signal.SIGKILL)

            self._clear_pid_file()
            return True
        except Exception as exc:
            self._last_error = f"stop by pid failed: {exc}"
            return False

    def _build_command(self, launch_args: Optional[Dict[str, str]] = None) -> List[str]:
        args = dict(self.default_args)
        if launch_args:
            args.update(launch_args)

        cmd = ["ros2", "launch", self.package, self.launch_file]
        for key, value in args.items():
            cmd.append(f"{key}:={value}")
        return cmd

    def is_running(self) -> bool:
        if self._process is not None and self._process.poll() is None:
            return True

        pid = self._read_pid_file()
        return self._is_pid_running(pid) if pid is not None else False

    def start(self, launch_args: Optional[Dict[str, str]] = None) -> bool:
        with self._lock:
            if self.is_running():
                return True

            os.makedirs(self.log_dir, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(self.log_dir, f"{self.name}_{timestamp}.log")

            cmd = self._build_command(launch_args)
            self._last_error = None

            try:
                self._log_fp = open(log_path, "a", encoding="utf-8")

                creationflags = 0
                if os.name == "nt":
                    creationflags = subprocess.CREATE_NEW_PROCESS_GROUP

                self._process = subprocess.Popen(
                    cmd,
                    stdout=self._log_fp,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=(os.name != "nt"),
                    creationflags=creationflags,
                )
                self._started_at = time.time()
                self._write_pid_file(self._process.pid)
                return True
            except Exception as exc:
                self._last_error = f"start failed: {exc}"
                self._started_at = None
                self._process = None
                if self._log_fp:
                    self._log_fp.close()
                    self._log_fp = None
                return False

    def stop(self, timeout_sec: float = 8.0) -> bool:
        with self._lock:
            if self._process is None:
                pid = self._read_pid_file()
                if pid is None:
                    return True
                if not self._is_pid_running(pid):
                    # PID file can be stale after abnormal exits or restarts.
                    self._clear_pid_file()
                    return True
                return self._stop_by_pid(pid, timeout_sec)

            if self._process.poll() is not None:
                self._cleanup_after_exit()
                return True

            try:
                if os.name == "nt":
                    self._process.send_signal(signal.CTRL_BREAK_EVENT)
                else:
                    os.killpg(self._process.pid, signal.SIGINT)

                self._process.wait(timeout=timeout_sec)
            except Exception:
                try:
                    self._process.terminate()
                    self._process.wait(timeout=3.0)
                except Exception:
                    try:
                        self._process.kill()
                    except Exception as exc:
                        self._last_error = f"stop failed: {exc}"
                        return False

            self._cleanup_after_exit()
            return True

    def _cleanup_after_exit(self) -> None:
        self._process = None
        self._started_at = None
        self._clear_pid_file()
        if self._log_fp:
            self._log_fp.close()
            self._log_fp = None

    def status(self) -> ProcessStatus:
        pid = self._process.pid if self._process is not None else self._read_pid_file()
        running = self.is_running()
        mode = "running" if running else "stopped"
        started_at = self._started_at

        if started_at is None and running and pid is not None:
            try:
                started_at = os.path.getmtime(self._pid_file_path())
            except Exception:
                started_at = None

        return ProcessStatus(
            name=self.name,
            running=running,
            pid=pid,
            started_at=started_at,
            mode=mode,
            last_error=self._last_error,
        )


class MappingController(LaunchProcessController):
    """Controller for mapping launch.

    Command:
        ros2 launch robot_bringup mapping.launch.py enable_teleop:=false

    Notes:
        Mapping shutdown should prefer graceful SIGINT and allow enough time
        for Cartographer/Nav2 map saving to finish cleanly.
    """

    def __init__(self, log_dir: str = "./logs") -> None:
        self.pbstream_path = os.path.abspath(
            os.environ.get(
                "ROBOT_BRINGUP_PBSTREAM_PATH",
                "src/car_nav2/maps/cartographer/latest.pbstream",
            )
        )
        super().__init__(
            name="mapping",
            package=os.environ.get("ROBOT_BRINGUP_MAPPING_PACKAGE", "robot_bringup"),
            launch_file=os.environ.get("ROBOT_BRINGUP_MAPPING_LAUNCH", "mapping.launch.py"),
            default_args={
                "enable_teleop": "false",
            },
            log_dir=log_dir,
        )

    def save_pbstream(self, timeout_sec: float = 25.0) -> bool:
        if not self.is_running():
            return True
        try:
            result = subprocess.run(
                [
                    "ros2", "run", "robot_bringup", "save_cartographer_state.py",
                    "--output", self.pbstream_path,
                    "--timeout", str(max(1.0, timeout_sec)),
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=max(2.0, timeout_sec + 5.0),
                check=False,
            )
            if result.returncode != 0:
                self._last_error = f"pbstream save failed: exit {result.returncode}"
                return False
            return True
        except Exception as exc:
            self._last_error = f"pbstream save failed: {exc}"
            return False

    def stop(self, timeout_sec: float = 30.0) -> bool:
        self.save_pbstream()
        return super().stop(timeout_sec=timeout_sec)


class NavigationController(LaunchProcessController):
    """Controller for navigation launch.

    Command:
        ros2 launch car_nav2 real_car_nav2.launch.py
    """

    def __init__(self, log_dir: str = "./logs") -> None:
        super().__init__(
            name="navigation",
            package=os.environ.get("ROBOT_BRINGUP_NAVIGATION_PACKAGE", "car_nav2"),
            launch_file=os.environ.get("ROBOT_BRINGUP_NAVIGATION_LAUNCH", "real_car_nav2.launch.py"),
            default_args={
                "use_sim_time": "false",
                "start_hardware": "true",
                "use_rviz": "false",
            },
            log_dir=log_dir,
        )


class ExplorationFlowController:
    """High-level switcher to ensure mapping/navigation are mutually exclusive."""

    def __init__(self, log_dir: str = "./logs") -> None:
        self.mapping = MappingController(log_dir=log_dir)
        self.navigation = NavigationController(log_dir=log_dir)
        self._mode = "idle"
        self._lock = threading.Lock()

        self._runtime_lock = threading.Lock()
        self._runtime_stop_event = threading.Event()
        self._runtime_started_event = threading.Event()
        self._runtime_thread: Optional[threading.Thread] = None
        self._runtime_status: Dict[str, Any] = {
            "monitor_running": False,
            "monitor_error": None,
            "last_update_time": None,
            "exploration_status_raw": None,
            "exploration_state": None,
            "exploration_cycle": None,
            "exploration_total_cycles": None,
            "exploration_goals_sent": None,
            "exploration_goals_ok": None,
            "exploration_goals_fail": None,
            "mapping_finished": False,
            "navigation_simple_status_raw": None,
            "navigation_action_status": None,
            "navigation_last_result": None,
            "navigation_has_goal": False,
            "navigation_goal_reached": False,
        }

    @staticmethod
    def _goal_status_name(status_code: int) -> str:
        names = {
            0: "unknown",
            1: "accepted_pending",
            2: "accepted",
            3: "executing",
            4: "canceling",
            5: "succeeded",
            6: "canceled",
            7: "aborted",
        }
        return names.get(status_code, f"status_{status_code}")

    @staticmethod
    def _parse_exploration_status(status_text: str) -> Dict[str, Optional[int | str]]:
        parsed: Dict[str, Optional[int | str]] = {
            "state": None,
            "cycle": None,
            "total_cycles": None,
            "sent": None,
            "ok": None,
            "fail": None,
        }
        if not status_text:
            return parsed

        tokens = status_text.strip().split()
        if not tokens:
            return parsed

        state = tokens[0]
        if re.fullmatch(r"[A-Z_]+", state):
            parsed["state"] = state

        for token in tokens[1:]:
            if token.startswith("cycle="):
                value = token.split("=", 1)[1]
                if "/" in value:
                    cur, total = value.split("/", 1)
                    if cur.isdigit() and total.isdigit():
                        parsed["cycle"] = int(cur)
                        parsed["total_cycles"] = int(total)
            elif token.startswith("sent="):
                value = token.split("=", 1)[1]
                if value.isdigit():
                    parsed["sent"] = int(value)
            elif token.startswith("ok="):
                value = token.split("=", 1)[1]
                if value.isdigit():
                    parsed["ok"] = int(value)
            elif token.startswith("fail="):
                value = token.split("=", 1)[1]
                if value.isdigit():
                    parsed["fail"] = int(value)

        return parsed

    def _update_runtime_status(self, **updates: Any) -> None:
        with self._runtime_lock:
            self._runtime_status.update(updates)
            self._runtime_status["last_update_time"] = time.time()

    def _ensure_runtime_monitor(self) -> None:
        with self._runtime_lock:
            if self._runtime_thread is not None and self._runtime_thread.is_alive():
                return
            self._runtime_stop_event.clear()
            self._runtime_started_event.clear()
            self._runtime_status["monitor_error"] = None
            self._runtime_status["monitor_running"] = True
            self._runtime_thread = threading.Thread(
                target=self._runtime_monitor_loop,
                name="demo_runtime_status_monitor",
                daemon=True,
            )
            self._runtime_thread.start()

    def _runtime_monitor_loop(self) -> None:
        try:
            import rclpy
            from action_msgs.msg import GoalStatus, GoalStatusArray
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
        except Exception as exc:
            self._update_runtime_status(monitor_running=False, monitor_error=f"monitor import failed: {exc}")
            self._runtime_started_event.set()
            return

        node = None
        executor = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            node = rclpy.create_node("demo_runtime_state_monitor")
            executor = SingleThreadedExecutor()
            executor.add_node(node)

            def on_exploration_status(msg: String) -> None:
                parsed = self._parse_exploration_status(msg.data)
                state = parsed["state"]
                cycle = parsed["cycle"]
                total = parsed["total_cycles"]
                mapping_finished = bool(
                    state == "IDLE"
                    and isinstance(cycle, int)
                    and isinstance(total, int)
                    and cycle >= total
                )
                self._update_runtime_status(
                    exploration_status_raw=msg.data,
                    exploration_state=state,
                    exploration_cycle=cycle,
                    exploration_total_cycles=total,
                    exploration_goals_sent=parsed["sent"],
                    exploration_goals_ok=parsed["ok"],
                    exploration_goals_fail=parsed["fail"],
                    mapping_finished=mapping_finished,
                )

            def on_simple_nav_status(msg: String) -> None:
                text = msg.data.strip()
                has_goal = self._runtime_status.get("navigation_has_goal", False)
                goal_reached = self._runtime_status.get("navigation_goal_reached", False)
                if text.startswith("idle"):
                    has_goal = False
                elif text.startswith("navigating"):
                    has_goal = True
                    goal_reached = False

                self._update_runtime_status(
                    navigation_simple_status_raw=text,
                    navigation_has_goal=has_goal,
                    navigation_goal_reached=goal_reached,
                )

            def on_action_status(msg: GoalStatusArray) -> None:
                if not msg.status_list:
                    return

                latest = max(
                    msg.status_list,
                    key=lambda item: (
                        int(item.goal_info.stamp.sec),
                        int(item.goal_info.stamp.nanosec),
                    ),
                )
                code = int(latest.status)
                action_status = self._goal_status_name(code)
                updates: Dict[str, Any] = {
                    "navigation_action_status": action_status,
                }

                if code in (
                    GoalStatus.STATUS_ACCEPTED,
                    GoalStatus.STATUS_EXECUTING,
                    GoalStatus.STATUS_CANCELING,
                ):
                    updates["navigation_has_goal"] = True
                    updates["navigation_goal_reached"] = False
                elif code == GoalStatus.STATUS_SUCCEEDED:
                    updates["navigation_has_goal"] = False
                    updates["navigation_goal_reached"] = True
                    updates["navigation_last_result"] = "succeeded"
                elif code == GoalStatus.STATUS_CANCELED:
                    updates["navigation_has_goal"] = False
                    updates["navigation_goal_reached"] = False
                    updates["navigation_last_result"] = "canceled"
                elif code == GoalStatus.STATUS_ABORTED:
                    updates["navigation_has_goal"] = False
                    updates["navigation_goal_reached"] = False
                    updates["navigation_last_result"] = "aborted"

                self._update_runtime_status(**updates)

            node.create_subscription(String, "/exploration/status", on_exploration_status, 20)
            node.create_subscription(String, "/navigation/simple_nav_status", on_simple_nav_status, 20)
            node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", on_action_status, 20)

            self._update_runtime_status(monitor_running=True, monitor_error=None)
            self._runtime_started_event.set()

            while not self._runtime_stop_event.is_set():
                executor.spin_once(timeout_sec=0.2)
        except Exception as exc:
            self._update_runtime_status(monitor_error=f"monitor runtime failed: {exc}")
            self._runtime_started_event.set()
        finally:
            self._update_runtime_status(monitor_running=False)
            if node is not None:
                if executor is not None:
                    try:
                        executor.remove_node(node)
                    except Exception:
                        pass
                node.destroy_node()
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:
                    pass

    def runtime_status(self) -> Dict[str, Any]:
        self._ensure_runtime_monitor()
        self._runtime_started_event.wait(timeout=0.2)
        with self._runtime_lock:
            return dict(self._runtime_status)

    def shutdown(self, join_timeout_sec: float = 1.0) -> None:
        self._runtime_stop_event.set()
        thread = self._runtime_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=max(0.0, join_timeout_sec))

    @property
    def mode(self) -> str:
        return self._mode

    @staticmethod
    def _derive_mode(
        mapping_status: ProcessStatus,
        navigation_status: ProcessStatus,
        fallback_mode: str,
    ) -> str:
        if mapping_status.running and not navigation_status.running:
            return "mapping"
        if navigation_status.running and not mapping_status.running:
            return "navigation"
        if mapping_status.running and navigation_status.running:
            return "conflict"
        return "error" if fallback_mode == "error" else "idle"

    def start_mapping(self, launch_args: Optional[Dict[str, str]] = None) -> bool:
        with self._lock:
            self.navigation.stop()
            ok = self.mapping.start(launch_args=launch_args)
            self._mode = "mapping" if ok else "error"
            if ok:
                self._ensure_runtime_monitor()
                self._update_runtime_status(mapping_finished=False)
            return ok

    def start_navigation(
        self,
        launch_args: Optional[Dict[str, str]] = None,
        force_restart: bool = False,
    ) -> bool:
        with self._lock:
            if force_restart:
                self.navigation.stop()
            self.mapping.stop()
            self._force_stop_mapping_processes(
                stop_hardware=self._launch_args_start_hardware(launch_args)
            )
            ok = self.navigation.start(launch_args=launch_args)
            self._mode = "navigation" if ok else "error"
            if ok:
                self._ensure_runtime_monitor()
            return ok

    @staticmethod
    def _launch_args_start_hardware(launch_args: Optional[Dict[str, str]]) -> bool:
        if not launch_args or "start_hardware" not in launch_args:
            return True
        return str(launch_args["start_hardware"]).strip().lower() in {
            "1",
            "true",
            "yes",
            "on",
        }

    def _force_stop_mapping_processes(self, stop_hardware: bool) -> None:
        patterns = [
            "ros2 launch robot_bringup mapping.launch.py",
            "ros2 launch robot_bringup real_robot.launch.py",
            "cartographer_node",
            "cartographer_occupancy_grid_node",
            "frontier_explorer",
        ]
        if stop_hardware:
            patterns.extend([
                "ros_robot_controller",
                "rplidar_node",
                "imu_cartographer_publisher",
                "odom_publisher",
                "wheel_joint_state_publisher",
                "rf2o_laser_odometry_node",
                "ekf_node",
            ])

        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-INT", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
            except Exception:
                pass

        time.sleep(1.0)

        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-TERM", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
            except Exception:
                pass

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Dict[str, float]:
        half = yaw * 0.5
        return {
            "x": 0.0,
            "y": 0.0,
            "z": math.sin(half),
            "w": math.cos(half),
        }

    def send_navigation_goal(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str = "map",
        topic: str = "/goal_pose",
        wait_for_subscribers_sec: float = 15.0,
        publish_times: int = 3,
    ) -> bool:
        """Publish a goal pose to the navigation goal topic.

        This is intentionally decoupled from start_navigation() so external
        workflows can start navigation first, then send goals any time later.
        """
        if publish_times < 1:
            publish_times = 1

        with self._lock:
            nav_running = self.navigation.is_running()

        if not nav_running:
            self.navigation._last_error = "navigation is not running"
            return False

        self._ensure_runtime_monitor()

        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped
            from rclpy.executors import SingleThreadedExecutor
        except Exception as exc:
            self.navigation._last_error = f"goal send failed (import): {exc}"
            return False

        initialized_here = False
        node = None
        executor = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                initialized_here = True

            node = rclpy.create_node("demo_navigation_goal_sender")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            pub = node.create_publisher(PoseStamped, topic, 10)

            deadline = time.time() + max(0.0, wait_for_subscribers_sec)
            while time.time() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if pub.get_subscription_count() > 0:
                    break

            if pub.get_subscription_count() == 0:
                self.navigation._last_error = (
                    f"goal send failed: no subscribers on {topic}"
                )
                return False

            quat = self._yaw_to_quaternion(yaw)
            msg = PoseStamped()
            msg.header.frame_id = frame_id
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = quat["x"]
            msg.pose.orientation.y = quat["y"]
            msg.pose.orientation.z = quat["z"]
            msg.pose.orientation.w = quat["w"]

            for _ in range(publish_times):
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                executor.spin_once(timeout_sec=0.05)

            self.navigation._last_error = None
            self._update_runtime_status(
                navigation_has_goal=True,
                navigation_goal_reached=False,
            )
            return True
        except Exception as exc:
            self.navigation._last_error = f"goal send failed: {exc}"
            return False
        finally:
            if node is not None:
                if executor is not None:
                    try:
                        executor.remove_node(node)
                    except Exception:
                        pass
                node.destroy_node()
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:
                    pass
            if initialized_here:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

    def stop_all(self) -> bool:
        with self._lock:
            ok_nav = self.navigation.stop()
            ok_map = self.mapping.stop()
            self._force_stop_all_processes()
            self._mode = "idle"
            self._update_runtime_status(navigation_has_goal=False)
            return ok_nav and ok_map

    def _force_stop_all_processes(self) -> None:
        patterns = [
            "ros2 launch robot_bringup mapping.launch.py",
            "ros2 launch robot_bringup real_robot.launch.py",
            "ros2 launch car_nav2 real_car_nav2.launch.py",
            "cartographer_node",
            "cartographer_occupancy_grid_node",
            "frontier_explorer",
            "map_server",
            "amcl",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "behavior_server",
            "recoveries_server",
            "waypoint_follower",
            "lifecycle_manager",
            "rviz2",
            "ros_robot_controller",
            "rplidar_node",
            "imu_cartographer_publisher",
            "odom_publisher",
            "wheel_joint_state_publisher",
            "rf2o_laser_odometry_node",
            "ekf_node",
        ]

        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-INT", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
            except Exception:
                pass

        time.sleep(1.0)

        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-TERM", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
            except Exception:
                pass

        time.sleep(0.5)

        for pattern in patterns:
            try:
                subprocess.run(
                    ["pkill", "-KILL", "-f", pattern],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=1.0,
                    check=False,
                )
            except Exception:
                pass

    def wait_navigation(
        self,
        action_name: str = "navigate_to_pose",
        timeout_sec: float = 3.0,
    ) -> bool:
        """Stop the robot in-place by canceling active navigation goals.

        This keeps navigation processes alive and only interrupts current motion.
        """
        with self._lock:
            nav_running = self.navigation.is_running()

        if not nav_running:
            self.navigation._last_error = "navigation is not running"
            return False

        self._ensure_runtime_monitor()

        try:
            import rclpy
            from action_msgs.msg import GoalInfo
            from action_msgs.srv import CancelGoal
            from rclpy.executors import SingleThreadedExecutor
        except Exception as exc:
            self.navigation._last_error = f"wait failed (import): {exc}"
            return False

        initialized_here = False
        node = None
        executor = None
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
                initialized_here = True

            node = rclpy.create_node("demo_navigation_wait_controller")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            cancel_service = f"{action_name}/_action/cancel_goal"
            client = node.create_client(CancelGoal, cancel_service)

            deadline = time.time() + max(0.1, timeout_sec)
            while time.time() < deadline:
                if client.wait_for_service(timeout_sec=0.1):
                    break

            if not client.service_is_ready():
                self.navigation._last_error = (
                    f"wait failed: cancel service unavailable ({cancel_service})"
                )
                return False

            req = CancelGoal.Request()
            req.goal_info = GoalInfo()
            future = client.call_async(req)

            while time.time() < deadline:
                executor.spin_once(timeout_sec=0.05)
                if future.done():
                    break

            if not future.done():
                self.navigation._last_error = "wait failed: cancel request timeout"
                return False

            resp = future.result()
            if resp is None:
                self.navigation._last_error = "wait failed: empty cancel response"
                return False

            ok_codes = {
                CancelGoal.Response.ERROR_NONE,
                CancelGoal.Response.ERROR_GOAL_TERMINATED,
                CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
            }
            if resp.return_code in ok_codes:
                self.navigation._last_error = None
                self._update_runtime_status(
                    navigation_has_goal=False,
                    navigation_goal_reached=False,
                )
                return True

            self.navigation._last_error = (
                f"wait failed: cancel return_code={resp.return_code}"
            )
            return False
        except Exception as exc:
            self.navigation._last_error = f"wait failed: {exc}"
            return False
        finally:
            if node is not None:
                if executor is not None:
                    try:
                        executor.remove_node(node)
                    except Exception:
                        pass
                node.destroy_node()
            if executor is not None:
                try:
                    executor.shutdown()
                except Exception:
                    pass
            if initialized_here:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass

    def status(self) -> Dict[str, Any]:
        with self._lock:
            mapping_status = self.mapping.status()
            navigation_status = self.navigation.status()
            mode = self._derive_mode(mapping_status, navigation_status, self._mode)
            return {
                "mode": mode,
                "mapping": mapping_status,
                "navigation": navigation_status,
                "runtime": self.runtime_status(),
            }
