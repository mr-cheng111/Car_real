"""Small process controllers for this workspace's launch files.

This is adapted from the external demo folder. The original scripts expected
an `exploration` package, which is not present in this workspace.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional


@dataclass
class ProcessStatus:
    name: str
    running: bool
    pid: Optional[int]
    started_at: Optional[float]
    last_error: Optional[str]


class LaunchProcessController:
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

    def _build_command(self, launch_args: Optional[Dict[str, str]] = None) -> List[str]:
        args = dict(self.default_args)
        if launch_args:
            args.update(launch_args)
        cmd = ["ros2", "launch", self.package, self.launch_file]
        cmd.extend(f"{key}:={value}" for key, value in args.items())
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
            log_path = os.path.join(
                self.log_dir,
                f"{self.name}_{time.strftime('%Y%m%d_%H%M%S')}.log",
            )
            cmd = self._build_command(launch_args)
            self._last_error = None

            try:
                self._log_fp = open(log_path, "a", encoding="utf-8")
                self._process = subprocess.Popen(
                    cmd,
                    stdout=self._log_fp,
                    stderr=subprocess.STDOUT,
                    text=True,
                    start_new_session=True,
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
            process = self._process
            if process is None:
                pid = self._read_pid_file()
                if pid is None or not self._is_pid_running(pid):
                    self._clear_pid_file()
                    return True
                try:
                    os.killpg(pid, signal.SIGINT)
                except Exception as exc:
                    self._last_error = f"stop failed: {exc}"
                    return False
            else:
                if process.poll() is not None:
                    self._cleanup_after_exit()
                    return True
                os.killpg(process.pid, signal.SIGINT)

            deadline = time.time() + timeout_sec
            while time.time() < deadline:
                if not self.is_running():
                    self._cleanup_after_exit()
                    return True
                time.sleep(0.2)

            try:
                target_pid = process.pid if process is not None else pid
                os.killpg(target_pid, signal.SIGTERM)
            except Exception:
                pass
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
        return ProcessStatus(
            name=self.name,
            running=self.is_running(),
            pid=pid,
            started_at=self._started_at,
            last_error=self._last_error,
        )


class MappingController(LaunchProcessController):
    @staticmethod
    def _workspace_root() -> Path:
        current = Path(__file__).resolve()
        for parent in current.parents:
            if (parent / "src" / "car_nav2").exists():
                return parent
        return Path.cwd()

    @classmethod
    def _resolve_workspace_path(cls, path: str) -> str:
        expanded = Path(os.path.expanduser(path))
        if expanded.is_absolute():
            return str(expanded)
        return str(cls._workspace_root() / expanded)

    def __init__(self, log_dir: str = "./logs") -> None:
        self.pbstream_path = self._resolve_workspace_path(
            os.environ.get(
                "ROBOT_BRINGUP_PBSTREAM_PATH",
                "src/car_nav2/maps/cartographer/latest.pbstream",
            )
        )
        super().__init__(
            name="mapping",
            package=os.environ.get("ROBOT_BRINGUP_MAPPING_PACKAGE", "robot_bringup"),
            launch_file=os.environ.get("ROBOT_BRINGUP_MAPPING_LAUNCH", "mapping.launch.py"),
            default_args={},
            log_dir=log_dir,
        )

    def save_pbstream(self, timeout_sec: float = 25.0) -> bool:
        try:
            result = subprocess.run(
                [
                    "ros2", "run", "robot_bringup", "save_cartographer_state.py",
                    "--output", self.pbstream_path,
                    "--timeout", str(max(1.0, timeout_sec)),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=max(2.0, timeout_sec + 5.0),
                check=False,
            )
            if result.returncode != 0:
                output = (result.stdout or "").strip()
                detail = f": {output}" if output else ""
                self._last_error = f"pbstream save failed: exit {result.returncode}{detail}"
                print(self._last_error)
                return False
            output = (result.stdout or "").strip()
            if output:
                print(output)
            print(f"pbstream saved: {self.pbstream_path}")
            return True
        except Exception as exc:
            self._last_error = f"pbstream save failed: {exc}"
            print(self._last_error)
            return False

    def stop(self, timeout_sec: float = 30.0) -> bool:
        if self.is_running():
            self.save_pbstream()
        return super().stop(timeout_sec=timeout_sec)


class NavigationController(LaunchProcessController):
    def __init__(self, log_dir: str = "./logs") -> None:
        super().__init__(
            name="navigation",
            package=os.environ.get("ROBOT_BRINGUP_NAVIGATION_PACKAGE", ""),
            launch_file=os.environ.get("ROBOT_BRINGUP_NAVIGATION_LAUNCH", ""),
            default_args={
                "start_sim": "false",
                "use_sim_time": "false",
                "use_rviz": "false",
            },
            log_dir=log_dir,
        )

    def start(self, launch_args: Optional[Dict[str, str]] = None) -> bool:
        if not self.package or not self.launch_file:
            self._last_error = (
                "navigation is not configured; set ROBOT_BRINGUP_NAVIGATION_PACKAGE "
                "and ROBOT_BRINGUP_NAVIGATION_LAUNCH if needed"
            )
            return False
        return super().start(launch_args=launch_args)


class ExplorationFlowController:
    """Mutually exclusive mapping/navigation launcher plus simple goal publisher."""

    def __init__(self, log_dir: str = "./logs") -> None:
        self.mapping = MappingController(log_dir=log_dir)
        self.navigation = NavigationController(log_dir=log_dir)
        self._mode = "idle"
        self._lock = threading.Lock()

    @property
    def mode(self) -> str:
        return self._mode

    def start_mapping(self, launch_args: Optional[Dict[str, str]] = None) -> bool:
        with self._lock:
            self.navigation.stop()
            ok = self.mapping.start(launch_args=launch_args)
            self._mode = "mapping" if ok else "error"
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
            ok = self.navigation.start(launch_args=launch_args)
            self._mode = "navigation" if ok else "error"
            return ok

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Dict[str, float]:
        half = yaw * 0.5
        return {"x": 0.0, "y": 0.0, "z": math.sin(half), "w": math.cos(half)}

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
        if not self.navigation.is_running():
            self.navigation._last_error = "navigation is not running"
            return False

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

            node = rclpy.create_node("robot_bringup_demo_goal_sender")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            pub = node.create_publisher(PoseStamped, topic, 10)

            deadline = time.time() + max(0.0, wait_for_subscribers_sec)
            while time.time() < deadline:
                executor.spin_once(timeout_sec=0.1)
                if pub.get_subscription_count() > 0:
                    break

            if pub.get_subscription_count() == 0:
                self.navigation._last_error = f"goal send failed: no subscribers on {topic}"
                return False

            quat = self._yaw_to_quaternion(yaw)
            msg = PoseStamped()
            msg.header.frame_id = frame_id
            msg.pose.position.x = float(x)
            msg.pose.position.y = float(y)
            msg.pose.orientation.x = quat["x"]
            msg.pose.orientation.y = quat["y"]
            msg.pose.orientation.z = quat["z"]
            msg.pose.orientation.w = quat["w"]

            for _ in range(max(1, publish_times)):
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                executor.spin_once(timeout_sec=0.05)
            self.navigation._last_error = None
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
                executor.shutdown()
            if initialized_here:
                rclpy.shutdown()

    def stop_all(self) -> bool:
        with self._lock:
            ok_nav = self.navigation.stop()
            ok_map = self.mapping.stop()
            self._mode = "idle"
            return ok_nav and ok_map

    def status(self) -> Dict[str, object]:
        mapping = self.mapping.status()
        navigation = self.navigation.status()
        if mapping.running and not navigation.running:
            mode = "mapping"
        elif navigation.running and not mapping.running:
            mode = "navigation"
        elif mapping.running and navigation.running:
            mode = "conflict"
        else:
            mode = "idle"
        return {"mode": mode, "mapping": mapping, "navigation": navigation}
