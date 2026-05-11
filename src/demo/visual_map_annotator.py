#!/usr/bin/env python3
"""Visual map annotation and named-goal navigation tool.

Workflow:
1. Start navigation with a saved occupancy-grid map.
2. Subscribe /map and render an interactive 2D map.
3. Click to create named points (saved as JSON).
4. Send navigation goals by point name.

Controls in map window:
- Left click: add/update a named point at clicked position
- g: send goal by name
- l: list all named points
- d: delete a named point by name
- q: quit
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict

import numpy as np

from ros2_launch_controllers import ExplorationFlowController


DEFAULT_MAP = "/home/mr-cheng/Car_real/src/exploration/map/exploration_map.yaml"
DEFAULT_DB = Path(__file__).resolve().parent / "named_points.json"
DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "logs"


@dataclass
class NamedPoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


def _load_points(db_path: Path) -> Dict[str, NamedPoint]:
    if not db_path.exists():
        return {}

    with db_path.open("r", encoding="utf-8") as fp:
        raw = json.load(fp)

    points: Dict[str, NamedPoint] = {}
    for name, item in raw.items():
        points[name] = NamedPoint(
            name=name,
            x=float(item["x"]),
            y=float(item["y"]),
            yaw=float(item.get("yaw", 0.0)),
            frame_id=str(item.get("frame_id", "map")),
        )
    return points


def _save_points(db_path: Path, points: Dict[str, NamedPoint]) -> None:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    raw = {name: asdict(pt) for name, pt in points.items()}
    with db_path.open("w", encoding="utf-8") as fp:
        json.dump(raw, fp, ensure_ascii=False, indent=2)


def _print_points(points: Dict[str, NamedPoint]) -> None:
    if not points:
        print("[points] empty")
        return

    print("[points]")
    for name in sorted(points.keys()):
        p = points[name]
        print(
            f"  - {p.name}: x={p.x:.3f}, y={p.y:.3f}, yaw={p.yaw:.3f}, frame={p.frame_id}"
        )


def _start_navigation(
    flow: ExplorationFlowController,
    map_yaml: str,
    disable_rviz: bool,
    start_hardware: bool,
) -> None:
    launch_args = {
        "map": map_yaml,
        "start_hardware": "true" if start_hardware else "false",
    }
    if disable_rviz:
        launch_args["use_rviz"] = "false"

    ok = flow.start_navigation(launch_args=launch_args)
    if not ok:
        status = flow.status()["navigation"]
        raise RuntimeError(f"navigation start failed: {status.last_error}")


class _MapReceiver:
    def __init__(self, topic: str = "/map") -> None:
        import rclpy
        from nav_msgs.msg import OccupancyGrid
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

        self.rclpy = rclpy
        self._inited_here = False
        if not self.rclpy.ok():
            self.rclpy.init(args=None)
            self._inited_here = True

        self.node = self.rclpy.create_node("visual_map_annotator_map_receiver")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._msg = None

        self.sub = self.node.create_subscription(OccupancyGrid, topic, self._on_map, qos)

    def _on_map(self, msg) -> None:
        self._msg = msg

    def wait_map(self, timeout_sec: float = 20.0):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            self.executor.spin_once(timeout_sec=0.2)
            if self._msg is not None:
                return self._msg
        return None

    def close(self) -> None:
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.executor.shutdown()
        except Exception:
            pass
        self.node.destroy_node()
        if self._inited_here:
            try:
                self.rclpy.shutdown()
            except Exception:
                pass


def _map_to_image(map_msg):
    h = int(map_msg.info.height)
    w = int(map_msg.info.width)
    grid = np.array(map_msg.data, dtype=np.int16).reshape((h, w))

    # unknown -> gray, occupied -> black, free -> white
    img = np.full((h, w), 205, dtype=np.uint8)
    known = grid >= 0
    occupied = grid >= 65
    free = known & (~occupied)

    img[occupied] = 0
    # Keep a soft gradient for known free cells.
    img[free] = np.clip(255 - (grid[free] * 2), 120, 255).astype(np.uint8)
    return img


def _warn_if_rotated_map(map_msg) -> None:
    q = map_msg.info.origin.orientation
    # For this tool we assume a nearly axis-aligned occupancy grid.
    if abs(q.z) > 1e-3 or abs(q.x) > 1e-3 or abs(q.y) > 1e-3:
        print(
            "[warn] map origin appears rotated; click-to-world conversion assumes axis-aligned map."
        )


def _run_cv2_ui(
    flow: ExplorationFlowController,
    points: Dict[str, NamedPoint],
    db_path: Path,
    map_msg,
) -> int:
    import cv2

    img = _map_to_image(map_msg)
    # Convert map image to OpenCV top-left origin view.
    img = np.flipud(img)
    base = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    res = float(map_msg.info.resolution)
    ox = float(map_msg.info.origin.position.x)
    oy = float(map_msg.info.origin.position.y)
    w = int(map_msg.info.width)
    h = int(map_msg.info.height)

    max_dim = max(h, w)
    scale = 1.0 if max_dim <= 1200 else 1200.0 / float(max_dim)

    window = "Map Annotator (cv2)"
    dirty = {"value": True}

    def world_to_px(x: float, y: float):
        gx = int(round((x - ox) / res))
        gy = int(round((y - oy) / res))
        px = gx
        py = (h - 1) - gy
        return px, py

    def px_to_world(px: int, py: int):
        gx = float(px)
        gy = float((h - 1) - py)
        x = ox + gx * res
        y = oy + gy * res
        return x, y

    def render_and_show():
        canvas = base.copy()
        for name, p in points.items():
            px, py = world_to_px(p.x, p.y)
            if 0 <= px < w and 0 <= py < h:
                cv2.circle(canvas, (px, py), 4, (0, 0, 255), -1)
                cv2.putText(
                    canvas,
                    name,
                    (px + 5, py - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )

        if scale != 1.0:
            view = cv2.resize(
                canvas,
                (int(w * scale), int(h * scale)),
                interpolation=cv2.INTER_NEAREST,
            )
        else:
            view = canvas

        cv2.imshow(window, view)

    def send_goal_by_name(name: str) -> None:
        p = points.get(name)
        if p is None:
            print(f"[goal] point not found: {name}")
            return

        ok = flow.send_navigation_goal(
            x=p.x,
            y=p.y,
            yaw=p.yaw,
            frame_id=p.frame_id,
            topic="/goal_pose",
            wait_for_subscribers_sec=15.0,
            publish_times=3,
        )
        if ok:
            print(f"[goal] sent to {name}: ({p.x:.3f}, {p.y:.3f}, yaw={p.yaw:.3f})")
        else:
            status = flow.status()["navigation"]
            print(f"[goal] send failed: {status.last_error}")

    def on_mouse(event, x, y, flags, param):
        del flags, param
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        rx = int(round(x / scale)) if scale != 1.0 else int(x)
        ry = int(round(y / scale)) if scale != 1.0 else int(y)
        if not (0 <= rx < w and 0 <= ry < h):
            return

        wx, wy = px_to_world(rx, ry)
        print(f"\n[click] x={wx:.3f}, y={wy:.3f}")
        name = input("point name (blank to cancel): ").strip()
        if not name:
            print("[point] cancelled")
            return

        yaw_raw = input("yaw in rad (blank=0): ").strip()
        yaw = 0.0
        if yaw_raw:
            try:
                yaw = float(yaw_raw)
            except ValueError:
                print("[point] invalid yaw, using 0.0")

        points[name] = NamedPoint(name=name, x=wx, y=wy, yaw=yaw, frame_id="map")
        _save_points(db_path, points)
        print(f"[point] saved {name}")
        dirty["value"] = True

    print("[ui] backend=cv2")
    print("[ui] controls: left-click add/update | g send goal | d delete | l list | q quit")
    _print_points(points)

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)

    try:
        while True:
            if dirty["value"]:
                render_and_show()
                dirty["value"] = False

            key = cv2.waitKey(50) & 0xFF
            if key == 255:
                continue
            if key == ord("q"):
                break
            if key == ord("l"):
                _print_points(points)
                continue
            if key == ord("d"):
                name = input("delete point name: ").strip()
                if name in points:
                    points.pop(name)
                    _save_points(db_path, points)
                    print(f"[point] deleted {name}")
                    dirty["value"] = True
                else:
                    print(f"[point] not found: {name}")
                continue
            if key == ord("g"):
                name = input("goal point name: ").strip()
                if name:
                    send_goal_by_name(name)
                continue
    finally:
        cv2.destroyAllWindows()

    return 0


def _run_matplotlib_ui(
    flow: ExplorationFlowController,
    points: Dict[str, NamedPoint],
    db_path: Path,
    map_msg,
) -> int:
    import matplotlib.pyplot as plt

    img = _map_to_image(map_msg)

    res = float(map_msg.info.resolution)
    ox = float(map_msg.info.origin.position.x)
    oy = float(map_msg.info.origin.position.y)
    w = int(map_msg.info.width)
    h = int(map_msg.info.height)

    xmin = ox
    xmax = ox + w * res
    ymin = oy
    ymax = oy + h * res

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(
        img,
        cmap="gray",
        origin="lower",
        extent=[xmin, xmax, ymin, ymax],
        interpolation="nearest",
    )
    ax.set_title(
        "Map Annotator | Left click: add point | g: go to name | d: delete | l: list | q: quit"
    )
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    ax.grid(True, alpha=0.2)

    scatter = None
    labels = []

    def redraw_points() -> None:
        nonlocal scatter, labels
        if scatter is not None:
            scatter.remove()
            scatter = None
        for t in labels:
            t.remove()
        labels = []

        if not points:
            fig.canvas.draw_idle()
            return

        xs = [points[k].x for k in points]
        ys = [points[k].y for k in points]
        scatter = ax.scatter(xs, ys, c="red", s=38)
        for name, p in points.items():
            labels.append(ax.text(p.x, p.y, f" {name}", color="red", fontsize=9))

        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax or event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        x = float(event.xdata)
        y = float(event.ydata)
        print(f"\n[click] x={x:.3f}, y={y:.3f}")
        name = input("point name (blank to cancel): ").strip()
        if not name:
            print("[point] cancelled")
            return

        yaw_raw = input("yaw in rad (blank=0): ").strip()
        yaw = 0.0
        if yaw_raw:
            try:
                yaw = float(yaw_raw)
            except ValueError:
                print("[point] invalid yaw, using 0.0")

        points[name] = NamedPoint(name=name, x=x, y=y, yaw=yaw, frame_id="map")
        _save_points(db_path, points)
        print(f"[point] saved {name}")
        redraw_points()

    def send_goal_by_name(name: str) -> None:
        p = points.get(name)
        if p is None:
            print(f"[goal] point not found: {name}")
            return

        ok = flow.send_navigation_goal(
            x=p.x,
            y=p.y,
            yaw=p.yaw,
            frame_id=p.frame_id,
            topic="/goal_pose",
            wait_for_subscribers_sec=15.0,
            publish_times=3,
        )
        if ok:
            print(f"[goal] sent to {name}: ({p.x:.3f}, {p.y:.3f}, yaw={p.yaw:.3f})")
        else:
            status = flow.status()["navigation"]
            print(f"[goal] send failed: {status.last_error}")

    def on_key(event):
        key = (event.key or "").lower()
        if key == "q":
            plt.close(fig)
            return
        if key == "l":
            _print_points(points)
            return
        if key == "d":
            name = input("delete point name: ").strip()
            if not name:
                return
            if name in points:
                points.pop(name)
                _save_points(db_path, points)
                print(f"[point] deleted {name}")
                redraw_points()
            else:
                print(f"[point] not found: {name}")
            return
        if key == "g":
            name = input("goal point name: ").strip()
            if not name:
                return
            send_goal_by_name(name)
            return

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)

    redraw_points()
    _print_points(points)
    plt.tight_layout()
    plt.show()
    return 0


def run_visual_annotator(
    map_yaml: str,
    db_path: Path,
    map_topic: str,
    disable_rviz: bool,
    start_hardware: bool,
    map_wait_sec: float,
    backend: str,
) -> int:
    flow = ExplorationFlowController(log_dir=str(DEFAULT_LOG_DIR))
    print(f"[nav] starting navigation, map={map_yaml}")
    _start_navigation(
        flow=flow,
        map_yaml=map_yaml,
        disable_rviz=disable_rviz,
        start_hardware=start_hardware,
    )

    receiver = _MapReceiver(topic=map_topic)
    map_msg = receiver.wait_map(timeout_sec=map_wait_sec)
    receiver.close()

    if map_msg is None:
        print(f"[map] failed to receive {map_topic} in {map_wait_sec:.1f}s")
        return 2

    _warn_if_rotated_map(map_msg)

    points = _load_points(db_path)

    backend = backend.lower().strip()
    if backend not in ("auto", "cv2", "matplotlib"):
        print(f"unknown backend: {backend}")
        return 3

    if backend in ("auto", "cv2"):
        try:
            return _run_cv2_ui(flow=flow, points=points, db_path=db_path, map_msg=map_msg)
        except Exception as exc:
            if backend == "cv2":
                print(f"cv2 backend failed: {exc}")
                return 4
            print(f"[ui] cv2 backend unavailable, fallback to matplotlib: {exc}")

    try:
        return _run_matplotlib_ui(flow=flow, points=points, db_path=db_path, map_msg=map_msg)
    except Exception as exc:
        print(f"matplotlib backend failed: {exc}")
        print("please install a compatible matplotlib or use --backend cv2")
        return 5


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Visual /map annotator + named-point navigation tool"
    )
    parser.add_argument(
        "--map",
        default=DEFAULT_MAP,
        help="map yaml path used by Nav2 map_server and AMCL",
    )
    parser.add_argument(
        "--db",
        default=str(DEFAULT_DB),
        help="named points JSON path",
    )
    parser.add_argument(
        "--map-topic",
        default="/map",
        help="map topic to visualize",
    )
    parser.add_argument(
        "--enable-rviz",
        action="store_true",
        help="keep RViz enabled in navigation launch",
    )
    parser.add_argument(
        "--map-wait-sec",
        type=float,
        default=25.0,
        help="wait timeout for /map reception",
    )
    parser.add_argument(
        "--backend",
        default="auto",
        choices=["auto", "cv2", "matplotlib"],
        help="ui backend: auto (prefer cv2), cv2, or matplotlib",
    )
    parser.add_argument(
        "--list-points",
        action="store_true",
        help="List named points in db and exit",
    )
    parser.add_argument(
        "--delete-point",
        default="",
        help="Delete one named point from db and exit",
    )

    args = parser.parse_args()

    db_path = Path(args.db).resolve()

    # Database-only operations (no ROS launch / no UI)
    if args.list_points or args.delete_point:
        points = _load_points(db_path)
        if args.delete_point:
            name = str(args.delete_point).strip()
            if not name:
                print("delete-point is empty")
                return 1
            if name not in points:
                print(f"[point] not found: {name}")
                _print_points(points)
                return 1
            points.pop(name)
            _save_points(db_path, points)
            print(f"[point] deleted {name}")
        if args.list_points or args.delete_point:
            _print_points(points)
        return 0

    map_yaml = os.path.abspath(args.map)
    if not os.path.exists(map_yaml):
        print(f"map yaml not found: {map_yaml}")
        return 1

    return run_visual_annotator(
        map_yaml=map_yaml,
        db_path=db_path,
        map_topic=args.map_topic,
        disable_rviz=(not args.enable_rviz),
        start_hardware=True,
        map_wait_sec=float(args.map_wait_sec),
        backend=str(args.backend),
    )


if __name__ == "__main__":
    raise SystemExit(main())
