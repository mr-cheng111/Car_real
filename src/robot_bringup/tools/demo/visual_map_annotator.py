#!/usr/bin/env python3
"""Visual /map annotator and named-point goal sender.

This tool does not start navigation by default; it subscribes to an existing
`/map`, lets the user save named points, and can publish a selected point to
`/goal_pose`.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict

import numpy as np


DEFAULT_DB = Path(__file__).resolve().parent / "named_points.json"


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
    return {
        name: NamedPoint(
            name=name,
            x=float(item["x"]),
            y=float(item["y"]),
            yaw=float(item.get("yaw", 0.0)),
            frame_id=str(item.get("frame_id", "map")),
        )
        for name, item in raw.items()
    }


def _save_points(db_path: Path, points: Dict[str, NamedPoint]) -> None:
    db_path.parent.mkdir(parents=True, exist_ok=True)
    raw = {name: asdict(point) for name, point in points.items()}
    with db_path.open("w", encoding="utf-8") as fp:
        json.dump(raw, fp, ensure_ascii=False, indent=2)


def _print_points(points: Dict[str, NamedPoint]) -> None:
    if not points:
        print("[points] empty")
        return
    for name in sorted(points):
        point = points[name]
        print(f"- {name}: x={point.x:.3f}, y={point.y:.3f}, yaw={point.yaw:.3f}, frame={point.frame_id}")


def _wait_map(topic: str, timeout_sec: float):
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True

    node = rclpy.create_node("robot_bringup_map_annotator")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    state = {"msg": None}
    node.create_subscription(OccupancyGrid, topic, lambda msg: state.update(msg=msg), qos)

    import time

    deadline = time.time() + timeout_sec
    try:
        while time.time() < deadline:
            executor.spin_once(timeout_sec=0.2)
            if state["msg"] is not None:
                return state["msg"]
        return None
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def _map_to_image(map_msg):
    height = int(map_msg.info.height)
    width = int(map_msg.info.width)
    grid = np.array(map_msg.data, dtype=np.int16).reshape((height, width))
    img = np.full((height, width), 205, dtype=np.uint8)
    known = grid >= 0
    occupied = grid >= 65
    free = known & (~occupied)
    img[occupied] = 0
    img[free] = np.clip(255 - (grid[free] * 2), 120, 255).astype(np.uint8)
    return img


def _send_goal(point: NamedPoint, topic: str) -> bool:
    import rclpy
    from geometry_msgs.msg import PoseStamped

    initialized_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialized_here = True
    node = rclpy.create_node("robot_bringup_named_goal_sender")
    pub = node.create_publisher(PoseStamped, topic, 10)
    msg = PoseStamped()
    msg.header.frame_id = point.frame_id
    msg.pose.position.x = point.x
    msg.pose.position.y = point.y
    half = point.yaw * 0.5
    msg.pose.orientation.z = math.sin(half)
    msg.pose.orientation.w = math.cos(half)
    for _ in range(3):
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    if initialized_here:
        rclpy.shutdown()
    return True


def run(args) -> int:
    db_path = Path(args.db).resolve()
    points = _load_points(db_path)
    if args.list_points:
        _print_points(points)
        return 0

    map_msg = _wait_map(args.map_topic, args.map_wait_sec)
    if map_msg is None:
        print(f"failed to receive {args.map_topic}")
        return 2

    import matplotlib.pyplot as plt

    img = _map_to_image(map_msg)
    res = float(map_msg.info.resolution)
    ox = float(map_msg.info.origin.position.x)
    oy = float(map_msg.info.origin.position.y)
    width = int(map_msg.info.width)
    height = int(map_msg.info.height)

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(
        img,
        cmap="gray",
        origin="lower",
        extent=[ox, ox + width * res, oy, oy + height * res],
        interpolation="nearest",
    )
    ax.set_title("Left click: save point | g: send goal | l: list | q: quit")
    ax.set_xlabel("map x (m)")
    ax.set_ylabel("map y (m)")
    labels = []

    def redraw() -> None:
        while labels:
            labels.pop().remove()
        for point in points.values():
            labels.append(ax.plot(point.x, point.y, "ro")[0])
            labels.append(ax.text(point.x, point.y, f" {point.name}", color="red"))
        fig.canvas.draw_idle()

    def on_click(event) -> None:
        if event.inaxes != ax or event.button != 1 or event.xdata is None or event.ydata is None:
            return
        name = input(f"point name for x={event.xdata:.3f}, y={event.ydata:.3f}: ").strip()
        if not name:
            return
        yaw_raw = input("yaw in rad (blank=0): ").strip()
        yaw = float(yaw_raw) if yaw_raw else 0.0
        points[name] = NamedPoint(name=name, x=float(event.xdata), y=float(event.ydata), yaw=yaw)
        _save_points(db_path, points)
        redraw()

    def on_key(event) -> None:
        key = (event.key or "").lower()
        if key == "q":
            plt.close(fig)
        elif key == "l":
            _print_points(points)
        elif key == "g":
            name = input("goal point name: ").strip()
            point = points.get(name)
            if point is None:
                print(f"point not found: {name}")
            else:
                _send_goal(point, args.goal_topic)
                print(f"goal sent: {name}")

    fig.canvas.mpl_connect("button_press_event", on_click)
    fig.canvas.mpl_connect("key_press_event", on_key)
    redraw()
    plt.tight_layout()
    plt.show()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Annotate /map and publish named goals")
    parser.add_argument("--db", default=str(DEFAULT_DB))
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--map-wait-sec", type=float, default=25.0)
    parser.add_argument("--list-points", action="store_true")
    return run(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
