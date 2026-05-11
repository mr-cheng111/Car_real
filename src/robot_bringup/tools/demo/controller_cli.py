#!/usr/bin/env python3
"""CLI for robot_bringup demo mapping/navigation helpers."""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
import time
from typing import Dict

from ros2_launch_controllers import ExplorationFlowController


def _default_points_db() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "named_points.json")


def _load_named_points(db_path: str) -> Dict[str, dict]:
    if not os.path.exists(db_path):
        return {}
    try:
        with open(db_path, "r", encoding="utf-8") as fp:
            raw = json.load(fp)
    except Exception:
        return {}
    return raw if isinstance(raw, dict) else {}


def _save_named_points(db_path: str, points: Dict[str, dict]) -> None:
    with open(db_path, "w", encoding="utf-8") as fp:
        json.dump(points, fp, ensure_ascii=False, indent=2)


def _print_named_points(points: Dict[str, dict]) -> None:
    if not points:
        print("named points: empty")
        return
    print("named points:")
    for name in sorted(points):
        p = points[name]
        print(
            f"- {name}: x={float(p['x']):.3f}, y={float(p['y']):.3f}, "
            f"yaw={float(p.get('yaw', 0.0)):.3f}, frame={p.get('frame_id', 'map')}"
        )


def _print_status(flow: ExplorationFlowController) -> None:
    status = flow.status()
    print(f"mode: {status['mode']}")
    for name in ("mapping", "navigation"):
        s = status[name]
        started_at = (
            time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(s.started_at))
            if s.started_at
            else "-"
        )
        print(
            f"{name}: running={s.running}, pid={s.pid}, "
            f"started_at={started_at}, last_error={s.last_error}"
        )


def _send_goal_by_name(flow, name: str, db_path: str, topic: str, wait: float, times: int) -> bool:
    points = _load_named_points(db_path)
    target = points.get(name)
    if target is None:
        print(f"named point not found: {name}")
        _print_named_points(points)
        return False
    return flow.send_navigation_goal(
        x=float(target["x"]),
        y=float(target["y"]),
        yaw=float(target.get("yaw", 0.0)),
        frame_id=str(target.get("frame_id", "map")),
        topic=topic,
        wait_for_subscribers_sec=wait,
        publish_times=times,
    )


def _execute(args: argparse.Namespace, parser: argparse.ArgumentParser, flow: ExplorationFlowController) -> int:
    action = args.action
    name = args.name
    known = {
        "mapping",
        "navigation",
        "goal",
        "goal-name",
        "save-map",
        "stop",
        "status",
        "points",
        "delete-point",
    }
    if not action:
        print(parser.format_usage().strip())
        return 2
    if action not in known:
        name = action
        action = "goal-name"

    if action == "mapping":
        ok = flow.start_mapping()
        print("mapping started" if ok else "mapping start failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "navigation":
        launch_args = {"use_rviz": "true" if args.rviz else "false"}
        ok = flow.start_navigation(launch_args=launch_args, force_restart=args.restart)
        print("navigation started" if ok else "navigation start failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "save-map":
        ok = flow.mapping.save_pbstream(timeout_sec=args.save_timeout)
        _print_status(flow)
        return 0 if ok else 1

    if action == "goal":
        if args.x is None or args.y is None:
            print("goal requires --x and --y")
            return 2
        ok = flow.send_navigation_goal(
            x=args.x,
            y=args.y,
            yaw=args.yaw,
            frame_id=args.frame_id,
            topic=args.topic,
            wait_for_subscribers_sec=args.wait_subscribers,
            publish_times=args.publish_times,
        )
        print("goal sent" if ok else "goal send failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "goal-name":
        if not name:
            print("goal-name requires a point name")
            return 2
        ok = _send_goal_by_name(flow, name, args.points_db, args.topic, args.wait_subscribers, args.publish_times)
        print("goal sent" if ok else "goal send failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "points":
        _print_named_points(_load_named_points(args.points_db))
        return 0

    if action == "delete-point":
        if not name:
            print("delete-point requires a point name")
            return 2
        points = _load_named_points(args.points_db)
        points.pop(name, None)
        _save_named_points(args.points_db, points)
        _print_named_points(points)
        return 0

    if action == "stop":
        ok = flow.stop_all()
        print("all stopped" if ok else "stop failed")
        _print_status(flow)
        return 0 if ok else 1

    _print_status(flow)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Control robot_bringup demo launches")
    parser.add_argument("action", nargs="?")
    parser.add_argument("name", nargs="?")
    parser.add_argument("--x", type=float)
    parser.add_argument("--y", type=float)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--frame-id", default="map")
    parser.add_argument("--topic", default="/goal_pose")
    parser.add_argument("--wait-subscribers", type=float, default=15.0)
    parser.add_argument("--publish-times", type=int, default=3)
    parser.add_argument("--points-db", default=_default_points_db())
    parser.add_argument("--save-timeout", type=float, default=25.0)
    parser.add_argument("--rviz", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--restart", action=argparse.BooleanOptionalAction, default=True)

    if len(sys.argv) == 1:
        flow = ExplorationFlowController(log_dir="./logs")
        print("Interactive mode. Type q to quit.")
        while True:
            line = input("controller> ").strip()
            if line.lower() in {"q", "quit", "exit"}:
                return 0
            if not line:
                continue
            try:
                args = parser.parse_args(shlex.split(line))
            except SystemExit:
                continue
            _execute(args, parser, flow)

    args = parser.parse_args()
    flow = ExplorationFlowController(log_dir="./logs")
    return _execute(args, parser, flow)


if __name__ == "__main__":
    raise SystemExit(main())
