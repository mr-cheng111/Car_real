"""Simple CLI for switching exploration modes.

Examples:
    python3 controller_cli.py mapping
    python3 controller_cli.py navigation
    python3 controller_cli.py goal --x 1.0 --y 0.5 --yaw 1.57
    python controller_cli.py home              # navigate to named point "home"
    python controller_cli.py wait              # stop current motion, keep nav running
    python controller_cli.py points            # list named points
    python controller_cli.py delete-point home # delete named point
    python controller_cli.py stop
    python controller_cli.py status
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import sys
import time
from typing import Any, Dict

from ros2_launch_controllers import ExplorationFlowController


def _default_points_db() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "named_points.json")


def _default_log_dir() -> str:
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")


def _default_map_yaml() -> str:
    return "/home/mr-cheng/Car_real/src/exploration/map/exploration_map.yaml"


def _load_named_points(db_path: str) -> Dict[str, Dict[str, Any]]:
    if not os.path.exists(db_path):
        return {}

    try:
        with open(db_path, "r", encoding="utf-8") as fp:
            raw = json.load(fp)
    except Exception:
        return {}

    points: Dict[str, Dict[str, Any]] = {}
    if isinstance(raw, dict):
        for name, item in raw.items():
            if not isinstance(item, dict):
                continue
            if "x" not in item or "y" not in item:
                continue
            points[name] = {
                "x": float(item["x"]),
                "y": float(item["y"]),
                "yaw": float(item.get("yaw", 0.0)),
                "frame_id": str(item.get("frame_id", "map")),
            }
    return points


def _save_named_points(db_path: str, points: Dict[str, Dict[str, Any]]) -> None:
    with open(db_path, "w", encoding="utf-8") as fp:
        json.dump(points, fp, ensure_ascii=False, indent=2)


def _print_named_points(points: Dict[str, Dict[str, Any]]) -> None:
    if not points:
        print("named points: empty")
        return

    print("named points:")
    for name in sorted(points.keys()):
        p = points[name]
        print(
            f"- {name}: x={p['x']:.3f}, y={p['y']:.3f}, yaw={p['yaw']:.3f}, frame={p['frame_id']}"
        )


def _send_goal_by_name(
    flow: ExplorationFlowController,
    name: str,
    db_path: str,
    topic: str,
    wait_subscribers: float,
    publish_times: int,
) -> bool:
    points = _load_named_points(db_path)
    target = points.get(name)
    if target is None:
        print(f"named point not found: {name}")
        _print_named_points(points)
        return False

    print(
        f"navigate to {name}: x={target['x']:.3f}, y={target['y']:.3f}, "
        f"yaw={target['yaw']:.3f}, frame={target['frame_id']}"
    )
    return flow.send_navigation_goal(
        x=target["x"],
        y=target["y"],
        yaw=target["yaw"],
        frame_id=target["frame_id"],
        topic=topic,
        wait_for_subscribers_sec=wait_subscribers,
        publish_times=publish_times,
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
            f"{name}: running={s.running}, pid={s.pid}, started_at={started_at}, "
            f"last_error={s.last_error}"
        )

    runtime = status.get("runtime")
    if isinstance(runtime, dict):
        print("runtime:")
        print(
            f"  monitor_running={runtime.get('monitor_running')}, "
            f"monitor_error={runtime.get('monitor_error')}"
        )
        print(
            f"  exploration_state={runtime.get('exploration_state')}, "
            f"cycle={runtime.get('exploration_cycle')}/{runtime.get('exploration_total_cycles')}, "
            f"mapping_finished={runtime.get('mapping_finished')}"
        )
        print(
            f"  navigation_has_goal={runtime.get('navigation_has_goal')}, "
            f"navigation_goal_reached={runtime.get('navigation_goal_reached')}, "
            f"navigation_action_status={runtime.get('navigation_action_status')}"
        )
        print(
            f"  exploration_status_raw={runtime.get('exploration_status_raw')}, "
            f"simple_nav_status_raw={runtime.get('navigation_simple_status_raw')}"
        )


def _print_interactive_help() -> None:
    print("Interactive mode: enter one command per line. Input q to quit.")
    print("Examples:")
    print("  mapping")
    print("  navigation")
    print("  goal --x 1.0 --y 0.5 --yaw 1.57")
    print("  wait")
    print("  stop")
    print("  points")
    print("  home")
    print("  status")


def _parse_command_args(parser: argparse.ArgumentParser, argv: list[str]) -> argparse.Namespace:
    return parser.parse_args(argv)


def _usage_error(parser: argparse.ArgumentParser, message: str) -> int:
    print(f"error: {message}")
    print(parser.format_usage().strip())
    return 2


def _execute_action(
    args: argparse.Namespace,
    parser: argparse.ArgumentParser,
    flow: ExplorationFlowController,
) -> int:
    known_actions = {
        "mapping",
        "navigation",
        "goal",
        "goal-name",
        "wait",
        "stop",
        "status",
        "points",
        "delete-point",
    }

    action = args.action
    target_name = args.name

    if not action:
        return _usage_error(parser, "missing action")

    # Convenience: if action is not a known command, treat it as a point name.
    if action not in known_actions:
        target_name = action
        action = "goal-name"

    if action == "points":
        _print_named_points(_load_named_points(args.points_db))
        return 0

    if action == "delete-point":
        if not target_name:
            return _usage_error(parser, "delete-point action requires a point name")
        points = _load_named_points(args.points_db)
        if target_name not in points:
            print(f"named point not found: {target_name}")
            _print_named_points(points)
            return 1
        points.pop(target_name)
        _save_named_points(args.points_db, points)
        print(f"deleted named point: {target_name}")
        _print_named_points(points)
        return 0

    if action == "mapping":
        ok = flow.start_mapping()
        print("mapping started" if ok else "mapping start failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "navigation":
        was_running = flow.navigation.is_running()
        nav_args = {
            "map": _default_map_yaml(),
            "start_hardware": "true",
            "use_rviz": "true" if args.rviz else "false",
        }
        if args.rviz and not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
            print("warning: rviz enabled but DISPLAY/WAYLAND_DISPLAY is empty; GUI window may not appear")
        ok = flow.start_navigation(launch_args=nav_args, force_restart=args.restart)
        if ok:
            if was_running and args.restart:
                print("navigation restarted")
            elif was_running and not args.restart:
                print("navigation already running")
            else:
                print("navigation started")
        else:
            print("navigation start failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "goal":
        if args.x is None or args.y is None:
            return _usage_error(parser, "goal action requires --x and --y")

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
        if not target_name:
            return _usage_error(parser, "goal-name action requires a point name")
        ok = _send_goal_by_name(
            flow=flow,
            name=target_name,
            db_path=args.points_db,
            topic=args.topic,
            wait_subscribers=args.wait_subscribers,
            publish_times=args.publish_times,
        )
        print("goal sent" if ok else "goal send failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "wait":
        ok = flow.wait_navigation(
            action_name=args.nav_action_name,
            timeout_sec=args.wait_timeout,
        )
        print("navigation waiting" if ok else "wait failed")
        _print_status(flow)
        return 0 if ok else 1

    if action == "stop":
        ok = flow.stop_all()
        print("all stopped" if ok else "stop failed")
        _print_status(flow)
        return 0 if ok else 1

    _print_status(flow)
    return 0


def _run_interactive(parser: argparse.ArgumentParser) -> int:
    flow = ExplorationFlowController(log_dir=_default_log_dir())
    _print_interactive_help()

    while True:
        try:
            line = input("controller> ").strip()
        except EOFError:
            print()
            break
        except KeyboardInterrupt:
            print("\nInput q to quit.")
            continue

        if not line:
            continue

        if line.lower() in {"q", "quit", "exit"}:
            break

        if line.lower() in {"help", "h", "?", "-h", "--help"}:
            _print_interactive_help()
            continue

        try:
            cmd_args = _parse_command_args(parser, shlex.split(line))
        except ValueError as exc:
            print(f"parse error: {exc}")
            continue
        except SystemExit:
            # argparse already printed an error/help message.
            continue

        _execute_action(cmd_args, parser, flow)

    flow.shutdown()
    print("bye")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Control mapping/navigation launches")
    parser.add_argument(
        "action",
        nargs="?",
        help=(
            "mapping|navigation|goal|wait|stop|status|points|delete-point|goal-name "
            "or directly provide a point name to navigate"
        ),
    )
    parser.add_argument(
        "name",
        nargs="?",
        help="Point name for goal-name/delete-point actions",
    )
    parser.add_argument("--x", type=float, help="Goal x in map frame")
    parser.add_argument("--y", type=float, help="Goal y in map frame")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw (radians)")
    parser.add_argument("--frame-id", default="map", help="Goal frame id")
    parser.add_argument(
        "--topic",
        default="/goal_pose",
        help="Goal topic consumed by Nav2 bt_navigator",
    )
    parser.add_argument(
        "--wait-subscribers",
        type=float,
        default=15.0,
        help="Wait timeout (sec) for goal topic subscribers",
    )
    parser.add_argument(
        "--publish-times",
        type=int,
        default=3,
        help="How many times to publish the same goal",
    )
    parser.add_argument(
        "--points-db",
        default=_default_points_db(),
        help="Named points JSON database path",
    )
    parser.add_argument(
        "--nav-action-name",
        default="navigate_to_pose",
        help="NavigateToPose action name used by wait/cancel",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=3.0,
        help="Wait timeout (sec) for wait action cancel request",
    )
    parser.add_argument(
        "--rviz",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable RViz when launching navigation",
    )
    parser.add_argument(
        "--restart",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Restart navigation process when action is navigation",
    )

    # No command-line arguments: start interactive shell mode.
    if len(sys.argv) == 1:
        return _run_interactive(parser)

    args = parser.parse_args()
    flow = ExplorationFlowController(log_dir=_default_log_dir())
    return _execute_action(args, parser, flow)


if __name__ == "__main__":
    raise SystemExit(main())
