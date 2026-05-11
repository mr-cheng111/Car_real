#!/usr/bin/env python3
"""Save Cartographer SLAM state to a pbstream file."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import rclpy
from cartographer_ros_msgs.srv import WriteState


def workspace_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "src" / "car_nav2").exists():
            return parent
    return Path.cwd()


def resolve_workspace_path(path: str) -> str:
    expanded = Path(os.path.expanduser(path))
    if expanded.is_absolute():
        return str(expanded)
    return str(workspace_root() / expanded)


def main() -> int:
    parser = argparse.ArgumentParser(description="Save Cartographer state")
    parser.add_argument(
        "--output",
        default="src/car_nav2/maps/cartographer/latest.pbstream",
        help="Output .pbstream path",
    )
    parser.add_argument("--service", default="/write_state")
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument(
        "--include-unfinished-submaps",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    args = parser.parse_args()

    output = resolve_workspace_path(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)

    rclpy.init(args=None)
    node = rclpy.create_node("save_cartographer_state")
    try:
        client = node.create_client(WriteState, args.service)
        if not client.wait_for_service(timeout_sec=max(0.0, args.timeout)):
            node.get_logger().error(f"service not available: {args.service}")
            return 1

        request = WriteState.Request()
        request.filename = output
        request.include_unfinished_submaps = bool(args.include_unfinished_submaps)

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, args.timeout))
        if not future.done():
            node.get_logger().error(f"timed out writing Cartographer state: {output}")
            return 1

        response = future.result()
        if response is None:
            node.get_logger().error("write_state returned no response")
            return 1

        status = response.status
        if int(status.code) != 0:
            node.get_logger().error(f"write_state failed ({status.code}): {status.message}")
            return 1

        node.get_logger().info(f"saved Cartographer state: {output}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
