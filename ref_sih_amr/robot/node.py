"""Standalone robot process for the real UDP P2P coordination path.

Example (three local robot processes):

python ref_sih_amr/robot/node.py --robot-id robot-0 --bind 127.0.0.1:19001 \
  --peers robot-1=127.0.0.1:19002,robot-2=127.0.0.1:19003 \
  --start 1,1 --pickup 4,1 --dropoff 4,5

Each process owns its LocalTaskManager and reservation table. The only
inter-robot state exchange is IntentMessage over UDP.
"""

from __future__ import annotations

import argparse
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from comms.channel import UdpPeerChannel
from config import load_map
from models import RobotState, Task, TaskStatus
from robot.planner import AStarPlanner
from robot.task_manager import LocalTaskManager
from experiments.runner import SCENARIOS


def parse_endpoint(value: str):
    host, port = value.rsplit(":", 1)
    return host, int(port)


def parse_peers(value: str):
    peers = {}
    if not value:
        return peers
    for item in value.split(","):
        robot_id, endpoint = item.split("=", 1)
        peers[robot_id] = parse_endpoint(endpoint)
    return peers


def parse_cell(value: str):
    x, y = value.split(",", 1)
    return int(x), int(y)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--robot-id", required=True)
    p.add_argument("--bind", required=True, help="host:port")
    p.add_argument("--peers", default="", help="robot-id=host:port,...")
    p.add_argument("--start", required=True)
    p.add_argument("--pickup", required=True)
    p.add_argument("--dropoff", required=True)
    p.add_argument("--scenario", default="S1_Normal")
    p.add_argument("--ticks", type=int, default=500)
    p.add_argument("--period", type=float, default=0.2)
    p.add_argument("--start-delay", type=float, default=0.0, help="seconds to wait before the first simulation tick")
    args = p.parse_args()

    costmap = load_map(SCENARIOS[args.scenario])
    channel = UdpPeerChannel(
        args.robot_id,
        parse_endpoint(args.bind),
        parse_peers(args.peers),
    )

    state = RobotState(
        robot_id=args.robot_id,
        timestamp=0.0,
        position=tuple(float(v) for v in parse_cell(args.start)),
        heading=0.0,
        velocity=0.0,
        battery=100.0,
        current_task_id=None,
        task_priority=1,
    )
    manager = LocalTaskManager(
        state,
        AStarPlanner(),
        channel,
        costmap,
        strategy="P2P",
    )
    task = Task(
        task_id=f"{args.robot_id}-TASK-1",
        pickup_cell=parse_cell(args.pickup),
        dropoff_cell=parse_cell(args.dropoff),
        priority=1,
        status=TaskStatus.QUEUED,
        source="MANUAL",
    )
    manager.assign_task(task)

    try:
        if args.start_delay > 0:
            time.sleep(args.start_delay)

        for tick in range(1, args.ticks + 1):
            manager.tick(float(tick))
            print(
                f"{args.robot_id} tick={tick} "
                f"pos={manager.state.position} status={manager.state.status.value} "
                f"task={manager.state.current_task_id}",
                flush=True,
            )
            if task.status == TaskStatus.COMPLETED:
                break
            time.sleep(args.period)
    finally:
        channel.close()


if __name__ == "__main__":
    main()
