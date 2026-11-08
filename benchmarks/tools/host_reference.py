"""Reference for measure.py without the board: the same JointState and odometry
exchange between two processes on the host, to tell the share of the host and
of the router in the board's downlink + uplink.

This script starts a stand-in for the board, which subscribes to
host_ref/joint best effort as the board does, publishes host_ref/odom every
10 ms stamped just before publication, and publishes host_ref/stats every
second in the format of the board's stats: the delay up to the rmw
(received_timestamp, as before_rmw_us) and from there up to the callback (as
in_executor_us). Both processes share one clock, so each direction holds on
its own, unlike with the board.

rmw_zenoh_cpp sessions are peers by default: the router only introduces them,
then they exchange data directly. The board is a client of the router and all
its traffic crosses it. --mode client sets ZENOH_CONFIG_OVERRIDE to
mode="client" in both processes, so that their data crosses the router as the
board's does. The router and its configuration are left as they are.

Usage: host_reference.py <label> [--mode peer|client] [--seconds 30] [--rate 99]
Results are printed and written to <label>_<date>.txt in measure.RESULTS.
"""

import argparse
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from measure import RESULTS, Measure, Phase, report

TOPICS = {"joint_topic": "host_ref/joint", "odom_topic": "host_ref/odom", "stats_topic": "host_ref/stats"}
ODOM_PERIOD_S = 0.01
CLIENT_OVERRIDE = 'mode="client"'


def span(name, values, count=True):
    prefix = f"{name} n={len(values)}" if count else name
    if not values:
        return f"{prefix} min=0 avg=0 max=0"
    return (f"{prefix} min={round(min(values))} avg={round(statistics.fmean(values))} "
            f"max={round(max(values))}")


class BoardStandIn:
    def __init__(self, node):
        self.node = node
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(JointState, TOPICS["joint_topic"], self.on_joint, best_effort)
        self.odom_pub = node.create_publisher(Odometry, TOPICS["odom_topic"], best_effort)
        self.stats_pub = node.create_publisher(String, TOPICS["stats_topic"], best_effort)
        self.odom = Odometry()
        self.odom.header.frame_id = "odom"
        self.odom.child_frame_id = "base_footprint"
        self.odom.pose.pose.orientation.w = 1.0
        self.total_us, self.before_rmw_us, self.in_executor_us = [], [], []
        node.create_timer(ODOM_PERIOD_S, self.on_odom_timer)
        node.create_timer(1.0, self.on_stats_timer)

    def on_joint(self, msg, info):
        now_ns = self.node.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        received_ns = info["received_timestamp"]
        self.total_us.append((now_ns - stamp_ns) / 1000)
        self.before_rmw_us.append((received_ns - stamp_ns) / 1000)
        self.in_executor_us.append((now_ns - received_ns) / 1000)

    def on_odom_timer(self):
        self.odom.header.stamp = self.node.get_clock().now().to_msg()
        self.odom_pub.publish(self.odom)

    def on_stats_timer(self):
        total, before, inside = self.total_us, self.before_rmw_us, self.in_executor_us
        self.total_us, self.before_rmw_us, self.in_executor_us = [], [], []
        text = (f"joint n={len(total)} {span('dl_us', total, count=False)} | split {span('before_rmw_us', before)} "
                f"{span('in_executor_us', inside)} ambiguous=0 | host reference")
        self.stats_pub.publish(String(data=text))


def run_board():
    rclpy.init()
    node = rclpy.create_node("host_ref_board")
    BoardStandIn(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def run_measure(args):
    rclpy.init()
    node = rclpy.create_node("host_ref_measure")
    measure = Measure(node, args.rate, **TOPICS)
    deadline = time.monotonic() + 30
    while not measure.stats_seen and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not measure.stats_seen:
        print("no host_ref/stats within 30 s: is the router running?", file=sys.stderr)
        return 1

    started = datetime.now()
    phases = [Phase("no_joint"), Phase("joint")]
    measure.run(phases[0], args.seconds, publishing=False, ping=False)
    measure.run(phases[1], args.seconds, publishing=True, ping=False)
    measure.publishing = False
    node.destroy_node()
    rclpy.shutdown()

    results = RESULTS
    results.mkdir(parents=True, exist_ok=True)
    lines = [f"{args.label} {started.isoformat(timespec='seconds')} host reference mode={args.mode} "
             f"JointState at {args.rate:g} Hz"]
    for phase in phases:
        lines += report(phase)
        before = phase.split.get("before_rmw_us")
        if before and phase.up_us:
            total = statistics.fmean(w[1] for w in before) + statistics.fmean(phase.up_us)
            lines.append(f"  before_rmw + uplink mean={total:.0f} us")
    text = "\n".join(lines)
    print(text)
    (results / f"{args.label}_{started:%Y%m%d_%H%M%S}.txt").write_text(text + "\n")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("label")
    parser.add_argument("--mode", choices=["peer", "client"], default="peer")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=99.0, help="JointState rate, Hz")
    parser.add_argument("--role", choices=["measure", "board"], default="measure", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.mode == "client":
        os.environ["ZENOH_CONFIG_OVERRIDE"] = CLIENT_OVERRIDE
    if args.role == "board":
        return run_board()

    stand_in = subprocess.Popen([sys.executable, __file__, args.label, "--mode", args.mode, "--role", "board"])
    try:
        return run_measure(args)
    finally:
        stand_in.send_signal(signal.SIGINT)
        try:
            stand_in.wait(timeout=10)
        except subprocess.TimeoutExpired:
            stand_in.kill()


if __name__ == "__main__":
    sys.exit(main())
