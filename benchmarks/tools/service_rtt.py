"""Service round trip and late joining on a transient local topic, for
../microros_rp2040_basic and ../picoros_rp2040_basic. Both serve pico/ping, a
std_srvs/Trigger answered at once, and publish pico/boot once per session,
reliable, transient local, depth 1.

Phase late_join: a new subscription to pico/boot, reliable and transient local,
waits for the boot message. The board published it when its session opened,
before this script started, so only the publisher's cache can deliver it. The
delay runs from the creation of the subscription to the message. Repeated
--attempts times.

Phases idle and joint: --calls calls to pico/ping, each sent 10 to 20 ms after
the previous answer, at random, so that the calls meet every phase of the
board's 10 ms cadence. The round trip runs from call_async() to the moment the
future is done, on the host clock alone: no clock offset enters it. As in
measure.py, the odometry and stats are subscribed to; phase joint adds
JointState at 99 Hz.

The FreeRTOS heap free and its lowest ever, and from the micro-ROS firmware
the unused stack of each task, are read from pico/stats at the start and after
each phase: run on a board just flashed, the start is the state after boot and
the end the low marks of the whole run.

With --host-reference peer|client, the same runs against a stand-in for the
board started on the host, which serves host_ref/ping and publishes
host_ref/boot, to tell the share of the host and of the router; the modes are
those of host_reference.py.

Usage: service_rtt.py <label> [--calls 1000] [--attempts 5] [--host-reference peer|client]
Results are printed and written to <label>_<date>.txt in measure.RESULTS.
"""

import argparse
import os
import random
import signal
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from measure import RESULTS, SETTLE_S, Measure, Phase, describe, memory_line, report

CLIENT_OVERRIDE = 'mode="client"'
JOINT_RATE_HZ = 99.0
GAP_S = (0.010, 0.020)
CALL_TIMEOUT_S = 2.0
LATE_JOIN_TIMEOUT_S = 5.0
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def spin_until(node, done, timeout_s):
    end = time.monotonic() + timeout_s
    while not done() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.001)
    return done()


def spin_for(node, seconds):
    spin_until(node, lambda: False, seconds)


def late_join(node, topic, attempts):
    """Delay of each attempt in ms, None when nothing came, and the last message."""
    delays, text = [], None
    for _ in range(attempts):
        received = []
        start = time.perf_counter()
        subscription = node.create_subscription(
            String, topic, lambda msg: received.append((time.perf_counter(), msg.data)), LATCHED)
        if spin_until(node, lambda: received, LATE_JOIN_TIMEOUT_S):
            delays.append((received[0][0] - start) * 1000)
            text = received[0][1]
        else:
            delays.append(None)
        node.destroy_subscription(subscription)
        spin_for(node, 0.5)
    return delays, text


def round_trips(node, client, calls):
    """Round trips in us, and the number of calls left unanswered."""
    rtts_us, lost = [], 0
    for _ in range(calls):
        start = time.perf_counter_ns()
        future = client.call_async(Trigger.Request())
        if spin_until(node, future.done, CALL_TIMEOUT_S):
            rtts_us.append((time.perf_counter_ns() - start) / 1000)
        else:
            lost += 1
            client.remove_pending_request(future)
        spin_for(node, random.uniform(*GAP_S))
    return rtts_us, lost


def late_join_lines(topic, delays, text):
    got = [d for d in delays if d is not None]
    lines = [f"[late_join] {topic}: received {len(got)}/{len(delays)}"]
    if got:
        lines.append("  delay from subscription to message ms: " + " ".join(f"{d:.1f}" for d in got))
        lines.append(f"  message: {text}")
    return lines


def round_trip_lines(rtts_us, lost):
    if not rtts_us:
        return [f"  no answer, lost={lost}"]
    return [f"  round trip {describe(rtts_us, 'us')} lost={lost}"]


class BoardStandIn:
    def __init__(self, node):
        node.create_service(Trigger, "host_ref/ping", self.on_ping)
        self.boot_pub = node.create_publisher(String, "host_ref/boot", LATCHED)
        self.boot_pub.publish(String(data="host reference"))

    def on_ping(self, request, response):
        response.success = True
        return response


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


def run_host_reference(args):
    rclpy.init()
    node = rclpy.create_node("host_ref_service")
    client = node.create_client(Trigger, "host_ref/ping")
    if not client.wait_for_service(timeout_sec=30):
        print("no host_ref/ping within 30 s: is the router running?", file=sys.stderr)
        return 1
    started = datetime.now()
    spin_for(node, SETTLE_S)
    delays, text = late_join(node, "host_ref/boot", args.attempts)
    rtts_us, lost = round_trips(node, client, args.calls)
    node.destroy_node()
    rclpy.shutdown()

    lines = [f"{args.label} {started.isoformat(timespec='seconds')} service round trip and late joining, "
             f"host reference mode={args.host_reference}"]
    lines += late_join_lines("host_ref/boot", delays, text)
    lines.append("[idle]")
    lines += round_trip_lines(rtts_us, lost)
    return write(args.label, started, lines)


def run_board_measure(args):
    rclpy.init()
    node = rclpy.create_node("benchmark_service")
    measure = Measure(node, JOINT_RATE_HZ)
    client = node.create_client(Trigger, "pico/ping")
    deadline = time.monotonic() + 30
    while not measure.stats_seen and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not measure.stats_seen or not client.wait_for_service(timeout_sec=10):
        print("no pico/stats or no pico/ping: is the router running and a benchmark firmware flashed?",
              file=sys.stderr)
        return 1

    started = datetime.now()
    memory = [("start", measure.next_memory())]
    delays, text = late_join(node, "pico/boot", args.attempts)
    memory.append(("after late_join", measure.next_memory()))
    results = []
    for phase, publishing in ((Phase("idle"), False), (Phase("joint"), True)):
        measure.publishing = publishing
        spin_for(node, SETTLE_S)
        measure.current = phase
        results.append((phase, round_trips(node, client, args.calls)))
        measure.current = None
        memory.append((f"after {phase.name}", measure.next_memory()))
    measure.publishing = False
    node.destroy_node()
    rclpy.shutdown()

    clock = f", board clock {measure.clock_mhz} MHz" if measure.clock_mhz else ""
    lines = [f"{args.label} {started.isoformat(timespec='seconds')} service round trip and late joining{clock}"]
    lines += late_join_lines("pico/boot", delays, text)
    for phase, (rtts_us, lost) in results:
        lines += report(phase)
        lines += round_trip_lines(rtts_us, lost)
    lines.append("[memory] bytes, from the first stats after each step")
    lines += [memory_line(when, values) for when, values in memory]
    return write(args.label, started, lines)


def write(label, started, lines):
    text = "\n".join(lines)
    print(text)
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / f"{label}_{started:%Y%m%d_%H%M%S}.txt").write_text(text + "\n")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("label")
    parser.add_argument("--calls", type=int, default=1000)
    parser.add_argument("--attempts", type=int, default=5)
    parser.add_argument("--host-reference", choices=["peer", "client"])
    parser.add_argument("--role", choices=["measure", "board"], default="measure", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.host_reference is None:
        return run_board_measure(args)
    if args.host_reference == "client":
        os.environ["ZENOH_CONFIG_OVERRIDE"] = CLIENT_OVERRIDE
    if args.role == "board":
        return run_board()

    stand_in = subprocess.Popen([sys.executable, __file__, args.label, "--host-reference", args.host_reference,
                                 "--role", "board"])
    try:
        return run_host_reference(args)
    finally:
        stand_in.send_signal(signal.SIGINT)
        try:
            stand_in.wait(timeout=10)
        except subprocess.TimeoutExpired:
            stand_in.kill()


if __name__ == "__main__":
    sys.exit(main())
