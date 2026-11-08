"""Measurement protocol, the same for ../microros_rp2040_basic and
../picoros_rp2040_basic, which publish the same topics and stats.

Phase no_joint: subscribes to pico/odom, as a consumer such as the EKF would,
and to pico/stats. zenoh-pico does not send what nobody subscribes to, so the
odometry is only paid for when someone listens.
Phase joint: the same, plus JointState on topic_based_joint_commands at 100 Hz,
stamped at publication as "ros2 topic pub ... header: auto" does. The board
publishes its odometry every 10 ms of its own clock, and a JointState arriving
meanwhile waits for it; at 100 Hz that phase barely moves within a run but
changes between runs. --rate 99 sweeps it over every 10 ms period once per
second, so that each stats period covers all of them.
Phase ping: the same traffic, plus 100 pings to the board.

The uplink delay is the reception time of pico/odom minus its header.stamp, the
downlink delay the board's 1 s average from pico/stats. Each moves with the
offset between the board clock and the host clock; their sum does not, as long
as the offset holds still. It drifts between SNTP updates, so the sum is also
taken per stats period: the downlink average plus the mean uplink delay of the
odometry received since the previous stats.

With --profile, the micro-ROS firmware samples its program counter during phase
joint, via the pico/profiler service, and dumps the samples on pico/profile
afterwards; they are written to <label>_<date>_profile.txt in RESULTS, for
profile_report.py.

Usage: measure.py <label> [--seconds 30] [--rate 100] [--profile]
Results are printed and written to <label>_<date>.txt in RESULTS.
"""

import argparse
import re
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

BOARD = "192.168.7.1"
RESULTS = Path(__file__).resolve().parents[2] / "docs" / "picoros_vs_microros" / "results"
PROFILER = "pico/profiler"
SETTLE_S = 2.0
PING_COUNT = 100
PING_INTERVAL_S = 0.2


class Phase:
    def __init__(self, name):
        self.name = name
        self.up_us = []
        self.up_since_stats_us = []
        self.down_avg_us = []
        self.down_spread_us = []
        self.period_sums_us = []
        self.joint_n = []
        self.idle = []
        self.cpu = {}
        self.xip_hit = []
        self.xip_acc = []
        self.waits = {}
        self.split = {}
        self.ambiguous = []
        self.lookup = []
        self.ping = None


class Measure:
    def __init__(self, node, rate_hz, joint_topic="topic_based_joint_commands", odom_topic="pico/odom",
                 stats_topic="pico/stats"):
        self.node = node
        self.current = None
        self.publishing = False
        self.stats_seen = False
        self.stats_count = 0
        self.memory = None
        self.clock_mhz = None
        self.profile_lines = []
        self.profile_done = False
        self.profile_client = node.create_client(SetBool, PROFILER)
        node.create_subscription(String, "pico/profile", self.on_profile, 1000)
        best_effort = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Odometry, odom_topic, self.on_odom, best_effort)
        node.create_subscription(String, stats_topic, self.on_stats, best_effort)
        self.joint_pub = node.create_publisher(JointState, joint_topic, 10)
        self.joint = JointState(name=["w1", "w2", "w3"], position=[0.0] * 3, velocity=[1.0, 2.0, 3.0],
                                effort=[0.0] * 3)
        node.create_timer(1.0 / rate_hz, self.on_joint_timer)

    def on_joint_timer(self):
        if self.publishing:
            self.joint.header.stamp = self.node.get_clock().now().to_msg()
            self.joint_pub.publish(self.joint)

    def on_odom(self, msg, info):
        if self.current is not None:
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            up_us = (info["received_timestamp"] - stamp_ns) / 1000
            self.current.up_us.append(up_us)
            self.current.up_since_stats_us.append(up_us)

    def on_profile(self, msg):
        self.profile_lines.append(msg.data)
        if msg.data.startswith("profile end"):
            self.profile_done = True

    def set_profiling(self, enabled):
        future = self.profile_client.call_async(SetBool.Request(data=enabled))
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)
        if not future.done():
            raise RuntimeError(f"{PROFILER}({enabled}) got no answer within 5 s")
        return future.result().message

    def collect_profile(self, seconds=30):
        end = time.monotonic() + seconds
        while not self.profile_done and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        if not self.profile_done:
            raise RuntimeError(f"no 'profile end' on pico/profile within {seconds} s")

    def on_stats(self, msg):
        self.stats_seen = True
        self.stats_count += 1
        text = msg.data
        match = re.search(r"clk_mhz=(\d+)", text)
        if match:
            self.clock_mhz = int(match.group(1))
        self.memory = parse_memory(text)
        phase = self.current
        if phase is None:
            return
        match = re.search(r"joint n=(\d+)", text)
        if match:
            phase.joint_n.append(int(match.group(1)))
        up_since_stats = phase.up_since_stats_us
        phase.up_since_stats_us = []
        match = re.search(r"dl_us min=(-?\d+) avg=(-?\d+) max=(-?\d+)", text)
        if match and " n=0 " not in text:
            phase.down_avg_us.append(int(match.group(2)))
            phase.down_spread_us.append(int(match.group(3)) - int(match.group(1)))
            if len(up_since_stats) >= 50:
                phase.period_sums_us.append(int(match.group(2)) + statistics.fmean(up_since_stats))
        match = re.search(r"idle=(\d+)%", text)
        if match:
            phase.idle.append(int(match.group(1)))
        match = re.search(r"\| cpu (.*?) \|", text)
        if match:
            for name, share in re.findall(r"([A-Za-z][A-Za-z ]*?)=(\d+\.\d)", match.group(1)):
                phase.cpu.setdefault(name.strip(), []).append(float(share))
        match = re.search(r"xip hit=(\d+\.\d)% acc=(\d+)", text)
        if match:
            phase.xip_hit.append(float(match.group(1)))
            phase.xip_acc.append(int(match.group(2)))
        match = re.search(r"\| wait (.*?) \|", text)
        if match:
            for name, count in re.findall(r"(\w+)=(\d+)", match.group(1)):
                phase.waits.setdefault(name, []).append(int(count))
        match = re.search(r"\| split (.*?) \|", text)
        if match:
            for name, n, low, avg, high in re.findall(r"(\w+) n=(\d+) min=(-?\d+) avg=(-?\d+) max=(-?\d+)",
                                                      match.group(1)):
                if int(n) > 0:
                    phase.split.setdefault(name, []).append((int(low), int(avg), int(high)))
            ambiguous = re.search(r"ambiguous=(\d+)", match.group(1))
            if ambiguous:
                phase.ambiguous.append(int(ambiguous.group(1)))
        match = re.search(r"\| lookup n=(\d+) sum_us=(\d+) max_us=(\d+) \|", text)
        if match:
            phase.lookup.append(tuple(int(g) for g in match.groups()))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def next_memory(self, timeout_s=3.0):
        """The memory of the first stats received after this call, None if none came."""
        count = self.stats_count
        end = time.monotonic() + timeout_s
        while self.stats_count == count and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)
        return self.memory if self.stats_count != count else None

    def run(self, phase, seconds, publishing, ping, profile=False):
        """Records after SETTLE_S, so that the 1 s stats straddling the switch are dropped."""
        self.publishing = publishing
        self.current = None
        self.spin_for(SETTLE_S)
        self.current = phase
        if profile:
            print(f"{PROFILER}(true): {self.set_profiling(True)}", file=sys.stderr)
        process = None
        if ping:
            process = subprocess.Popen(["ping", "-q", "-c", str(PING_COUNT), "-i", str(PING_INTERVAL_S), BOARD],
                                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            seconds = max(seconds, PING_COUNT * PING_INTERVAL_S + 1)
        self.spin_for(seconds)
        self.current = None
        if profile:
            print(f"{PROFILER}(false): {self.set_profiling(False)}", file=sys.stderr)
            self.collect_profile()
        if process is not None:
            output = process.communicate(timeout=10)[0]
            match = re.search(r"(\d+)% packet loss.*?= ([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms", output, re.S)
            phase.ping = match.groups() if match else output.strip()


def parse_memory(text):
    """In bytes: the FreeRTOS heap free and its lowest ever, and from the micro-ROS
    firmware only, newlib's malloc() bytes in use and arena, and the unused stack
    of each task."""
    memory = {}
    match = re.search(r"heap=(\d+) min=(\d+)", text)
    if match:
        memory["heap"] = int(match.group(1))
        memory["heap_min"] = int(match.group(2))
    match = re.search(r"libc=(\d+)/(\d+)", text)
    if match:
        memory["libc_used"] = int(match.group(1))
        memory["libc_arena"] = int(match.group(2))
    match = re.search(r"\| stack free (.*?)(?: \||$)", text)
    if match:
        memory["stacks"] = {name.strip(): int(free)
                            for name, free in re.findall(r"([A-Za-z][A-Za-z ]*?)=(\d+)", match.group(1))}
    return memory


def memory_line(when, memory):
    if not memory:
        return f"  {when}: no stats"
    line = f"  {when}: heap free={memory['heap']} min={memory['heap_min']}"
    if "libc_used" in memory:
        line += f" | malloc used={memory['libc_used']} arena={memory['libc_arena']}"
    if "stacks" in memory:
        line += " | stack free " + " ".join(f"{name}={free}" for name, free in memory["stacks"].items())
    return line


def describe(values, unit):
    values = sorted(values)
    p = lambda q: values[min(len(values) - 1, int(q * len(values)))]
    return (f"n={len(values)} min={values[0]:.0f} p10={p(0.1):.0f} median={p(0.5):.0f} "
            f"mean={statistics.fmean(values):.0f} p90={p(0.9):.0f} max={values[-1]:.0f} "
            f"sd={statistics.pstdev(values):.0f} {unit}")


def report(phase):
    lines = [f"[{phase.name}]"]
    if phase.idle:
        lines.append(f"  idle mean={statistics.fmean(phase.idle):.1f} % min={min(phase.idle)} max={max(phase.idle)}")
    if phase.cpu:
        lines.append("  cpu mean %: " + " ".join(f"{n}={statistics.fmean(v):.1f}" for n, v in phase.cpu.items()))
    if phase.xip_hit:
        misses = statistics.fmean(a * (100 - h) / 100 for a, h in zip(phase.xip_acc, phase.xip_hit))
        lines.append(f"  xip hit mean={statistics.fmean(phase.xip_hit):.1f} % "
                     f"acc mean={statistics.fmean(phase.xip_acc):.0f}/s misses mean={misses:.0f}/s")
    if phase.waits:
        lines.append("  rmw_wait per second: " +
                     " ".join(f"{n}={statistics.fmean(v):.1f}" for n, v in phase.waits.items()))
    if phase.joint_n:
        lines.append(f"  joint received per second: mean={statistics.fmean(phase.joint_n):.1f} "
                     f"min={min(phase.joint_n)} max={max(phase.joint_n)}")
    if phase.up_us:
        lines.append("  uplink " + describe(phase.up_us, "us"))
    if phase.down_avg_us:
        lines.append("  downlink 1 s averages " + describe(phase.down_avg_us, "us"))
        lines.append("  downlink max - min per 1 s " + describe(phase.down_spread_us, "us"))
    for name, windows in phase.split.items():
        lines.append(f"  {name} per 1 s: avg mean={statistics.fmean(w[1] for w in windows):.0f} "
                     f"min={min(w[0] for w in windows)} max={max(w[2] for w in windows)}")
        lines.append(f"  {name} max - min per 1 s " + describe([w[2] - w[0] for w in windows], "us"))
    if phase.ambiguous:
        lines.append(f"  split ambiguous per second: mean={statistics.fmean(phase.ambiguous):.1f}")
    samples = sum(n for n, _, _ in phase.lookup)
    if samples:
        lines.append(f"  subscription lookup per sample: mean={sum(s for _, s, _ in phase.lookup) / samples:.1f} us "
                     f"max={max(m for _, _, m in phase.lookup)} us, samples per second "
                     f"mean={samples / len(phase.lookup):.1f}")
    if phase.up_us and phase.down_avg_us:
        total = statistics.fmean(phase.up_us) + statistics.fmean(phase.down_avg_us)
        lines.append(f"  downlink + uplink mean={total:.0f} us")
    if phase.period_sums_us:
        lines.append("  downlink + uplink per stats period " + describe(phase.period_sums_us, "us"))
    if isinstance(phase.ping, tuple):
        loss, low, avg, high, mdev = phase.ping
        lines.append(f"  ping loss={loss} % min={low} avg={avg} max={high} mdev={mdev} ms")
    elif phase.ping:
        lines.append(f"  ping failed: {phase.ping}")
    return lines


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("label")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--rate", type=float, default=100.0, help="JointState rate, Hz")
    parser.add_argument("--profile", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("benchmark_measure")
    measure = Measure(node, args.rate)
    deadline = time.monotonic() + 30
    while not measure.stats_seen and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not measure.stats_seen:
        print("no pico/stats within 30 s: is the router running and the board flashed?", file=sys.stderr)
        return 1
    if args.profile and not measure.profile_client.wait_for_service(timeout_sec=10):
        print(f"no {PROFILER} service: only the micro-ROS firmware has the profiler", file=sys.stderr)
        return 1

    started = datetime.now()
    phases = [Phase("no_joint"), Phase("joint"), Phase("joint_ping")]
    measure.run(phases[0], args.seconds, publishing=False, ping=False)
    measure.run(phases[1], args.seconds, publishing=True, ping=False, profile=args.profile)
    measure.run(phases[2], 0, publishing=True, ping=True)
    measure.publishing = False
    node.destroy_node()
    rclpy.shutdown()

    results = RESULTS
    results.mkdir(parents=True, exist_ok=True)
    stem = f"{args.label}_{started:%Y%m%d_%H%M%S}"
    clock = f", board clock {measure.clock_mhz} MHz" if measure.clock_mhz else ""
    lines = [f"{args.label} {started.isoformat(timespec='seconds')} JointState at {args.rate:g} Hz{clock}"]
    if any(n > 0 for n in phases[0].joint_n):
        lines.append("WARNING: the board received JointState in phase no_joint: another publisher is running")
    for phase in phases:
        lines += report(phase)
    if args.profile:
        profile_path = results / f"{stem}_profile.txt"
        profile_path.write_text("\n".join(measure.profile_lines) + "\n")
        lines.append(f"profile of phase joint: {profile_path}")
    text = "\n".join(lines)
    print(text)
    (results / f"{stem}.txt").write_text(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
