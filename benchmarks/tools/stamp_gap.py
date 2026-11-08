"""Gap between header.stamp (set by ros2 topic pub) and the rmw publication
timestamp carried in the attachment, on topic_based_joint_commands."""

import statistics
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

gaps_us = []
loop_us = []


def on_msg(msg, info):
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    gaps_us.append((info["source_timestamp"] - stamp_ns) / 1000)
    loop_us.append((info["received_timestamp"] - info["source_timestamp"]) / 1000)


def summary(name, values):
    values = sorted(values)
    p = lambda q: values[min(len(values) - 1, int(q * len(values)))]
    print(f"{name}: n={len(values)} min={values[0]:.0f} p10={p(0.1):.0f} median={p(0.5):.0f} "
          f"mean={statistics.fmean(values):.0f} p90={p(0.9):.0f} max={values[-1]:.0f} us")


rclpy.init()
node = rclpy.create_node("stamp_gap")
node.create_subscription(JointState, "topic_based_joint_commands", on_msg,
                         QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT))
end = time.monotonic() + float(sys.argv[1] if len(sys.argv) > 1 else 10)
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
summary("source_timestamp - header.stamp", gaps_us)
summary("received - source (host loopback)", loop_us)
node.destroy_node()
rclpy.shutdown()
