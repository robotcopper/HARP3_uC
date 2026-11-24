#!/usr/bin/env python3
"""Publishes a synthetic LaserScan on `scan`, to try the ring without a lidar.

    source /opt/ros/lyrical/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    tools/fake_scan.py --angle 0 --range 0.15          # red at +x, green elsewhere
    tools/fake_scan.py --angle 90 --angle -90          # two obstacles, 0.15 m
    tools/fake_scan.py --sweep 20                      # an obstacle turning at 20 deg/s
    tools/fake_scan.py --missed 0.3                    # 30 % of the beams report 0

The scan has the shape of the YDLIDAR Tmini Plus driver's: -pi to pi, 500 beams, 8 Hz,
best effort, 0 for a point it missed. Angles are in base_link, anticlockwise from +x.
"""

import argparse
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class FakeScan(Node):
    def __init__(self, args):
        super().__init__('fake_scan')
        self.args = args
        self.beams = args.beams
        self.increment = 2.0 * math.pi / self.beams
        self.publisher = self.create_publisher(LaserScan, args.topic, qos_profile_sensor_data)
        self.start = self.get_clock().now()
        self.create_timer(1.0 / args.rate, self.publish)

    def obstacle_angles(self):
        if self.args.sweep:
            elapsed = (self.get_clock().now() - self.start).nanoseconds * 1e-9
            return [math.radians(self.args.sweep * elapsed)]
        return [math.radians(a) for a in self.args.angle]

    def publish(self):
        args = self.args
        ranges = [args.background] * self.beams
        half_width = math.radians(args.width) / 2.0
        for angle in self.obstacle_angles():
            for k in range(self.beams):
                beam = -math.pi + k * self.increment
                if abs(math.atan2(math.sin(beam - angle), math.cos(beam - angle))) <= half_width:
                    ranges[k] = args.range
        for k in range(self.beams):
            if random.random() < args.missed:
                ranges[k] = 0.0
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = args.frame
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = self.increment
        msg.scan_time = 1.0 / args.rate
        msg.time_increment = msg.scan_time / self.beams
        msg.range_min = 0.08
        msg.range_max = 12.0
        msg.ranges = ranges
        msg.intensities = [47.0] * self.beams if args.intensities else []
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--topic', default='scan')
    parser.add_argument('--frame', default='laser_frame')
    parser.add_argument('--angle', type=float, action='append', default=[], help='degrees, repeatable')
    parser.add_argument('--range', type=float, default=0.15, help='obstacle distance, m')
    parser.add_argument('--width', type=float, default=10.0, help='obstacle width, degrees')
    parser.add_argument('--background', type=float, default=2.0, help='distance elsewhere, m')
    parser.add_argument('--sweep', type=float, default=0.0, help='turn one obstacle at this many deg/s')
    parser.add_argument('--missed', type=float, default=0.0, help='fraction of beams reporting 0')
    parser.add_argument('--beams', type=int, default=500)
    parser.add_argument('--rate', type=float, default=8.0)
    parser.add_argument('--no-intensities', dest='intensities', action='store_false',
                        help='leave intensities empty, a message of 2 kB instead of 4 kB')
    args = parser.parse_args()
    rclpy.init()
    node = FakeScan(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
