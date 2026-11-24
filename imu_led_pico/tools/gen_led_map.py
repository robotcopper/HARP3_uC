#!/usr/bin/env python3
"""Writes include/led_map.h: the sector of the lidar's field of view that each LED shows.

    tools/gen_led_map.py [--check]

The strip follows the outline of the robot's base plate, and the lidar sits at its
centre. Each LED is projected onto the lidar: the LED shows the nearest obstacle
within the angles its own stretch of strip spans, seen from the lidar.

The geometry below comes from "base_link+otos Drawing.svg" (CAD export, 300 dpi, so
1 unit is 25.4 / 300 mm), measured in millimetres on the drawing. Only the 18 vertices
of the outline, the centre and the start are kept: the SVG itself is 1 MB.

The drawing is a top view whose x runs to the right and y up the page, which are ROS's
x and y: the lobe on the right, one of the three wheels, is +x. SVG y runs down the
page, so it is flipped.

--check   fail if include/led_map.h is not what this script would write.
"""

import argparse
import math
import sys
from pathlib import Path

# Outline of the plate, in the order of the drawing, in mm, SVG coordinates (y down).
OUTLINE = [
    (170.19, 291.31), (215.41, 212.99), (230.41, 204.33), (320.85, 204.33),
    (330.88, 194.30), (330.88, 110.20), (320.85, 100.18), (230.41, 100.18),
    (215.41, 91.52), (170.19, 13.19), (156.50, 9.52), (83.66, 51.57),
    (79.99, 65.27), (125.21, 143.59), (125.21, 160.91), (79.99, 239.23),
    (83.66, 252.93), (156.50, 294.98),
]

# Centroid of the outline, and the centre of the OTOS circle: the origin of base_link,
# where the lidar sits (a static transform with no offset in the lidar's launch file).
CENTER = (190.345, 152.251)

# The strip starts at the middle of the chamfer between these two vertices:
# the notch at the bottom right of the drawing, marked by the yellow line.
START_EDGE = (1, 2)

# Which way the strip runs from there: "ccw" seen from above goes towards the +x lobe
# first, since START_EDGE runs that way in OUTLINE. Not decided on the robot yet.
DIRECTION = "ccw"

# The ring is the first 94 cm of the strip, from LED 0 at the start; the rest of the
# strip stays dark (BOARD_LED_STRIP_PIXELS in include/board.h).
LED_COUNT = 57
# 60 LEDs per metre nominally; 56 of them measure 92 cm on the test strip.
LED_PITCH_MM = 920.0 / 56.0

OUT = Path(__file__).resolve().parent.parent / "include" / "led_map.h"


def wrap(angle):
    """Angle in (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def to_ros(point):
    return (point[0] - CENTER[0], -(point[1] - CENTER[1]))


def strip_path():
    """The closed path of the strip, from its start, in the strip's direction.

    OUTLINE runs anticlockwise seen from above, which the check in sectors() enforces:
    START_EDGE[1] is the next vertex after START_EDGE[0].
    """
    a, b = OUTLINE[START_EDGE[0]], OUTLINE[START_EDGE[1]]
    start = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    if DIRECTION == "ccw":
        index, step = START_EDGE[1], 1
    else:
        index, step = START_EDGE[0], -1
    path = [start]
    for _ in range(len(OUTLINE)):
        path.append(OUTLINE[index])
        index = (index + step) % len(OUTLINE)
    path.append(start)
    return path


def point_at(path, lengths, s):
    s %= sum(lengths)
    for i in range(len(path) - 1):
        if s <= lengths[i] or i == len(path) - 2:
            t = 0.0 if lengths[i] == 0 else min(s / lengths[i], 1.0)
            return (path[i][0] + t * (path[i + 1][0] - path[i][0]),
                    path[i][1] + t * (path[i + 1][1] - path[i][1]))
        s -= lengths[i]
    return path[-1]


def led_edges(perimeter):
    """Where each LED's stretch of strip starts along the path, and where the last ends.

    A strip at least as long as the outline is laid on it proportionally. A shorter one
    keeps its pitch from the start and leaves a gap before it, which the first and the
    last LED share so that every direction still belongs to one LED.
    """
    strip = LED_COUNT * LED_PITCH_MM
    if strip >= perimeter:
        return [perimeter * i / LED_COUNT for i in range(LED_COUNT + 1)]
    edges = [LED_PITCH_MM * i for i in range(LED_COUNT + 1)]
    gap = perimeter - strip
    edges[0] -= gap / 2
    edges[-1] += gap / 2
    return edges


def sectors():
    path = strip_path()
    lengths = [math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)]
    perimeter = sum(lengths)
    edges = led_edges(perimeter)
    result = []
    for i in range(LED_COUNT):
        first = point_at(path, lengths, edges[i])
        last = point_at(path, lengths, edges[i + 1])
        middle = point_at(path, lengths, (edges[i] + edges[i + 1]) / 2)
        a = math.atan2(*reversed(to_ros(first)))
        b = math.atan2(*reversed(to_ros(last)))
        if DIRECTION == "ccw":
            start, width = a, wrap(b - a)
        else:
            start, width = b, wrap(a - b)
        if not 0 < width < math.pi / 2:
            sys.exit(f"LED {i}: the outline is not star-shaped around the centre (width {width})")
        radius = math.hypot(*to_ros(middle)) / 1000.0
        result.append((wrap(start), width, radius))
    return perimeter, result


def render():
    perimeter, table = sectors()
    # Every direction belongs to exactly one LED: the sectors tile the circle.
    total = sum(width for _, width, _ in table)
    if abs(total - 2 * math.pi) > 1e-6:
        sys.exit(f"the sectors cover {math.degrees(total):.3f} degrees, not 360")
    lines = [
        "/*",
        " * Generated by tools/gen_led_map.py: do not edit. See that file for the geometry.",
        " *",
        f" * {LED_COUNT} LEDs on a {LED_COUNT * LED_PITCH_MM / 10:.0f} cm strip along a {perimeter:.0f} mm outline,",
        f" * {DIRECTION} seen from above from the start of the strip. LED_SECTORS[i] is the angle span, in",
        " * base_link, that LED i shows as seen from the lidar, and the distance of its middle from it.",
        " * The spans tile the full circle: each direction belongs to exactly one LED.",
        " */",
        "",
        "#ifndef IMU_LED_LED_MAP_H",
        "#define IMU_LED_LED_MAP_H",
        "",
        f"#define LED_COUNT {LED_COUNT}",
        "",
        "typedef struct {",
        "  /* Anticlockwise from +x, in (-pi, pi]. The span may cross +-pi. */",
        "  float start_rad;",
        "  float width_rad;",
        "  /* Distance from the lidar to the middle of the LED. */",
        "  float radius_m;",
        "} led_sector_t;",
        "",
        "static const led_sector_t LED_SECTORS[LED_COUNT] = {",
    ]
    for i, (start, width, radius) in enumerate(table):
        lines.append(f"    {{{start:.6f}f, {width:.6f}f, {radius:.4f}f}}, /* {i:2d}: {math.degrees(start):8.2f} deg, "
                     f"{math.degrees(width):5.2f} deg wide */")
    lines += ["};", "", "#endif /* IMU_LED_LED_MAP_H */", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    text = render()
    if args.check:
        if not OUT.is_file() or OUT.read_text() != text:
            sys.exit(f"{OUT} is out of date: run tools/gen_led_map.py")
        print(f"{OUT.name} is up to date")
        return
    OUT.write_text(text)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
