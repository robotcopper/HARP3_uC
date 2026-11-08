"""Draws the charts of docs/picoros_vs_microros from its results files, and
renders the Graphviz diagrams of its images directory, all as SVG.

Distributions are drawn from the statistics the results files hold: the box
spans the 10th to the 90th percentile, the whiskers the minimum and the
maximum, and the line inside the box is the median.

Usage: figures.py
Needs matplotlib and Graphviz's dot.
"""

import re
import subprocess
from pathlib import Path

import matplotlib

matplotlib.use("svg")
import matplotlib.pyplot as plt  # noqa: E402

DOC = Path(__file__).resolve().parents[2] / "docs" / "picoros_vs_microros"
RESULTS = DOC / "results"
IMAGES = DOC / "images"

PICOROS = "#e08a1e"
MICROROS = "#1c9fd0"
HOST = "#4762a6"

# Pico-ROS times the downlink from the rmw attachment, which the host's
# publisher stamps this much later than header.stamp on average (stamp_gap.py).
PICOROS_STAMP_GAP_US = 120

STAT = re.compile(r"(?P<name>.+?) n=\d+ min=(?P<min>-?\d+) p10=(?P<p10>-?\d+) median=(?P<median>-?\d+) "
                  r"mean=(?P<mean>-?\d+) p90=(?P<p90>-?\d+) max=(?P<max>-?\d+)")


def load(label):
    """The phases of the results file of a run, each a dict of its statistics."""
    pattern = re.compile(re.escape(label) + r"_\d{8}_\d{6}\.txt")
    paths = [p for p in RESULTS.iterdir() if pattern.fullmatch(p.name)]
    if len(paths) != 1:
        raise SystemExit(f"{label}: expected one results file, found {len(paths)}")
    phases = {}
    phase = None
    for line in paths[0].read_text().splitlines():
        m = re.match(r"\[(\w+)\]", line)
        if m:
            phase = phases.setdefault(m.group(1), {})
            continue
        if phase is None:
            continue
        line = line.strip()
        m = re.match(r"idle mean=([\d.]+) %", line)
        if m:
            phase["idle"] = float(m.group(1))
            continue
        m = STAT.match(line)
        if m:
            phase[m.group("name")] = {k: float(v) for k, v in m.groupdict().items() if k != "name"}
    return phases


def shifted(stats, us):
    return {k: v + us for k, v in stats.items()}


def box(stats, label, scale=1e-3):
    return {"label": label, "med": stats["median"] * scale, "q1": stats["p10"] * scale,
            "q3": stats["p90"] * scale, "whislo": stats["min"] * scale, "whishi": stats["max"] * scale}


def draw_boxes(ax, entries, unit="ms", annotation=lambda b: f"{b['med']:.2f}"):
    """entries: (stats, label, colour), the statistics in microseconds."""
    boxes = [box(s, label) for s, label, _ in entries]
    artists = ax.bxp(boxes, showfliers=False, patch_artist=True, widths=0.55,
                     medianprops={"color": "black", "linewidth": 1.5})
    for patch, (_, _, colour) in zip(artists["boxes"], entries):
        patch.set_facecolor(colour)
        patch.set_alpha(0.45)
        patch.set_edgecolor(colour)
    for i, b in enumerate(boxes, start=1):
        ax.annotate(annotation(b), (i + 0.3, b["med"]), va="center", fontsize=8)
    ax.set_ylabel(unit)
    ax.grid(axis="y", color="#dddddd")
    ax.set_axisbelow(True)


def save(fig, name):
    fig.savefig(IMAGES / f"{name}.svg", facecolor="white", metadata={"Date": None})
    plt.close(fig)


def cpu_progression():
    steps = [
        ("rmw_zenoh_pico\nat -O3", "rmw_O3_cpu_stats"),
        ("no function\nentry trace", "rmw_O3_no_entry_trace"),
        ("profiler built in:\nlayout only", "profiler"),
        ("hot code\nin RAM", "hot_ram"),
        ("rmw_wait\ncounters", "wait_counts"),
        ("cadence task,\nno rcl timer", "cadence"),
        ("rmw_wait\nrace fixed", "cadence_raced_99hz"),
        ("200 MHz", "clock200_99hz"),
        ("RX cache", "clock200_rx_cache_99hz"),
        ("benchmark\nfirmware", "microros_rp2040_basic_99hz"),
    ]
    runs = [load(label) for _, label in steps]
    picoros_125 = load("picoros_basic_99hz")
    picoros_200 = load("picoros_rp2040_basic_99hz")
    first_200 = 7

    fig, ax = plt.subplots(figsize=(11, 5.4))
    x = range(len(steps))
    no_joint = [r["no_joint"]["idle"] for r in runs]
    joint = [r["joint"]["idle"] for r in runs]
    ax.bar([i - 0.2 for i in x], no_joint, 0.4, color=MICROROS, alpha=0.4, label="micro-ROS, no JointState")
    ax.bar([i + 0.2 for i in x], joint, 0.4, color=MICROROS, label="micro-ROS, JointState at 99-100 Hz")
    for i, v in enumerate(joint):
        ax.annotate(f"{v:.1f}", (i + 0.2, v + 1), ha="center", fontsize=8)
    for lo, hi, ref, clock in ((-0.5, first_200 - 0.5, picoros_125, 125), (first_200 - 0.5, len(steps) - 0.5,
                                                                          picoros_200, 200)):
        ax.hlines(ref["joint"]["idle"], lo, hi, colors=PICOROS, linestyles="--",
                  label="Pico-ROS, JointState at 99 Hz" if clock == 125 else None)
        ax.hlines(ref["no_joint"]["idle"], lo, hi, colors=PICOROS, linestyles=":",
                  label="Pico-ROS, no JointState" if clock == 125 else None)
    ax.axvline(first_200 - 0.5, color="#888888", linewidth=0.8)
    ax.text(first_200 / 2 - 0.5, 103, "125 MHz", ha="center")
    ax.text((first_200 + len(steps)) / 2 - 0.5, 103, "200 MHz", ha="center")
    ax.set_xticks(list(x), [s for s, _ in steps], fontsize=8)
    ax.set_ylim(0, 110)
    ax.set_ylabel("idle time, % of the CPU")
    ax.grid(axis="y", color="#dddddd")
    ax.set_axisbelow(True)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=2, fontsize=8)
    ax.set_title("micro-ROS idle time, one optimisation at a time, against Pico-ROS")
    fig.tight_layout()
    save(fig, "cpu_progression")


def latency_sum():
    key = "downlink + uplink per stats period"
    entries = [
        (load("host_ref_peer")["joint"][key], "host only\npeer", HOST),
        (load("host_ref_client")["joint"][key], "host only\nclient", HOST),
        (shifted(load("picoros_basic_99hz")["joint"][key], PICOROS_STAMP_GAP_US), "Pico-ROS\n125 MHz", PICOROS),
        (load("rx_split_99hz")["joint"][key], "micro-ROS\n125 MHz", MICROROS),
        (shifted(load("picoros_rp2040_basic_99hz")["joint"][key], PICOROS_STAMP_GAP_US), "Pico-ROS\n200 MHz",
         PICOROS),
        (load("microros_rp2040_basic_99hz")["joint"][key], "micro-ROS\n200 MHz", MICROROS),
    ]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    draw_boxes(ax, entries)
    ax.set_ylim(0, None)
    ax.set_title("Downlink + uplink, JointState at 99 Hz, per 1 s stats period")
    fig.tight_layout()
    save(fig, "latency_sum")


def service_rtt():
    key = "round trip"
    picoros = load("picoros_rp2040_basic_service")
    microros = load("microros_rp2040_basic_service")
    entries = [
        (load("host_ref_service_peer")["idle"][key], "host only\npeer", HOST),
        (load("host_ref_service_client")["idle"][key], "host only\nclient", HOST),
        (picoros["idle"][key], "Pico-ROS\nalone", PICOROS),
        (picoros["joint"][key], "Pico-ROS\n+ JointState", PICOROS),
        (microros["idle"][key], "micro-ROS\nalone", MICROROS),
        (microros["joint"][key], "micro-ROS\n+ JointState", MICROROS),
    ]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    draw_boxes(ax, entries)
    ax.set_ylim(0, None)
    ax.set_title("Service round trip, std_srvs/Trigger, 1000 calls, 200 MHz")
    fig.tight_layout()
    save(fig, "service_rtt")


def jitter():
    picoros = load("picoros_rp2040_basic_99hz")["joint"]
    microros = load("microros_rp2040_basic_99hz")["joint"]
    peer = load("host_ref_peer")["joint"]
    client = load("host_ref_client")["joint"]
    spread = "downlink max - min per 1 s"

    def centred(stats):
        return shifted(stats, -stats["median"])

    fig, (left, right) = plt.subplots(1, 2, figsize=(11, 4.5))
    draw_boxes(left, [
        (peer[spread], "host only\npeer", HOST),
        (client[spread], "host only\nclient", HOST),
        (picoros[spread], "Pico-ROS", PICOROS),
        (microros[spread], "micro-ROS", MICROROS),
    ])
    left.set_ylim(0, None)
    left.set_title("Downlink: max - min of each second's 99 JointStates", fontsize=10)
    draw_boxes(right, [
        (centred(peer["uplink"]), "host only\npeer", HOST),
        (centred(client["uplink"]), "host only\nclient", HOST),
        (centred(picoros["uplink"]), "Pico-ROS", PICOROS),
        (centred(microros["uplink"]), "micro-ROS", MICROROS),
    ], unit="ms from the median", annotation=lambda b: f"p90-p10\n{b['q3'] - b['q1']:.2f}")
    right.set_title("Uplink: each odometry message, from the run's median", fontsize=10)
    fig.suptitle("Jitter at 200 MHz, JointState at 99 Hz")
    fig.tight_layout()
    save(fig, "jitter")


def diagrams():
    for dot in sorted(IMAGES.glob("*.dot")):
        subprocess.run(["dot", "-Tsvg", str(dot), "-o", str(dot.with_suffix(".svg"))], check=True)


def main():
    plt.rcParams["svg.hashsalt"] = "picoros_vs_microros"
    diagrams()
    cpu_progression()
    latency_sum()
    service_rtt()
    jitter()


if __name__ == "__main__":
    main()
