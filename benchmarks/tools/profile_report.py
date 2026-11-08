"""Maps the program counter samples of the micro-ROS firmware, as measure.py
--profile saves them, to functions and components of the ELF that was flashed.

Each 64-byte bucket of flash is credited to the function its start falls in.
The idle task's functions are reported apart; the shares of "busy" time leave
them out. The coverage table orders functions by samples per byte, which is
the order in which moving them to RAM saves the most XIP cache misses per byte
of RAM: it gives the bytes needed to cover a share of the busy samples.

With --map, the same is done per object file, from the linker map: the Pico
SDK's linker script moves whole object files to RAM, by excluding them from
.text, so that is the granularity that costs no code.

Usage: profile_report.py <profile.txt> <elf> [--map <elf.map>] [--top 40]
"""

import argparse
import bisect
import re
import subprocess
import sys
from collections import defaultdict

IDLE_FUNCTIONS = {"prvIdleTask", "vApplicationIdleHook", "prvCheckTasksWaitingTermination"}
COMPONENTS = [
    ("zenoh-pico", "zenoh-pico"),
    ("rmw_zenoh_pico", "rmw_zenoh_pico"),
    ("/rclc/", "rclc"),
    ("/rcl/", "rcl"),
    ("rcutils", "rcutils"),
    ("microcdr", "microcdr"),
    ("Micro-CDR", "microcdr"),
    ("rosidl", "rosidl"),
    ("_c/msg/", "typesupport"),
    ("_c/srv/", "typesupport"),
    ("/rmw/", "rmw"),
    ("lwip", "lwip"),
    ("tinyusb", "tinyusb"),
    ("FreeRTOS-Kernel", "freertos"),
    ("picoros-usb-eth", "platform"),
    ("firmware/src", "app"),
    ("pico-sdk", "pico-sdk"),
]


def parse_profile(path):
    buckets = defaultdict(int)
    totals = {}
    for line in open(path):
        line = line.strip()
        if line.startswith("profile end"):
            totals = dict(re.findall(r"(\w+)=(\w+)", line))
        elif line.startswith("profile "):
            for index, count in re.findall(r" ([0-9a-f]+):(\d+)", line):
                buckets[int(index, 16)] += int(count)
    if not totals:
        sys.exit(f"{path}: no 'profile end' line")
    return buckets, totals


def read_functions(elf):
    output = subprocess.run(["arm-none-eabi-nm", "-n", "-S", "--defined-only", elf],
                            capture_output=True, text=True, check=True).stdout
    functions = []
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2] in "tTwW":
            address, size = int(parts[0], 16), int(parts[1], 16)
            if 0x10000000 <= address < 0x11000000 and size > 0:
                functions.append((address & ~1, size, parts[3]))
    functions.sort()
    return functions


def components_of(elf, functions):
    addresses = [f"0x{address:x}" for address, _, _ in functions]
    output = subprocess.run(["arm-none-eabi-addr2line", "-e", elf] + addresses,
                            capture_output=True, text=True, check=True).stdout.splitlines()
    result = {}
    for (address, _, name), source in zip(functions, output):
        component = "libc/libgcc" if source.startswith("??") else "other"
        for needle, label in COMPONENTS:
            if needle in source:
                component = label
                break
        result[name] = component
    return result


def read_objects(map_path):
    """(address, size, object, section) of every input .text section placed in flash."""
    sections = []
    pending = None
    started = False
    for line in open(map_path):
        if not started:
            started = line.startswith("Linker script and memory map")
            continue
        if pending is not None:
            match = re.match(r"\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.*)$", line)
            if match:
                sections.append((int(match.group(1), 16), int(match.group(2), 16), match.group(3).strip(), pending))
            pending = None
            continue
        match = re.match(r"^ (\.text\S*)\s+0x([0-9a-f]+)\s+0x([0-9a-f]+)\s+(\S.*)$", line)
        if match:
            sections.append((int(match.group(2), 16), int(match.group(3), 16), match.group(4).strip(),
                             match.group(1)))
        else:
            match = re.match(r"^ (\.text\S*)\s*$", line)
            if match:
                pending = match.group(1)
    return sorted(s for s in sections if 0x10000000 <= s[0] < 0x11000000 and s[1] > 0)


def short_object(path):
    match = re.search(r"\(([^)]+)\)$", path)
    archive = re.search(r"([^/]+\.a)\(", path)
    if match and archive:
        return f"{archive.group(1)}:{match.group(1)}"
    return path.split("/")[-1]


def report_objects(buckets, base, shift, sections, busy_flash, idle_buckets):
    starts = [section[0] for section in sections]
    samples = defaultdict(int)
    sizes = defaultdict(int)
    for _, size, path, _ in sections:
        sizes[path] += size
    for index, count in buckets.items():
        if index in idle_buckets:
            continue
        address = base + (index << shift)
        i = bisect.bisect_right(starts, address) - 1
        if i >= 0 and address < sections[i][0] + sections[i][1]:
            samples[sections[i][2]] += count
    hot = sorted(((n, path) for path, n in samples.items()), reverse=True)
    print("\ntop object files, share of busy flash samples:")
    print("   busy %   cumul %    size   object")
    cumulative = 0
    for n, path in hot[:30]:
        cumulative += n
        print(f"  {100 * n / busy_flash:6.2f}   {100 * cumulative / busy_flash:6.1f}   {sizes[path]:6d}   "
              f"{short_object(path)}")
    by_density = sorted(hot, key=lambda item: -item[0] / max(sizes[item[1]], 1))
    print("\nRAM needed per object file, densest first:")
    goals = [0.25, 0.5, 0.75, 0.9]
    cumulative = size = count = 0
    for n, path in by_density:
        cumulative += n
        size += sizes[path]
        count += 1
        while goals and cumulative >= goals[0] * busy_flash:
            print(f"  {100 * goals.pop(0):3.0f} % of flash samples: {size / 1024:6.1f} KB in {count} objects")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("profile")
    parser.add_argument("elf")
    parser.add_argument("--map")
    parser.add_argument("--top", type=int, default=40)
    args = parser.parse_args()

    buckets, totals = parse_profile(args.profile)
    base, shift = int(totals["base"], 16), int(totals["shift"])
    functions = read_functions(args.elf)
    starts = [address for address, _, _ in functions]

    samples = defaultdict(int)
    sizes = {name: size for _, size, name in functions}
    idle_buckets = set()
    for index, count in buckets.items():
        address = base + (index << shift)
        i = bisect.bisect_right(starts, address) - 1
        name = functions[i][2] if i >= 0 else "?"
        if name in IDLE_FUNCTIONS:
            idle_buckets.add(index)
        if i >= 0 and address >= functions[i][0] + functions[i][1]:
            name += "+"
        samples[name] += count
    component = components_of(args.elf, functions)

    total = int(totals["samples"])
    idle = sum(n for name, n in samples.items() if name.rstrip("+") in IDLE_FUNCTIONS)
    rom, ram = int(totals["rom"]), int(totals["ram"])
    busy = total - idle
    period_us = int(totals["period_us"])
    print(f"samples={total} every {period_us} us ({total * period_us / 1e6:.1f} s), "
          f"saturated buckets={totals['saturated']}")
    print(f"idle={100 * idle / total:.1f} %  rom={100 * rom / total:.1f} %  ram={100 * ram / total:.1f} %  "
          f"busy={100 * busy / total:.1f} %")

    per_component = defaultdict(int)
    for name, n in samples.items():
        if name.rstrip("+") not in IDLE_FUNCTIONS:
            per_component[component.get(name.rstrip("+"), "other")] += n
    per_component["rom (memcpy, memset, float)"] = rom
    per_component["ram (time_critical)"] = ram
    print("\nbusy share by component:")
    for name, n in sorted(per_component.items(), key=lambda item: -item[1]):
        print(f"  {100 * n / busy:5.1f} %  {name}")

    hot = sorted(((n, name) for name, n in samples.items() if name.rstrip("+") not in IDLE_FUNCTIONS),
                 reverse=True)
    print(f"\ntop {args.top} functions, share of busy samples:")
    print("   busy %   cumul %   size   function (component)")
    cumulative = 0
    for n, name in hot[:args.top]:
        cumulative += n
        bare = name.rstrip("+")
        print(f"  {100 * n / busy:6.2f}   {100 * cumulative / busy:6.1f}   {sizes.get(bare, 0):5d}   "
              f"{name} ({component.get(bare, 'other')})")

    by_density = sorted(hot, key=lambda item: -item[0] / max(sizes.get(item[1].rstrip("+"), 64), 1))
    print("\nRAM needed to cover a share of the busy samples, densest functions first:")
    goals = [0.25, 0.5, 0.75, 0.9]
    cumulative = size = count = 0
    for n, name in by_density:
        cumulative += n
        size += sizes.get(name.rstrip("+"), 0)
        count += 1
        while goals and cumulative >= goals[0] * (busy - rom - ram):
            print(f"  {100 * goals.pop(0):3.0f} % of flash samples: {size / 1024:6.1f} KB in {count} functions")

    if args.map:
        report_objects(buckets, base, shift, read_objects(args.map), busy - rom - ram, idle_buckets)
    return 0


if __name__ == "__main__":
    sys.exit(main())
