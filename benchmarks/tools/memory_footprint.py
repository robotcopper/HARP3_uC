"""Flash and RAM that the test firmwares take before anything runs, read from
their ELF files with the symbols of the Pico SDK's linker script.

  flash      bytes written to flash, up to __flash_binary_end
  static     main RAM from its start up to __end__, less the FreeRTOS heap
  freertos   the FreeRTOS heap, ucHeap, configTOTAL_HEAP_SIZE bytes
  malloc     main RAM the linker script leaves to newlib's malloc(), from
             __end__ to __HeapLimit
  ram code   the functions linked to run from RAM, part of static

static, freertos and malloc add up to the RP2040's 256 KB of main RAM; the
two 4 KB scratch banks, which hold the initial stacks of the two cores, are
left out.

Usage: memory_footprint.py <label> <elf>... [--readelf arm-none-eabi-readelf]
Results are printed and written to <label>_<date>.txt in RESULTS.
"""

import argparse
import subprocess
import sys
from datetime import datetime
from pathlib import Path

RESULTS = Path(__file__).resolve().parents[2] / "docs" / "picoros_vs_microros" / "results"
FLASH_START = 0x10000000
RAM_START = 0x20000000
RAM_END = 0x20040000
SCRATCH_END = 0x20042000


def read_symbols(readelf, elf):
    """(value, size, type, name) of each defined symbol of an ELF."""
    out = subprocess.run([readelf, "-sW", str(elf)], capture_output=True, text=True, check=True).stdout
    rows = []
    for line in out.splitlines():
        fields = line.split()
        if len(fields) < 8 or not fields[0].rstrip(":").isdigit() or fields[6] == "UND":
            continue
        rows.append((int(fields[1], 16), int(fields[2], 0), fields[3], fields[7]))
    return rows


def footprint(readelf, elf):
    rows = read_symbols(readelf, elf)
    table = {name: (value, size) for value, size, _, name in rows}
    # Keyed by address, so that a function with two names counts once.
    ram_code = {value & ~1: size for value, size, kind, _ in rows
                if kind == "FUNC" and RAM_START <= value & ~1 < SCRATCH_END}
    end = table["__end__"][0]
    freertos = table["ucHeap"][1]
    return {
        "flash": table["__flash_binary_end"][0] - FLASH_START,
        "static": end - RAM_START - freertos,
        "freertos": freertos,
        "malloc": table["__HeapLimit"][0] - end,
        "ram code": sum(ram_code.values()),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("label")
    parser.add_argument("elf", nargs="+", type=Path)
    parser.add_argument("--readelf", default="arm-none-eabi-readelf")
    args = parser.parse_args()

    started = datetime.now()
    lines = [f"{args.label} {started.isoformat(timespec='seconds')} static memory, bytes"]
    for elf in args.elf:
        sizes = footprint(args.readelf, elf)
        if sizes["static"] + sizes["freertos"] + sizes["malloc"] != RAM_END - RAM_START:
            print(f"{elf}: static, freertos and malloc do not add up to the main RAM", file=sys.stderr)
            return 1
        lines.append(f"[{elf.stem}]")
        lines += [f"  {name}={value} ({value / 1024:.1f} KB)" for name, value in sizes.items()]
    text = "\n".join(lines)
    print(text)
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / f"{args.label}_{started:%Y%m%d_%H%M%S}.txt").write_text(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
