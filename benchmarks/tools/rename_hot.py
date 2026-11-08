"""Renames the sections of the functions listed in <list>, from .text.<name> to
.time_critical.hot.<name>, in the objects under the given directories and in
the given archives. The Pico SDK's linker script places .time_critical* in
.data, which crt0 copies to RAM before anything else runs.

The names applied by the previous run, kept in <state>, are renamed back
first, so that the result depends only on the current list, whatever was
recompiled in between. objcopy ignores sections a file does not have.

Usage: rename_hot.py <objcopy> <list> <state> <directory or archive>...
"""

import subprocess
import sys
from pathlib import Path

PREFIX = ".time_critical.hot."


def read_names(path):
    if not path.exists():
        return []
    return [line.strip() for line in path.read_text().splitlines() if line.strip() and not line.startswith("#")]


def rename(objcopy, files, pairs):
    if not pairs:
        return
    options = [f"--rename-section={old}={new}" for old, new in pairs]
    for path in files:
        subprocess.run([objcopy] + options + [str(path)], check=True)


def main():
    objcopy, list_path, state_path = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
    files = []
    for argument in sys.argv[4:]:
        path = Path(argument)
        if path.is_dir():
            files += sorted(p for p in path.rglob("*") if p.suffix in (".o", ".obj"))
        else:
            files.append(path)

    previous = read_names(state_path)
    current = read_names(list_path)
    rename(objcopy, files, [(PREFIX + name, ".text." + name) for name in previous])
    rename(objcopy, files, [(".text." + name, PREFIX + name) for name in current])
    state_path.write_text("\n".join(current) + "\n")
    print(f"rename_hot: {len(current)} functions to RAM in {len(files)} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
