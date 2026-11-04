#!/usr/bin/env python3
"""Compare every type hash of a Pico-ROS type header with the .json rosidl installed.

    check_type_hashes.py HEADER SHARE_DIR [SHARE_DIR ...]

A type whose .json is found in no share directory is reported as unchecked. Exits
non-zero on any mismatch.
"""

import json
import re
import sys
from pathlib import Path

ENTRY = re.compile(r'(?:CTYPE|SRV)\(\w+,\s*\\\s*"(\w+)::(msg|srv)::dds_::(\w+)",\s*\\\s*"([0-9a-f]{64})"')


def installed_hash(shares, package, kind, name):
    for share in shares:
        path = Path(share) / package / kind / f'{name}.json'
        if path.is_file():
            data = json.loads(path.read_text())
            wanted = f'{package}/{kind}/{name}'
            for entry in data['type_hashes']:
                if entry['type_name'] == wanted:
                    return entry['hash_string'].removeprefix('RIHS01_')
    return None


def main():
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    header = Path(sys.argv[1]).read_text()
    shares = sys.argv[2:]
    checked, unchecked, mismatched = 0, [], []
    for package, kind, name, generated in ENTRY.findall(header):
        expected = installed_hash(shares, package, kind, name)
        if expected is None:
            unchecked.append(f'{package}/{kind}/{name}')
        elif expected != generated:
            mismatched.append(f'{package}/{kind}/{name}: header {generated[:16]}, installed {expected[:16]}')
        else:
            checked += 1
    print(f'{checked} type hashes match the installed definitions')
    if unchecked:
        print('unchecked, no installed .json: ' + ', '.join(unchecked))
    for line in mismatched:
        print('MISMATCH ' + line, file=sys.stderr)
    return 1 if mismatched else 0


if __name__ == '__main__':
    sys.exit(main())
