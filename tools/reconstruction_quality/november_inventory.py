#!/usr/bin/env python3
"""Snapshot the two shared November campaigns without modifying Drive files."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path


def snapshot(source, output):
    if not source.is_dir():
        raise FileNotFoundError(source)
    output.mkdir(parents=True, exist_ok=False)
    inventory = []
    jobs = []
    for path in sorted(source.rglob('*')):
        if not path.is_file():
            continue
        rel = path.relative_to(source)
        # Raw IR and STL duplicate information unnecessary for this depth audit.
        selected = path.suffix in ('.yaml', '.json', '.ply', '.png') and 'ir_raw' not in path.parts and 'test_capture' not in path.parts
        row = dict(path=str(rel), size=path.stat().st_size, selected=selected)
        inventory.append(row)
        if selected:
            jobs.append((path, output / 'data' / rel, row))
    if not jobs:
        raise ValueError('No input files found')

    def copy(job):
        src, dst, row = job
        data = src.read_bytes()
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(data)
        row['sha256'] = hashlib.sha256(data).hexdigest()
        assert hashlib.sha256(dst.read_bytes()).hexdigest() == row['sha256']
        return row['size']

    total = 0
    with ThreadPoolExecutor(max_workers=4) as pool:
        for n, future in enumerate(as_completed(pool.submit(copy, j) for j in jobs), 1):
            total += future.result()
            if n % 25 == 0 or n == len(jobs):
                print(f'Snapshotted {n}/{len(jobs)} files, {total / 1e6:.1f} MB', flush=True)
    record = dict(source=str(source), drive_url='https://drive.google.com/drive/folders/1fohHZ1mIbK39buiSANj6SX4hXxn0WKRm', files=inventory)
    (output / 'inventory.json').write_text(json.dumps(record, indent=2) + '\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('output', type=Path)
    args = p.parse_args()
    snapshot(args.source, args.output)
