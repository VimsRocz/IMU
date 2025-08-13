#!/usr/bin/env python
import sys, os
from pathlib import Path
from task6_overlay_all_frames import run_task6_overlay_all_frames


if len(sys.argv) < 4:
    print('Usage: task6_overlay_debug_min.py <est.mat/npz> <truth.txt> <outdir> [lat lon]')
    sys.exit(1)

est, truth, outdir = sys.argv[1:4]
lat = float(sys.argv[4]) if len(sys.argv) > 4 else None
lon = float(sys.argv[5]) if len(sys.argv) > 5 else None
Path(outdir).mkdir(parents=True, exist_ok=True)
print('DEBUG: calling run_task6_overlay_all_frames()...')
paths = run_task6_overlay_all_frames(
    est_file=est, truth_file=truth, output_dir=outdir, lat_deg=lat, lon_deg=lon
)
print('DEBUG: returned:', paths)
for p in Path(outdir).glob('*.png'):
    print('PNG:', p.resolve())

