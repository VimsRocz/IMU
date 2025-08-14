#!/usr/bin/env python
import argparse, json, os, sys, traceback
from pathlib import Path


def eprint(*a, **k):
    print(*a, file=sys.stderr, **k)


def derive_run_id(est_file: Path) -> str:
    stem = est_file.stem  # e.g., IMU_X002_GNSS_X002_TRIAD_kf_output
    # Keep what you already use elsewhere if present:
    return stem.replace('_kf_output', '')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--est-file', required=True)
    ap.add_argument('--truth-file', required=True)
    ap.add_argument('--gnss-file', default=None)
    ap.add_argument('--lat', type=float, default=None)
    ap.add_argument('--lon', type=float, default=None)
    ap.add_argument('--qbody', default=None, help='qw,qx,qy,qz constant fallback')
    ap.add_argument('--run-id', default=None)
    ap.add_argument('--triad-file', default=None)
    ap.add_argument('--davenport-file', default=None)
    ap.add_argument('--svd-file', default=None)
    ap.add_argument('--no-interactive', action='store_true', help='disable any interactive imports')
    ap.add_argument('--output', default=None, help='(ignored) kept for backward compat')
    ap.add_argument('--debug-task6', action='store_true', help='enable verbose Task 6 debugging')
    args = ap.parse_args()

    est_path = Path(args.est_file).resolve()
    run_id = args.run_id or derive_run_id(est_path)
    out_dir = Path(args.output or Path('PYTHON') / 'results').resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    from matplotlib.figure import Figure
    _orig_savefig = Figure.savefig
    SAVED_PLOTS = []
    def _sf(self, fname, *a, **k):
        path = out_dir / Path(fname).name
        _orig_savefig(self, path, *a, **k)
        print(f"[SAVE] {path}")
        SAVED_PLOTS.append(str(path))
    Figure.savefig = _sf

    time_hint = est_path.with_name(f"{run_id}_task5_time.mat")
    time_hint_path = str(time_hint) if time_hint.is_file() else None

    # Parse qbody
    q_const = None
    if args.qbody:
        try:
            parts = [float(x) for x in args.qbody.split(',')]
            if len(parts) == 4:
                q_const = tuple(parts)
        except Exception:
            eprint('WARN: could not parse --qbody, ignoring')

    # Import our static overlay helper ONLY (never import interactive on import)
    try:
        from task6_overlay_all_frames import (
            run_task6_overlay_all_frames,
            run_task6_compare_methods_all_frames
        )
    except Exception as ex:
        eprint('FATAL: cannot import task6_overlay_all_frames:', ex)
        traceback.print_exc()
        sys.exit(2)

    # Single-method overlays
    saved = {}
    try:
        saved_single = run_task6_overlay_all_frames(
            est_file=str(est_path),
            truth_file=args.truth_file,
            output_dir=str(out_dir),
            run_id=run_id,
            lat_deg=args.lat, lon_deg=args.lon,
            gnss_file=args.gnss_file,
            q_b2n_const=q_const,
            time_hint_path=time_hint_path,
            debug=args.debug_task6,
        )
        saved.update(saved_single if isinstance(saved_single, dict) else {})
    except Exception as ex:
        eprint('ERROR: run_task6_overlay_all_frames failed:', ex)
        traceback.print_exc()

    # Multi-method overlays (best-effort)
    method_files = {}
    if args.triad_file:
        method_files['TRIAD'] = args.triad_file
    if args.davenport_file:
        method_files['Davenport'] = args.davenport_file
    if args.svd_file:
        method_files['SVD'] = args.svd_file
    # Try to infer siblings if none provided:
    if not method_files:
        s = str(est_path)
        cand = [('Davenport', s.replace('TRIAD','Davenport')),
                ('SVD',       s.replace('TRIAD','SVD')),
                ('TRIAD',     s)]
        for name,p in cand:
            if os.path.isfile(p):
                method_files[name] = p

    if method_files:
        try:
            saved_multi = run_task6_compare_methods_all_frames(
                method_files=method_files,
                truth_file=args.truth_file,
                output_dir=str(out_dir),
                run_id=run_id,
                lat_deg=args.lat, lon_deg=args.lon,
                gnss_file=args.gnss_file,
                q_b2n_const=q_const,
                time_hint_path=time_hint_path,
                debug=args.debug_task6,
            )
            if isinstance(saved_multi, dict):
                saved.update(saved_multi)
        except Exception as ex:
            eprint('ERROR: run_task6_compare_methods_all_frames failed:', ex)
            traceback.print_exc()

    # Print what we created
    print('[TASK 6] Output dir:', out_dir)
    pngs = sorted([str(p) for p in out_dir.glob('*.png')])
    if pngs:
        print('[TASK 6] PNGs:')
        for p in pngs: print('  ', p)
    else:
        print('[TASK 6] No PNGs found. Debug manifest follows.')

    # Dump (or append) manifest
    manifest = out_dir / 'task6_overlay_manifest.json'
    try:
        if manifest.exists():
            data = json.loads(manifest.read_text())
        else:
            data = {}
        data['run_id'] = run_id
        data['generated'] = pngs
        manifest.write_text(json.dumps(data, indent=2))
        print('[Task6] Manifest:', manifest)
    except Exception as ex:
        eprint('WARN: could not write manifest:', ex)


if __name__ == '__main__':
    main()

