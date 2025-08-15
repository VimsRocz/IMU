#!/usr/bin/env python
import argparse, json, os, sys, traceback
from pathlib import Path

from paths import ensure_results_dir
from utils.read_truth_txt import read_truth_txt
from task6_overlay_all_frames import ecef_to_ned, ecef_to_ned_vec
from scipy.interpolate import interp1d
import numpy as np
import pandas as pd


def eprint(*a, **k):
    print(*a, file=sys.stderr, **k)


def derive_run_id(est_file: Path) -> str:
    stem = est_file.stem  # e.g., IMU_X002_GNSS_X002_TRIAD_kf_output
    # Keep what you already use elsewhere if present:
    return stem.replace('_kf_output', '')


def _get_ref_scalar(store: dict, keys: list[str]):
    """Return first available scalar value for any key in ``keys``.

    Handles numpy arrays and MATLAB-style scalars by squeezing and taking the
    first element. Returns ``None`` if no key is present.
    """
    for k in keys:
        if k in store and store[k] is not None:
            v = np.asarray(store[k]).squeeze()
            if v.size >= 1:
                try:
                    return float(v.flat[0])
                except Exception:
                    continue
    return None


def _get_ref_vec3(store: dict, keys: list[str]):
    """Return first available 3-vector for any key in ``keys``.

    Coerces to ``float64`` and returns shape ``(3,)`` when possible.
    Returns ``None`` if not found.
    """
    for k in keys:
        if k in store and store[k] is not None:
            v = np.asarray(store[k], dtype=float).reshape(-1)
            if v.size >= 3:
                return v[:3]
    return None


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
    ap.add_argument('--flat-output', action='store_true', default=True, help='also copy PNGs to results/ root')
    args = ap.parse_args()

    est_path = Path(args.est_file).resolve()
    run_id = args.run_id or derive_run_id(est_path)

    # Ensure plots are saved under the repository's ``PYTHON/results`` directory
    res_dir = ensure_results_dir()
    out_dir = (res_dir / run_id / 'task6').resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    time_hint = est_path.with_name(f"{run_id}_task5_time.mat")
    time_hint_path = str(time_hint) if time_hint.is_file() else None

    # ------------------------------------------------------------------
    # Truth loading and estimator metadata debug
    # ------------------------------------------------------------------
    print(f"[Task6] truth_file={args.truth_file} exists={Path(args.truth_file).exists()}")
    try:
        t_truth, pos_ecef_truth, vel_ecef_truth = read_truth_txt(args.truth_file)
        print(
            f"[Task6][Truth] t:{t_truth.shape} pos_ecef:{pos_ecef_truth.shape} vel_ecef:{vel_ecef_truth.shape}"
        )
        if t_truth.size:
            print(
                f"[Task6][Truth] t[:3]={t_truth[:3]} ... t[-3:]={t_truth[-3:]}"
            )
    except Exception as ex:
        print(f"[Task6][Truth][ERROR] failed to load truth: {ex}")
        t_truth = np.array([])
        pos_ecef_truth = vel_ecef_truth = np.zeros((0, 3))

    # Load estimator file and time vector
    import scipy.io as sio
    if est_path.suffix.lower() == '.npz':
        est = {k: v for k, v in np.load(est_path, allow_pickle=True).items()}
    else:
        est = {k: v for k, v in sio.loadmat(est_path, squeeze_me=True).items() if not k.startswith('__')}
    print(f"[Task6][Est] keys={list(est.keys())}")
    t_est = est.get('t_est')
    if t_est is None:
        t_est = est.get('time_s')
    if t_est is None:
        dt = _get_ref_scalar(est, ['dt', 'imu_dt']) or 0.0
        n = None
        for key in ('pos_ned_m','pos_ecef_m','pos_body_m'):
            if key in est:
                n = len(est[key])
                break
        if n is not None and dt:
            t_est = np.arange(n) * float(dt)
    t_est = np.asarray(t_est, float).reshape(-1)
    if t_est.size:
        print(
            f"[Task6][Est] len(t_est)={len(t_est)} t0={t_est[0]:.3f} t1={t_est[-1]:.3f} dt≈{np.median(np.diff(t_est)) if len(t_est)>1 else 0:.6f}"
        )

    # Reference lat/lon/r0
    ref_lat_rad = _get_ref_scalar(est, ['ref_lat_rad', 'lat_rad', 'lat0_rad', 'lat'])
    ref_lon_rad = _get_ref_scalar(est, ['ref_lon_rad', 'lon_rad', 'lon0_rad', 'lon'])
    ref_r0_m = _get_ref_vec3(est, ['ref_r0_m', 'r0', 'r0_ecef_m', 'ecef_ref'])
    if ref_lat_rad is None or ref_lon_rad is None or ref_r0_m is None:
        print("[Task6] ref values missing, deriving from first GNSS sample")
        if args.gnss_file:
            df_g = pd.read_csv(args.gnss_file)
            row = df_g.iloc[0]
            x,y,z = float(row.get('X_ECEF_m',0)), float(row.get('Y_ECEF_m',0)), float(row.get('Z_ECEF_m',0))
            from utils import ecef_to_geodetic
            ref_lat_rad, ref_lon_rad, _ = ecef_to_geodetic(x,y,z)
            ref_r0_m = np.array([x,y,z],dtype=float)
    print(f"[Task6] ref_lat={ref_lat_rad} ref_lon={ref_lon_rad} r0={ref_r0_m}")

    # Interpolate truth into estimator timebase
    if t_truth.size and t_est.size:
        pos_ned_truth = ecef_to_ned(pos_ecef_truth, ref_lat_rad, ref_lon_rad, ref_r0_m)
        vel_ned_truth = ecef_to_ned_vec(vel_ecef_truth, ref_lat_rad, ref_lon_rad)
        interp = lambda X: np.column_stack([
            interp1d(t_truth, X[:,i], bounds_error=False, fill_value="extrapolate")(t_est) for i in range(3)
        ])
        pos_ned_truth_interp = interp(pos_ned_truth)
        print(
            f"[Task6] interp truth -> t_est: in={len(t_truth)} out={len(t_est)} NaNs_pos={np.isnan(pos_ned_truth_interp).sum()}"
        )
        assert pos_ned_truth_interp.shape[0] == t_est.shape[0], "Truth interp length mismatch"

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
            lat_deg=args.lat, lon_deg=args.lon,
            gnss_file=args.gnss_file,
            q_b2n_const=q_const,
            time_hint_path=time_hint_path,
            debug=args.debug_task6,
            flat_output=args.flat_output,
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
                lat_deg=args.lat, lon_deg=args.lon,
                gnss_file=args.gnss_file,
                q_b2n_const=q_const,
                time_hint_path=time_hint_path,
                debug=args.debug_task6,
                flat_output=args.flat_output,
            )
            if isinstance(saved_multi, dict):
                saved.update(saved_multi)
        except Exception as ex:
            eprint('ERROR: run_task6_compare_methods_all_frames failed:', ex)
            traceback.print_exc()

    # Print what we created
    print('[Task6] Output dir:', out_dir)
    pngs = sorted([str(p) for p in out_dir.glob('*.png')])
    print(f"[TASK 6] Plots saved to results/: {[Path(p).name for p in pngs]}")

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
