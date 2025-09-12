#!/usr/bin/env python
import argparse, json, os, sys, traceback
from pathlib import Path

from paths import ensure_results_dir, TRUTH_DIR
from utils.read_truth_txt import read_truth_txt
from task6_overlay_all_frames import ecef_to_ned, ecef_to_ned_vec, ned_to_ecef
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation, Slerp
import numpy as np
import pandas as pd
from utils.resolve_truth_path import resolve_truth_path
import shutil

# ------------------------------------------------------------------
# Helpers (robust metadata lookup and coercion)
# ------------------------------------------------------------------

def _get_first_key(d, keys):
    """Return the first non-None value for any key in keys from dict-like d."""
    for k in keys:
        if k in d:
            v = d.get(k)
            if v is not None:
                return v
    return None

import numpy as _np

def _as_vec3(x):
    """Coerce x to shape (3,) float array."""
    if x is None:
        return None
    x = _np.asarray(x, dtype=float)
    x = x.squeeze()
    if x.size != 3:
        # Try to take first 3 if it’s longer; else fail clearly
        if x.size > 3:
            x = x.ravel()[:3]
        else:
            raise ValueError(f"Expected 3 elements for vec3, got shape {x.shape} with size {x.size}")
    return x


def eprint(*a, **k):
    print(*a, file=sys.stderr, **k)


def derive_run_id(est_file: Path) -> str:
    stem = est_file.stem  # e.g., IMU_X002_GNSS_X002_TRIAD_kf_output
    # Keep what you already use elsewhere if present:
    return stem.replace('_kf_output', '')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--est-file', required=True)
    ap.add_argument('--truth-file', required=False, help='Truth file path; if omitted, auto-resolve canonical truth')
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
    ap.add_argument('--include-accel', action='store_true', default=False, help='Include acceleration overlays (default False)')
    ap.add_argument('--no-decimate', action='store_true', default=False, help='Disable plot decimation for long runs')
    ap.add_argument('--decimate-maxpoints', type=int, default=200_000, help='Target max points after decimation (per series)')
    ap.add_argument('--ylim-percentile', type=float, default=99.5, help='Percentile (0-100) for symmetric y-limit scaling')
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
    # Resolve truth file if not provided or missing
    truth_path = args.truth_file
    if not truth_path or not Path(truth_path).exists():
        try:
            auto = resolve_truth_path()
        except Exception:
            auto = None
        if auto and Path(auto).exists():
            truth_path = auto
            print(f"[Task6] Using auto-resolved truth file: {truth_path}")
    if not truth_path or not Path(truth_path).exists():
        raise FileNotFoundError("Truth file not provided and could not be auto-resolved.")

    print(f"[Task6] truth_file={truth_path} exists={Path(truth_path).exists()}")

    # Export/copy truth into canonical DATA/Truth/STATE_X001.txt for future runs
    try:
        TRUTH_DIR.mkdir(parents=True, exist_ok=True)
        canonical_truth = TRUTH_DIR / 'STATE_X001.txt'
        if Path(truth_path).resolve() != canonical_truth.resolve():
            shutil.copy2(truth_path, canonical_truth)
            print(f"[Task6] Exported truth to {canonical_truth}")
    except Exception as ex:
        eprint(f"WARN: could not export truth to canonical path: {ex}")
    try:
        t_truth, pos_ecef_truth, vel_ecef_truth, quat_truth = read_truth_txt(truth_path)
        print(
            f"[Task6][Truth] t:{t_truth.shape} pos_ecef:{pos_ecef_truth.shape} vel_ecef:{vel_ecef_truth.shape} quat:{quat_truth.shape}"
        )
        if t_truth.size:
            print(
                f"[Task6][Truth] t[:3]={t_truth[:3]} ... t[-3:]={t_truth[-3:]}"
            )
    except Exception as ex:
        print(f"[Task6][Truth][ERROR] failed to load truth: {ex}")
        t_truth = np.array([])
        pos_ecef_truth = vel_ecef_truth = np.zeros((0, 3))
        quat_truth = np.zeros((0, 4))

    # Load estimator file and time vector
    import scipy.io as sio
    if est_path.suffix.lower() == '.npz':
        est = {k: v for k, v in np.load(est_path, allow_pickle=True).items()}
    else:
        est = {k: v for k, v in sio.loadmat(est_path, squeeze_me=True).items() if not k.startswith('__')}
    print(f"[Task6][Est] keys={list(est.keys())}")
    t_est = _get_first_key(est, ['t_est', 'time_s', 't', 'time'])
    if t_est is None:
        dt = est.get('dt') or est.get('imu_dt') or 0.0
        n = None
        for key in ('pos_ned_m','pos_ecef_m','pos_body_m'):
            if key in est:
                n = len(est[key])
                break
        if n is not None and dt:
            t_est = np.arange(n) * float(dt)
    t_est = np.asarray(t_est, float).reshape(-1)
    if t_est.size:
        t_est = t_est - t_est[0]
    if t_est.size:
        print(
            f"[Task6][Est] len(t_est)={len(t_est)} t0={t_est[0]:.3f} t1={t_est[-1]:.3f} dt≈{np.median(np.diff(t_est)) if len(t_est)>1 else 0:.6f}"
        )
    # Normalise time to start at zero for consistent cross-task alignment
    t_rel = (t_est - t_est[0]) if t_est.size else t_est

    # --- Robust metadata lookup (avoid 'or' on arrays) ---
    ref_r0_m = _get_first_key(est, ['ref_r0_m', 'r0', 'r0_ecef_m'])
    ref_lat_rad = _get_first_key(est, ['ref_lat_rad', 'lat_rad', 'ref_lat'])
    ref_lon_rad = _get_first_key(est, ['ref_lon_rad', 'lon_rad', 'ref_lon'])

    # Coerce to proper types/shapes
    ref_r0_m = _as_vec3(ref_r0_m)

    # lat/lon may come as 0-d arrays; cast to float
    if ref_lat_rad is not None:
        ref_lat_rad = float(_np.asarray(ref_lat_rad).squeeze())
    if ref_lon_rad is not None:
        ref_lon_rad = float(_np.asarray(ref_lon_rad).squeeze())

    # Fallbacks if any missing: derive from GNSS first row
    if (ref_r0_m is None) or _np.any(~_np.isfinite(ref_r0_m)):
        # Load minimal GNSS ECEF columns
        import pandas as _pd
        gnss = _pd.read_csv(args.gnss_file)
        # Prefer standard names; otherwise take first 3 numeric ECEF-like cols
        cols = [c for c in gnss.columns if 'ECEF' in c.upper()]
        if len(cols) >= 3:
            Xc = [c for c in cols if 'X_ECEF' in c.upper()]
            Yc = [c for c in cols if 'Y_ECEF' in c.upper()]
            Zc = [c for c in cols if 'Z_ECEF' in c.upper()]
            if Xc and Yc and Zc:
                ref_r0_m = _np.array([gnss[Xc[0]][0], gnss[Yc[0]][0], gnss[Zc[0]][0]], dtype=float)
            else:
                ref_r0_m = _np.array(gnss[cols[:3]].iloc[0].to_list(), dtype=float)
        else:
            raise RuntimeError("Cannot derive ref_r0_m from GNSS; missing ECEF columns")

    # If lat/lon are still None, estimate from ECEF
    if (ref_lat_rad is None) or (ref_lon_rad is None):
        # Convert ref_r0_m ECEF -> lat/lon (WGS84)
        x, y, z = ref_r0_m
        # quick geodetic from ECEF (sufficient for plotting)
        a = 6378137.0; e2 = 6.69437999014e-3
        lon = _np.arctan2(y, x)
        p = _np.hypot(x, y)
        lat = _np.arctan2(z, p * (1 - e2))
        for _ in range(5):  # iteratively refine
            sinlat = _np.sin(lat)
            N = a / _np.sqrt(1 - e2 * sinlat*sinlat)
            lat = _np.arctan2(z + e2 * N * sinlat, p)
        ref_lat_rad = float(lat)
        ref_lon_rad = float(lon)

    # (Optional) helpful debug
    print(f"[Task6][Est] keys={list(est.keys())}")
    try:
        print(f"[Task6][Meta] ref_lat_rad={ref_lat_rad:.6f} ref_lon_rad={ref_lon_rad:.6f} ref_r0_m={ref_r0_m}")
    except Exception:
        print(f"[Task6][Meta] ref_lat_rad={ref_lat_rad} ref_lon_rad={ref_lon_rad} ref_r0_m={ref_r0_m}")

    # Interpolate truth into estimator timebase
    if t_truth.size and t_est.size:
        # Prefer truth velocity columns when non-trivial; fallback to smoothed derivative of position
        pos_ned_truth_10hz = ecef_to_ned(pos_ecef_truth, ref_lat_rad, ref_lon_rad, ref_r0_m)
        def _robust_p95(v):
            s = np.linalg.norm(v, axis=1)
            s = s[np.isfinite(s)]
            return float(np.percentile(s, 95)) if s.size else 0.0
        use_cols = False
        if vel_ecef_truth is not None and np.all(np.isfinite(vel_ecef_truth)):
            p95_cols = _robust_p95(vel_ecef_truth)
            if p95_cols > 0.10:  # > 10 cm/s treated as informative
                use_cols = True
        if use_cols:
            vel_ecef_src = vel_ecef_truth
            src = 'columns'
        else:
            t_truth = t_truth.astype(float)
            dt_truth = np.gradient(t_truth)
            v_raw = np.gradient(pos_ecef_truth, axis=0) / dt_truth[:, None]
            try:
                from scipy.signal import savgol_filter
                # ~5 s window for typical 10 Hz logs; ensure odd length
                win = 51 if (len(t_truth) >= 51) else max(5, (len(t_truth)//2)*2 + 1)
                vel_ecef_src = savgol_filter(v_raw, win, 3, axis=0, mode='interp')
                src = 'd/dt(position) (Savitzky–Golay)'
            except Exception:
                k = 21 if len(t_truth) >= 21 else max(5, (len(t_truth)//2)*2 + 1)
                kern = np.ones(k) / k
                vel_ecef_src = np.vstack([
                    np.convolve(v_raw[:, i], kern, mode='same') for i in range(3)
                ]).T
                src = 'd/dt(position) (moving-average)'
        vel_ned_truth_10hz = ecef_to_ned_vec(vel_ecef_src, ref_lat_rad, ref_lon_rad)
        interp = lambda X: np.column_stack([
            interp1d(t_truth, X[:, i], bounds_error=False, fill_value="extrapolate")(t_est) for i in range(3)
        ])
        pos_ned_truth_interp = interp(pos_ned_truth_10hz)
        _vel_tmp = interp(vel_ned_truth_10hz)
        try:
            p95 = float(np.percentile(np.linalg.norm(_vel_tmp, axis=1), 95))
            print(f"[Task6][Truth] velocity source: {src}; interp. p95(|v|)={p95:.3f} m/s")
        except Exception:
            pass
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

    # Helper for extracting vectors from estimator
    def _pick(store, keys):
        for k in keys:
            if k in store and store[k] is not None:
                arr = np.asarray(store[k]).squeeze()
                if arr.ndim == 1 and arr.size == 3:
                    arr = np.tile(arr, (len(t_est), 1))
                if arr.ndim == 2 and arr.shape[0] >= len(t_est) and arr.shape[1] == 3:
                    return arr[:len(t_est)].astype(float)
                if arr.ndim == 2 and arr.shape[1] >= len(t_est) and arr.shape[0] == 3:
                    return arr.T[:len(t_est)].astype(float)
        return None

    def _plot_2x3(time_s, pos_est, vel_est, pos_tru=None, vel_tru=None, title='', outfile=None):
        comps = ['X','Y','Z']
        # Decimation
        n = len(time_s)
        stride = 1 if args.no_decimate else max(1, int(np.ceil(n / max(1, args.decimate_maxpoints))))
        t_plot = time_s[::stride]
        pe = None if pos_est is None else pos_est[::stride]
        ve = None if vel_est is None else vel_est[::stride]
        pt = None if pos_tru is None else pos_tru[::stride]
        vt = None if vel_tru is None else vel_tru[::stride]
        fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
        # Top row: Position
        for i in range(3):
            ax = axes[0, i]
            if pt is not None:
                ax.plot(t_plot, pt[:, i], '--', label='Truth' if i == 0 else None, alpha=0.9, linewidth=1.0)
            if pe is not None:
                ax.plot(t_plot, pe[:, i], label='Estimate' if i == 0 else None, alpha=0.95, linewidth=1.3)
            ax.set_ylabel('Position [m]')
            ax.set_title(f'{title} {comps[i]}')
            ax.grid(alpha=0.3)
        # Bottom row: Velocity
        for i in range(3):
            ax = axes[1, i]
            if vt is not None:
                ax.plot(t_plot, vt[:, i], '--', label='Truth' if i == 0 else None, alpha=0.9, linewidth=1.0)
            if ve is not None:
                ax.plot(t_plot, ve[:, i], label='Estimate' if i == 0 else None, alpha=0.95, linewidth=1.3)
            ax.set_ylabel('Velocity [m/s]')
            ax.set_xlabel('Time [s]')
            ax.grid(alpha=0.3)
        # Harmonise y-limits using robust symmetric limits
        try:
            perc = float(args.ylim_percentile)
            if perc > 0 and perc <= 100:
                rows = []
                if pe is not None:
                    rows.append(np.abs(pe))
                if pt is not None:
                    rows.append(np.abs(pt))
                if rows:
                    lim_p = float(np.percentile(np.concatenate(rows, axis=0), perc))
                    if np.isfinite(lim_p) and lim_p > 0:
                        for i in range(3):
                            axes[0, i].set_ylim(-lim_p, lim_p)
                rows = []
                if ve is not None:
                    rows.append(np.abs(ve))
                if vt is not None:
                    rows.append(np.abs(vt))
                if rows:
                    lim_v = float(np.percentile(np.concatenate(rows, axis=0), perc))
                    if np.isfinite(lim_v) and lim_v > 0:
                        for i in range(3):
                            axes[1, i].set_ylim(-lim_v, lim_v)
        except Exception:
            pass
        # Single, centered legend covering both rows
        handles, labels = axes[0, 0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labels, ncol=2, loc='upper center')
        fig.suptitle(f'Task 6 Overlay ({title})')
        fig.tight_layout(rect=[0, 0, 1, 0.94])
        if outfile:
            try:
                # Save PNG and PDF directly from Matplotlib
                fig.savefig(outfile, dpi=220, bbox_inches='tight')
                pdf = str(Path(outfile).with_suffix('.pdf'))
                fig.savefig(pdf, dpi=300, bbox_inches='tight')
                print(f"[PNG] {outfile}")
                print(f"[PDF] {pdf}")
            except Exception as e:
                eprint(f"WARN: failed to save raster/vector outputs for {outfile}: {e}")
            # Also save MATLAB-compatible .mat with full-resolution series
            try:
                from scipy.io import savemat  # type: ignore
                base = Path(outfile).with_suffix('')
                payload = {
                    'time': np.asarray(time_s, dtype=float),
                    'pos_est': None if pos_est is None else np.asarray(pos_est, dtype=float),
                    'vel_est': None if vel_est is None else np.asarray(vel_est, dtype=float),
                    'pos_truth': None if pos_tru is None else np.asarray(pos_tru, dtype=float),
                    'vel_truth': None if vel_tru is None else np.asarray(vel_tru, dtype=float),
                    'frame': str(title),
                }
                # Remove None entries to keep .mat tidy
                payload_clean = {k: v for k, v in payload.items() if v is not None}
                savemat(str(base.with_suffix('.mat')), {'data': payload_clean})
                print(f"[MAT] {base.with_suffix('.mat')}")
            except Exception as e:
                eprint(f"WARN: failed to save .mat for {outfile}: {e}")
            try:
                # Save MATLAB .fig (plus PNG via helper) when engine is available
                from utils.matlab_fig_export import save_matlab_fig, validate_fig_openable
                base = str(Path(outfile).with_suffix(''))
                fig_path = save_matlab_fig(fig, base)
                if fig_path:
                    validate_fig_openable(str(fig_path))
            except Exception as e:
                eprint(f"WARN: .fig export/validation skipped for {outfile}: {e}")
        plt.close(fig)

    def _plot_quat(time_s, quat_est, quat_tru=None, outfile=None):
        comps = ['w', 'x', 'y', 'z']
        # Decimation
        n = len(time_s)
        stride = 1 if args.no_decimate else max(1, int(np.ceil(n / max(1, args.decimate_maxpoints))))
        t_plot = time_s[::stride]
        qe = None if quat_est is None else quat_est[::stride]
        qt = None if quat_tru is None else quat_tru[::stride]
        fig, axes = plt.subplots(4, 1, figsize=(10, 8), sharex=True)
        for i, comp in enumerate(comps):
            ax = axes[i]
            if qt is not None:
                ax.plot(t_plot, qt[:, i], label='truth', alpha=0.8)
            if qe is not None:
                ax.plot(t_plot, qe[:, i], label='estimate', alpha=0.8)
            ax.set_ylabel(comp)
            if i == 0:
                ax.legend(loc='upper right')
        axes[-1].set_xlabel('Time [s]')
        fig.suptitle('Task 6 Quaternion Overlay', y=1.02)
        fig.tight_layout()
        if outfile:
            try:
                fig.savefig(outfile, dpi=200, bbox_inches='tight')
                print(f"[PNG] {outfile}")
            except Exception as e:
                eprint(f"WARN: could not save quaternion PNG {outfile}: {e}")
        plt.close(fig)

    import matplotlib.pyplot as plt

    def _plot_diff_2x3(time_s, diff_pos, diff_vel, labels, frame, out_base):
        # Decimation
        n = len(time_s)
        stride = 1 if args.no_decimate else max(1, int(np.ceil(n / max(1, args.decimate_maxpoints))))
        t_plot = time_s[::stride]
        dp = diff_pos[::stride]
        dv = diff_vel[::stride]
        fig, axes = plt.subplots(2, 3, figsize=(9, 4), sharex=True)
        for i in range(3):
            axes[0, i].plot(t_plot, dp[:, i])
            axes[0, i].set_title(labels[i])
            axes[0, i].set_ylabel('Difference [m]')
            axes[0, i].grid(True)

            axes[1, i].plot(t_plot, dv[:, i])
            axes[1, i].set_xlabel('Time [s]')
            axes[1, i].set_ylabel('Difference [m/s]')
            axes[1, i].grid(True)
        # Harmonise y-limits
        try:
            perc = float(args.ylim_percentile)
            if perc > 0 and perc <= 100:
                lim_p = float(np.percentile(np.abs(dp), perc))
                lim_v = float(np.percentile(np.abs(dv), perc))
                if np.isfinite(lim_p) and lim_p > 0:
                    for i in range(3):
                        axes[0, i].set_ylim(-lim_p, lim_p)
                if np.isfinite(lim_v) and lim_v > 0:
                    for i in range(3):
                        axes[1, i].set_ylim(-lim_v, lim_v)
        except Exception:
            pass
        fig.suptitle(f'Truth - Fused Differences ({frame} Frame)')
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        png = Path(out_base).with_suffix('.png')
        pdf = Path(out_base).with_suffix('.pdf')
        try:
            fig.savefig(png, dpi=200, bbox_inches='tight')
            print(f"[PNG] {png}")
            fig.savefig(pdf, dpi=200, bbox_inches='tight')
            print(f"[PDF] {pdf}")
        except Exception as e:
            eprint(f"WARN: could not save diff plots for {frame}: {e}")
        plt.close(fig)

    # Single-method overlays (acceleration OFF by default per requirements)
    saved = {}
    if not args.include_accel:
        # Build NED/ECEF/BODY overlays with only pos/vel
        run_id = run_id or derive_run_id(est_path)
        # Time base already in t_est
        # Estimate vectors
        # Prefer fused outputs for NED to match Task 7
        pos_ned = _pick(est, ['fused_pos', 'pos_ned_m', 'fused_pos_ned', 'pos_ned'])
        vel_ned = _pick(est, ['fused_vel', 'vel_ned_ms', 'fused_vel_ned', 'vel_ned'])
        pos_ecef_est = _pick(est, ['pos_ecef_m', 'fused_pos_ecef', 'pos_ecef'])
        vel_ecef_est = _pick(est, ['vel_ecef_ms', 'fused_vel_ecef', 'vel_ecef'])
        pos_body = _pick(est, ['pos_body_m', 'fused_pos_body', 'pos_body'])
        vel_body = _pick(est, ['vel_body_ms', 'fused_vel_body', 'vel_body'])

        # Derive ECEF from NED if needed
        if pos_ecef_est is None and pos_ned is not None:
            pos_ecef_est = ned_to_ecef(pos_ned, ref_lat_rad, ref_lon_rad, ref_r0_m)
        if vel_ecef_est is None and vel_ned is not None:
            # vector transform only
            Re2n = np.array([[ -np.sin(ref_lat_rad)*np.cos(ref_lon_rad), -np.sin(ref_lat_rad)*np.sin(ref_lon_rad),  np.cos(ref_lat_rad)],
                             [                 -np.sin(ref_lon_rad),                  np.cos(ref_lon_rad),               0.0],
                             [ -np.cos(ref_lat_rad)*np.cos(ref_lon_rad), -np.cos(ref_lat_rad)*np.sin(ref_lon_rad), -np.sin(ref_lat_rad)]])
            vel_ecef_est = (Re2n.T @ vel_ned.T).T

        # Interpolate truth to estimator time base
        pos_ned_truth = vel_ned_truth = None
        pos_ecef_truth_i = vel_ecef_truth_i = None
        pos_body_truth_i = vel_body_truth_i = None
        quat_truth_i = None
        quat_est = None
        if t_truth.size and t_est.size:
            # Zero-base truth time for consistent interpolation
            t_truth = t_truth - t_truth[0]
            if pos_ecef_truth.size:
                pos_ned_truth_10hz = ecef_to_ned(pos_ecef_truth, ref_lat_rad, ref_lon_rad, ref_r0_m)
                vel_ecef_from_pos_10hz = np.gradient(pos_ecef_truth, t_truth, axis=0)
                vel_ned_truth_10hz = ecef_to_ned_vec(vel_ecef_from_pos_10hz, ref_lat_rad, ref_lon_rad)
                # interp to t_est
                def _interp(X):
                    return np.column_stack([
                        interp1d(t_truth, X[:, i], bounds_error=False, fill_value="extrapolate")(t_est) for i in range(3)
                    ])
                pos_ned_truth = _interp(pos_ned_truth_10hz)
                vel_ned_truth = _interp(vel_ned_truth_10hz)
                pos_ecef_truth_i = _interp(pos_ecef_truth)
                # Provide derived ECEF velocity for ECEF overlay
                vel_ecef_truth_i = _interp(vel_ecef_from_pos_10hz)
                # BODY truth: use quaternion when available; otherwise mirror NED
                pos_body_truth_i = pos_ned_truth
                vel_body_truth_i = vel_ned_truth
                if quat_truth.size:
                    r_truth = Rotation.from_quat(
                        np.column_stack([quat_truth[:, 1:4], quat_truth[:, 0]])
                    )
                    slerp = Slerp(t_truth, r_truth)
                    r_i = slerp(np.clip(t_est, t_truth[0], t_truth[-1]))
                    q_xyzw = r_i.as_quat()
                    quat_truth_i = np.column_stack([q_xyzw[:, 3], q_xyzw[:, :3]])
                    # Rotate NED -> BODY using inverse of body->NED quaternion
                    R_b2n = r_i.as_matrix()  # shape (N,3,3)
                    R_n2b = np.transpose(R_b2n, (0, 2, 1))
                    if pos_ned_truth is not None:
                        pos_body_truth_i = np.einsum('nij,nj->ni', R_n2b, pos_ned_truth)
                    if vel_ned_truth is not None:
                        vel_body_truth_i = np.einsum('nij,nj->ni', R_n2b, vel_ned_truth)

        # Extract estimator quaternion if present
        for k in ['att_quat', 'quat', 'quaternion']:
            if k in est and est[k] is not None:
                q = np.asarray(est[k]).squeeze()
                if q.ndim == 2 and q.shape[1] == 4:
                    quat_est = q[: len(t_est)].astype(float)
                elif q.ndim == 2 and q.shape[0] == 4:
                    quat_est = q.T[: len(t_est)].astype(float)
                break
        if quat_est is not None and quat_truth_i is not None:
            # Hemisphere alignment
            dots = np.sum(quat_est * quat_truth_i, axis=1)
            quat_est[dots < 0] *= -1

        # Save 2x3 grids
        # NED
        out_ned = out_dir / f"{run_id}_task6_overlay_NED.png"
        _plot_2x3(t_rel, pos_ned, vel_ned, pos_ned_truth, vel_ned_truth, 'NED', str(out_ned))
        # ECEF
        out_ecef = out_dir / f"{run_id}_task6_overlay_ECEF.png"
        _plot_2x3(t_rel, pos_ecef_est, vel_ecef_est, pos_ecef_truth_i, vel_ecef_truth_i, 'ECEF', str(out_ecef))
        # BODY (include truth mirrored from NED for consistent overlay)
        out_body = out_dir / f"{run_id}_task6_overlay_BODY.png"
        _plot_2x3(t_rel, pos_body, vel_body, pos_body_truth_i, vel_body_truth_i, 'BODY', str(out_body))

        # Quaternion overlay
        if quat_est is not None and quat_truth_i is not None:
            out_quat = out_dir / f"{run_id}_task6_overlay_QUAT.png"
            _plot_quat(t_rel, quat_est, quat_truth_i, str(out_quat))
            print(f"[Task6] Saved quaternion overlay: {out_quat}")

        print(f"[Task6] Saved overlays (2x3 pos/vel only): {[out_ned, out_ecef, out_body]}")

        # Also copy flat into PYTHON/results root for convenience
        try:
            res_root = ensure_results_dir()
            for p in [out_ned, out_ecef, out_body]:
                dst = res_root / p.name
                shutil.copy2(p, dst)
                print(f"[SAVE] Copied {p.name} -> {dst}")
        except Exception as ex:
            eprint(f"WARN: flat results copy failed: {ex}")
        # Difference plots (Truth - Fused) in all frames
        try:
            diff_pngs = []
            if pos_ned is not None and vel_ned is not None and pos_ned_truth is not None and vel_ned_truth is not None:
                diff_pos_ned = pos_ned_truth - pos_ned
                diff_vel_ned = vel_ned_truth - vel_ned
                p = out_dir / f"{run_id}_task6_diff_truth_fused_over_time_NED"
                _plot_diff_2x3(t_rel, diff_pos_ned, diff_vel_ned, ['North','East','Down'], 'NED', str(p))
                diff_pngs.append(p.with_suffix('.png'))
            if pos_ecef_est is not None and vel_ecef_est is not None and pos_ecef_truth_i is not None and vel_ecef_truth_i is not None:
                diff_pos_ecef = pos_ecef_truth_i - pos_ecef_est
                diff_vel_ecef = vel_ecef_truth_i - vel_ecef_est
                p = out_dir / f"{run_id}_task6_diff_truth_fused_over_time_ECEF"
                _plot_diff_2x3(t_rel, diff_pos_ecef, diff_vel_ecef, ['X','Y','Z'], 'ECEF', str(p))
                diff_pngs.append(p.with_suffix('.png'))
            if pos_body is not None and vel_body is not None and pos_body_truth_i is not None and vel_body_truth_i is not None:
                diff_pos_body = pos_body_truth_i - pos_body
                diff_vel_body = vel_body_truth_i - vel_body
                p = out_dir / f"{run_id}_task6_diff_truth_fused_over_time_Body"
                _plot_diff_2x3(t_rel, diff_pos_body, diff_vel_body, ['X','Y','Z'], 'Body', str(p))
                diff_pngs.append(p.with_suffix('.png'))
            # Copy diff PNGs to results root for convenience
            try:
                res_root = ensure_results_dir()
                for p in diff_pngs:
                    if p.exists():
                        dst = res_root / p.name
                        shutil.copy2(p, dst)
                        print(f"[SAVE] Copied {p.name} -> {dst}")
            except Exception as ex:
                eprint(f"WARN: flat diff results copy failed: {ex}")
        except Exception as ex:
            eprint(f"WARN: failed to create Task 6 diff plots: {ex}")
    else:
        try:
            saved_single = run_task6_overlay_all_frames(
                est_file=str(est_path),
                truth_file=truth_path,
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

    if method_files and args.include_accel:
        try:
            saved_multi = run_task6_compare_methods_all_frames(
                method_files=method_files,
                truth_file=truth_path,
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
