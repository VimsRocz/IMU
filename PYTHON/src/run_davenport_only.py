#!/usr/bin/env python3
"""Run the Davenport initialisation on a single fixed IMU/GNSS dataset.

This script mirrors :mod:`run_triad_only.py` but sets the method to
``Davenport``. It processes the default ``IMU_X002.dat`` with
``GNSS_X002.csv`` and runs the full pipeline (Tasks 1–7), producing the
same prints/logs and artifacts layout, now under
``results/IMU_X002_GNSS_X002_Davenport``. Use this helper to compare the
Python and MATLAB pipelines on a single dataset before running the full
batch.

Usage
-----
    python src/run_davenport_only.py [options]
"""

from __future__ import annotations

# Self-contained imports: make local src importable from any CWD
from pathlib import Path as _Path
import sys as _sys
_SRC = _Path(__file__).resolve().parent
if str(_SRC) not in _sys.path:
    _sys.path.insert(0, str(_SRC))
# Repository root (…/IMU)
REPO_ROOT = _SRC.parents[2]

import argparse
import json
import logging
import math
import os
import pathlib
import re
import subprocess
import sys
import time
import shutil
import io
from contextlib import redirect_stdout
from pathlib import Path
from typing import Iterable, List, Dict

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import scipy.io as sio
from tabulate import tabulate
import matplotlib.pyplot as plt
from scipy.io import loadmat, savemat

from evaluate_filter_results import run_evaluation_npz
from run_all_methods import run_case, compute_C_NED_to_ECEF
from utils import save_mat

# Import helper utilities from the utils package
from utils.timeline import print_timeline
from utils.resolve_truth_path import resolve_truth_path
from utils.io_checks import assert_single_pair
from utils.run_id import run_id as build_run_id

sys.path.append(str(Path(__file__).resolve().parents[1] / "tools"))

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
from paths import (
    imu_path as _imu_path_helper,
    gnss_path as _gnss_path_helper,
    truth_path as _truth_path_helper,
    ensure_results_dir as _ensure_results,
    python_results_dir as _py_results_dir,
)

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def check_files(
    imu_file: str, gnss_file: str
) -> tuple[pathlib.Path, pathlib.Path]:
    """Return validated paths for the IMU and GNSS files."""
    logger.info("Checking files: imu=%s gnss=%s", imu_file, gnss_file)
    imu_path = pathlib.Path(imu_file)
    gnss_path = pathlib.Path(gnss_file)
    logger.info("Validated files exist: imu=%s gnss=%s", imu_path, gnss_path)
    return imu_path, gnss_path


def _unwrap_clock_1s(t_raw, wrap=1.0, tol=0.25):
    """Unwrap a clock that resets every ``wrap`` seconds (default 1s)."""
    import numpy as np
    logger.info("Unwrapping clock with wrap=%s tol=%s", wrap, tol)
    t = np.asarray(t_raw, dtype=float).ravel()
    if t.size == 0:
        logger.info("No timestamps provided; returning empty array")
        return t, 0
    out = t.copy()
    wraps = 0
    offset = 0.0
    for i in range(1, t.size):
        if t[i] + 1e-12 < t[i - 1] - tol:
            wraps += 1
            offset += wrap
        out[i] = t[i] + offset
    logger.info("Unwrapped clock applied %d wraps", wraps)
    return out, wraps


def _make_monotonic_time(t_like, fallback_len=None, dt=None, imu_rate_hint=None):
    """Build a strictly increasing IMU timebase."""
    import numpy as np
    logger.info(
        "Building monotonic time: len=%s dt=%s imu_rate_hint=%s",
        len(t_like) if hasattr(t_like, "__len__") else None,
        dt,
        imu_rate_hint,
    )
    if t_like is None or (hasattr(t_like, "__len__") and len(t_like) == 0):
        if dt is None:
            if imu_rate_hint and imu_rate_hint > 0:
                dt = 1.0 / float(imu_rate_hint)
            else:
                raise ValueError("No IMU timestamps and no dt/imu_rate_hint provided.")
        n = int(fallback_len) if fallback_len is not None else 0
        t = np.arange(n, dtype=float) * float(dt)
        meta = {"source": "synth", "wraps": 0, "dt": dt}
        logger.info("Generated synthetic timebase of %d samples", n)
        return t, meta

    t = np.asarray(t_like, dtype=float).ravel()
    meta = {"source": "file", "wraps": 0}
    span = float(t.max() - t.min()) if t.size else 0.0
    if span < 2.0 and t.size > 2000:
        t, wraps = _unwrap_clock_1s(t, wrap=1.0, tol=0.25)
        meta["wraps"] = int(wraps)
        meta["unwrapped_span"] = float(t[-1] - t[0]) if t.size else 0.0
        print(
            f"[Clock] Detected 1s-resetting IMU clock; unwrapped with {wraps} wraps -> {meta['unwrapped_span']:.2f}s total."
        )
    d = np.diff(t)
    if (d <= 0).any():
        eps = np.finfo(float).eps
        t = t + np.arange(t.size) * eps
        meta["jitter_eps_applied"] = True
    logger.info("Monotonic timebase built with %d samples", t.size)
    return t, meta


def _write_run_meta(outdir, run_id, **kv):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    meta_path = outdir / f"{run_id}_runmeta.json"
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(kv, f, indent=2, sort_keys=True)
    logger.info("Saved run meta -> %s", meta_path)


# ----------------------------
# Inline validation helpers
# ----------------------------
def _norm_quat(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def _fix_hemisphere(qt, qe):
    dot = np.sum(qt * qe, axis=1)
    s = np.sign(dot)
    s[s == 0] = 1.0
    return qe * s[:, None]


def _quat_angle_deg(qt, qe):
    d = np.clip(np.abs(np.sum(qt * qe, axis=1)), 0.0, 1.0)
    return 2.0 * np.degrees(np.arccos(d))


def _quat_to_euler_zyx_deg(q):  # yaw, pitch, roll from [w,x,y,z]
    w, x, y, z = q.T
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    s = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(s))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return np.vstack([yaw, pitch, roll]).T


def _overlap(t1, t2):
    t0 = max(t1[0], t2[0])
    t1e = min(t1[-1], t2[-1])
    if t1e <= t0:
        raise RuntimeError("No overlapping time between truth and estimate.")
    return t0, t1e


def _interp_columns_to(t_src, Y_src, t_dst):
    Y_src = np.asarray(Y_src)
    out = np.empty((t_dst.size, Y_src.shape[1]), float)
    for j in range(Y_src.shape[1]):
        out[:, j] = np.interp(t_dst, t_src, Y_src[:, j])
    return out


# Robust overlap and interpolation helpers for inline validation
def _uniq_sorted_time(t, *arrays):
    """Sort by time, drop duplicates, and apply the same indexing to companions."""
    t = np.asarray(t, dtype=float)
    order = np.argsort(t, kind="mergesort")
    t_sorted = t[order]
    uniq_mask = np.ones_like(t_sorted, dtype=bool)
    uniq_mask[1:] = np.diff(t_sorted) > 0
    keep = order[uniq_mask]
    out = [t[keep]]
    for arr in arrays:
        out.append(np.asarray(arr)[keep])
    return out if len(out) > 1 else out[0]


def _compute_overlap(t_est, t_truth, *truth_arrays):
    """Return overlapped (t_truth_sub, *truth_arrays_sub) aligned to [t_est.min, t_est.max]."""
    if t_est.size == 0 or t_truth.size == 0:
        raise RuntimeError("Empty time arrays passed to _compute_overlap.")
    t0, t1 = float(np.nanmin(t_est)), float(np.nanmax(t_est))
    # clean & sort truth; drop duplicate times
    t_truth_clean, *truth_arrays_clean = _uniq_sorted_time(t_truth, *truth_arrays)
    finite_mask = np.isfinite(t_truth_clean)
    for arr in truth_arrays_clean:
        finite_mask &= np.all(np.isfinite(arr), axis=1) if arr.ndim == 2 else np.isfinite(arr)
    t_truth_clean = t_truth_clean[finite_mask]
    truth_arrays_clean = [arr[finite_mask] for arr in truth_arrays_clean]
    # window to est range
    if t_truth_clean.size == 0:
        return (np.array([]),) + tuple(np.array([]) for _ in truth_arrays_clean)
    mask = (t_truth_clean >= t0) & (t_truth_clean <= t1)
    t_sub = t_truth_clean[mask]
    arrays_sub = [arr[mask] for arr in truth_arrays_clean]
    return (t_sub,) + tuple(arrays_sub)


def _interp_quat_components(t_src, q_src, t_query):
    """
    Component-wise linear interp of quaternions w/o SLERP (OK for diagnostics).
    Returns (N,4) or raises with a clear message if inputs are unusable.
    """
    if t_src.size < 2:
        raise ValueError(
            f"Not enough truth samples to interpolate: len(t_src)={t_src.size}. "
            "Check overlap window and NaNs."
        )
    # ensure strictly increasing
    order = np.argsort(t_src, kind="mergesort")
    t_src = t_src[order]
    q_src = q_src[order]
    # clamp query to src bounds to avoid extrapolation errors
    tq = np.clip(t_query, t_src[0], t_src[-1])
    from scipy.interpolate import interp1d
    q_interp = np.empty((tq.size, 4), dtype=float)
    for k in range(4):
        f = interp1d(
            t_src,
            q_src[:, k],
            kind="linear",
            bounds_error=False,
            fill_value=(q_src[0, k], q_src[-1, k]),
            assume_sorted=True,
        )
        q_interp[:, k] = f(tq)
    # renormalize
    norm = np.linalg.norm(q_interp, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    return q_interp / norm


def _divergence_time(t, err_deg, thr_deg, persist_s):
    above = err_deg > thr_deg
    if not np.any(above):
        return np.nan
    dt = np.median(np.diff(t))
    need = max(1, int(round(persist_s / dt)))
    i = 0
    N = above.size
    while i < N:
        if above[i]:
            j = i
            while j < N and above[j]:
                j += 1
            if (j - i) >= need:
                return t[i]
            i = j
        else:
            i += 1
    return np.nan


def _read_state_x001(truth_file):
    """
    Read STATE_X001-like truth with flexible column handling.

    Expected columns include (typical order):
      [idx?] time_s  x_ecef  y_ecef  z_ecef  vx_ecef  vy_ecef  vz_ecef  qw  qx  qy  qz

    This function:
      - loads all numeric columns
      - auto-selects the best time column
      - zero-bases time if it looks offset (e.g., starts at ~10000 s)
      - returns (t, pos_ecef, vel_ecef, quat) with shapes:
          t:(N,), pos:(N,3), vel:(N,3), quat:(N,4) (qw,qx,qy,qz)
    """
    import numpy as np

    M = np.loadtxt(truth_file, dtype=float)
    if M.ndim != 2 or M.shape[1] < 10:
        raise ValueError(f"Unexpected truth shape {M.shape} for {truth_file}")

    # Heuristics to find quaternion columns: last 4 numeric columns
    quat = M[:, -4:]  # assume (qw,qx,qy,qz)

    # Candidate time columns: prefer one whose range overlaps small window (seconds)
    # Common layouts: time at col 0 or col 1
    candidates = []
    for c in range(min(3, M.shape[1])):  # probe first up to 3 cols
        t_try = M[:, c].astype(float)
        if np.all(np.isfinite(t_try)):
            span = float(np.nanmax(t_try) - np.nanmin(t_try))
            candidates.append((c, span, t_try))
    if not candidates:
        raise ValueError("No finite time-like column found in truth file.")

    # Choose the candidate with the smallest positive span >= 1.0 sec (prefers [0..~2000] over [10000..20086])
    candidates = [(c, s, tcol) for (c, s, tcol) in candidates if s > 0]
    candidates.sort(key=lambda x: x[1])
    c_best, span_best, t_raw = candidates[0]

    # Zero-base time if it appears offset (e.g., starts >> 0 and span looks reasonable)
    t0 = float(np.nanmin(t_raw))
    t = t_raw - t0 if t0 > 1.0 else t_raw

    # Now pick positions/velocities assuming standard layout: find 6 columns before quats
    # If your format differs, adjust here.
    six_before_quat = M.shape[1] - 4 - 6
    if six_before_quat < 0:
        raise ValueError("Not enough columns to extract pos/vel before quaternions.")
    pv = M[:, six_before_quat:six_before_quat+6]
    pos_ecef = pv[:, :3]
    vel_ecef = pv[:, 3:6]

    # Simple cleaning: drop rows with any NaNs in pos/vel/quats
    good = np.all(np.isfinite(pos_ecef), axis=1) & np.all(np.isfinite(vel_ecef), axis=1) & np.all(np.isfinite(quat), axis=1)
    t = t[good]
    pos_ecef = pos_ecef[good]
    vel_ecef = vel_ecef[good]
    quat = quat[good]

    return t, pos_ecef, vel_ecef, quat


def _heuristic_truth_frame_from_residuals(t_est, q_est, t_truth, q_truth_ned_candidate, pos_ecef_tru, results_dir, tag):
    """Heuristically decide if the truth quaternion is body->NED or body->ECEF.

    Compare the angle error vs KF attitude in both interpretations and pick the lower-median one.
    Saves a small text file with the detected frame for reproducibility.
    Returns ``q_truth_as_ned, q_est_aligned, angle_error_deg, detected_frame``.
    """
    q_est = _norm_quat(q_est)
    q_truth_on_est_raw = _norm_quat(q_truth_ned_candidate)

    # Try both: assume given truth is Body->NED, or interpret it as Body->ECEF then convert
    detected_frame = "NED"
    q_truth_ned = q_truth_on_est_raw.copy()
    try:
        from utils import ecef_to_geodetic, compute_C_ECEF_to_NED
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_ecef_tru[0])
        C_e2n = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
        from scipy.spatial.transform import Rotation as _R
        R_bn_list = []
        for q in q_truth_on_est_raw:
            r_be = _R.from_quat([q[1], q[2], q[3], q[0]])  # [x,y,z,w]
            R_bn = C_e2n @ r_be.as_matrix()
            R_bn_list.append(_R.from_matrix(R_bn).as_quat())  # [x,y,z,w]
        q_truth_from_ecef = np.array([[q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]] for q_xyzw in R_bn_list])
        q_truth_from_ecef = _norm_quat(q_truth_from_ecef)
        q_truth_ned = _norm_quat(q_truth_ned)
        q_est_aligned_for_ned = _fix_hemisphere(q_truth_ned, q_est)
        q_est_aligned_for_ecef = _fix_hemisphere(q_truth_from_ecef, q_est)
        ang_ned = _quat_angle_deg(q_truth_ned, q_est_aligned_for_ned)
        ang_ecef = _quat_angle_deg(q_truth_from_ecef, q_est_aligned_for_ecef)
        med_ned = float(np.nanmedian(ang_ned))
        med_ecef = float(np.nanmedian(ang_ecef))
        if np.isfinite(med_ecef) and (med_ecef + 1e-3) < med_ned:
            detected_frame = "ECEF"
            q_truth_ned = q_truth_from_ecef
            qE = q_est_aligned_for_ecef
            ang = ang_ecef
        else:
            detected_frame = "NED"
            qE = q_est_aligned_for_ned
            ang = ang_ned
        print(f"[Task7] Detected truth quaternion frame: {detected_frame}")
        det_path = os.path.join(results_dir, f"{tag}_Task7_truth_quaternion_frame.txt")
        with open(det_path, "w", encoding="utf-8") as fdet:
            fdet.write(detected_frame + "\n")
    except Exception as ex:
        print(f"[WARN] Truth frame detection failed ({ex}); assuming NED.")
        q_truth_ned = _norm_quat(q_truth_ned)
        qE = _fix_hemisphere(q_truth_ned, q_est)
        ang = _quat_angle_deg(q_truth_ned, qE)

    # Truth direction detection: Body->NED vs NED->Body (conjugate)
    try:
        q_truth_ned = _norm_quat(q_truth_ned)
        def _err0(qt):
            qe = _fix_hemisphere(qt, q_est)
            ang0 = _quat_angle_deg(qt, qe)
            return float(ang0[0]) if ang0.size else float('nan')
        e0_dir = _err0(q_truth_ned)
        e0_con = _err0(_q_conj(q_truth_ned))
        if np.isfinite(e0_con) and e0_con + 1e-3 < e0_dir:
            q_truth_ned = _q_conj(q_truth_ned)
            print(f"[Task7] Truth appears NED→Body; using conjugate. (err0 direct={e0_dir:.2f}°, conj={e0_con:.2f}°)")
        else:
            print(f"[Task7] Truth matches Body→NED. (err0 direct={e0_dir:.2f}°)")
    except Exception:
        pass

    return q_truth_ned, qE, ang, detected_frame


def _q_conj(q):
    q = np.asarray(q, float)
    return np.column_stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]])


def _task5_plot_quat_cases(results_dir, run_id, truth_file, method_name="Davenport"):
    """Compare METHOD-init KF vs TRUE-init KF quaternion components vs truth."""
    results_dir = Path(results_dir)
    est_npz = results_dir / f"{run_id}_kf_output.npz"
    est_true_npz = results_dir / f"{run_id}_true_init_kf_output.npz"
    if not (est_npz.exists() and est_true_npz.exists() and Path(truth_file).exists()):
        print(f"[Task5] Skipping {method_name} vs TRUE-init comparison (missing outputs).")
        return

    def load_npz(npz_path):
        npz = np.load(npz_path, allow_pickle=True)
        t = npz['t']
        q = npz['attitude_q']  # [w,x,y,z]
        return t, q

    t, qA = load_npz(est_npz)
    tB, qB = load_npz(est_true_npz)
    if t.size == 0 or tB.size == 0:
        return

    # Load truth quaternion and interpolate to the estimate timebase
    tT, pos_ecef_tru, vel_ecef_tru, qT_all = _read_state_x001(truth_file)
    if tT.size == 0:
        return
    qT = _interp_quat_components(tT, qT_all, t)

    # Heuristic: compare error if we treat provided truth quaternion as Body->NED
    # vs converting from Body->ECEF->Body->NED using truth position for frame.
    q_truth_ned, qE, ang, detected_frame = _heuristic_truth_frame_from_residuals(
        t, qA, tT, qT, pos_ecef_tru, str(results_dir), run_id
    )

    # Compute and report divergence time
    thr_deg = 30.0
    persist = 10.0
    t_div = _divergence_time(t, ang, thr_deg, persist)
    if np.isfinite(t_div):
        print(f"[Task5] {method_name} vs Truth attitude error sustained >{thr_deg:.0f}° for {persist:.0f}s starting at t={t_div:.1f}s")
    else:
        print(f"[Task5] {method_name} vs Truth shows no sustained divergence >{thr_deg:.0f}° over the run.")

    # Plot quaternion components
    plt.figure(figsize=(10, 8))
    labs = ["qw", "qx", "qy", "qz"]
    for i in range(4):
        ax = plt.subplot(4, 1, i + 1)
        ax.plot(t, qT[:, i], 'k-', label='Truth')
        ax.plot(t, qA[:, i], 'b--', label=f'KF ({method_name}-init)')
        ax.plot(t, qB[:, i], 'r-.', label='KF (TRUE-init)')
        ax.set_ylabel(labs[i])
        ax.grid(True)
        if i == 0:
            ax.legend(loc='best')
    plt.xlabel('Time [s]')
    plt.suptitle(f'Task 5: Quaternions — Truth vs KF ({method_name}-init vs TRUE-init)')
    out = Path(results_dir) / f"{run_id}_Task5_quat_cases_{method_name}init_vs_TRUEinit.png"
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    try:
        plt.gcf().savefig(out, dpi=200, bbox_inches='tight')
        print(f"[Task5] Saved {method_name} vs TRUE-init quaternion comparison -> {out}")
    except Exception as e:
        print(f"[Task5] Failed to save comparison plot: {e}")
    finally:
        plt.close()


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run Davenport (Task-1) on a selected dataset or run all methods",
        allow_abbrev=False,
    )
    parser.add_argument(
        "--dataset",
        choices=["X001", "X002", "X003"],
        default="X002",
        help="Dataset ID to use (default X002)",
    )
    parser.add_argument("--imu", type=str, help="Path to IMU data file")
    parser.add_argument("--gnss", type=str, help="Path to GNSS data file")
    parser.add_argument("--truth", type=str, help="Path to truth data file")
    parser.add_argument(
        "--allow-truth-mismatch",
        action="store_true",
        help="Allow using a truth file from a different dataset (use with care)",
    )
    parser.add_argument(
        "--outdir", type=str, help="Directory to write results (default PYTHON/results)"
    )
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    parser.add_argument(
        "--mode",
        choices=["all", "davenport"],
        default="davenport",
        help=(
            "Mode: run all methods across datasets (all) or Davenport-only (davenport). "
            "Default is davenport."
        ),
    )
    parser.add_argument(
        "--show-measurements",
        action="store_true",
        help="Include IMU and GNSS measurements in Task 6 overlay plots",
    )
    parser.add_argument(
        "--task",
        type=int,
        help="Run a single helper task and exit",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        "--debug",
        dest="debug",
        action="store_true",
        help="Enable verbose task-level logging",
    )
    parser.add_argument("--imu-rate", type=float, default=None, help="Hint IMU sample rate [Hz]")
    parser.add_argument("--gnss-rate", type=float, default=None, help="Hint GNSS sample rate [Hz]")
    parser.add_argument("--truth-rate", type=float, default=None, help="Hint truth sample rate [Hz]")
    parser.add_argument("--tasks", type=str, default=None, help="Comma-separated task numbers to run")
    parser.add_argument('--div-threshold-deg', type=float, default=30.0,
                        help='Attitude error threshold in degrees for divergence.')
    parser.add_argument('--div-persist-sec', type=float, default=10.0,
                        help='Seconds above threshold required to declare divergence.')
    parser.add_argument('--length-scan', type=str, default="60,120,300,600,900,1200",
                        help='Comma-separated end-times (s) for divergence-vs-length study.')
    # Attitude initialisation and yaw-fusion controls (passed to GNSS_IMU_Fusion)
    parser.add_argument('--init-att-with-truth', action='store_true',
                        help='Initialise the KF attitude with the truth quaternion at IMU t0')
    parser.add_argument('--truth-quat-frame', choices=['NED', 'ECEF'], default='NED',
                        help='Frame of the truth quaternion when initialising from truth')
    parser.add_argument('--fuse-yaw', action='store_true',
                        help='Fuse GNSS-derived yaw to correct quaternion drift')
    parser.add_argument('--yaw-gain', type=float, default=0.2,
                        help='Gain [0..1] per update for GNSS yaw fusion')
    parser.add_argument('--yaw-speed-min', type=float, default=2.0,
                        help='Min horizontal speed [m/s] to trust GNSS yaw')
    parser.add_argument('--true-init-second-pass', action='store_true',
                        help='Run a second KF pass with true attitude init and compare quaternions (Task 5).')

    args = parser.parse_args(argv)

    # Convenience: if requested to run everything, forward to run_all_methods.py
    if args.mode == "all":
        from pathlib import Path as _Path
        import subprocess as _subprocess
        HERE_ = _Path(__file__).resolve().parent
        cmd = [sys.executable, str(HERE_ / "run_all_methods.py")]
        if args.no_plots:
            cmd.append("--no-plots")
        # Forward dataset override if explicitly given
        if args.imu and args.gnss:
            cmd += ["--datasets", args.imu, args.gnss]
        print("Running all methods across datasets …")
        _subprocess.run(cmd, check=True)
        return

    # Resolve default dataset files if not explicitly provided
    dataset = args.dataset
    if args.imu and args.gnss:
        imu_path = Path(args.imu)
        gnss_path = Path(args.gnss)
    else:
        imu_path = _imu_path_helper(dataset)
        gnss_path = _gnss_path_helper(dataset)
    truth_path = Path(args.truth) if args.truth else _truth_path_helper()

    method = "Davenport"

    # Results directory setup
    results_dir = Path(args.outdir) if args.outdir else _py_results_dir()
    results_dir.mkdir(parents=True, exist_ok=True)

    # Build run ID and write meta
    run_id = build_run_id(str(imu_path), str(gnss_path), method)
    _write_run_meta(
        results_dir,
        run_id,
        dataset=dataset,
        imu=str(imu_path),
        gnss=str(gnss_path),
        truth=str(truth_path),
        method=method,
    )

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    print_timeline("Task 1–7 Davenport pipeline starting …")

    # ----------------------------
    # Task 1–5 via run_case
    # ----------------------------
    print(f"Running {method} with:")
    print(f"  IMU : {imu_path}")
    print(f"  GNSS: {gnss_path}")
    print(f"  Truth: {truth_path if truth_path else '(none)'}")

    t0 = time.time()
    case = run_case(
        imu_path=str(imu_path),
        gnss_path=str(gnss_path),
        method=method,
        outdir=str(results_dir),
        make_plots=not args.no_plots,
        return_artifacts=True,
        fuse_yaw=args.fuse_yaw,
        yaw_gain=args.yaw_gain,
        yaw_speed_min=args.yaw_speed_min,
        init_att_with_truth=args.init_att_with_truth,
        truth_quat_frame=args.truth_quat_frame,
        verbose=args.debug,
    )
    elapsed = time.time() - t0
    print(f"[SUMMARY] Task 1–5 completed in {elapsed:.1f}s (method={method}).")

    # Extract primary outputs
    est_mat = results_dir / f"{run_id}_kf_output.mat"
    est_npz = results_dir / f"{run_id}_kf_output.npz"
    true_npz = results_dir / f"{run_id}_true_init_kf_output.npz"

    # Optionally plot Task 5 comparison if second pass available
    try:
        if true_npz.exists() and truth_path.exists():
            _task5_plot_quat_cases(str(results_dir), run_id, str(truth_path), method_name=method)
    except Exception as ex:
        print(f"[Task5] Comparison plotting failed: {ex}")

    # ----------------------------
    # Task 6 overlay and Task 7 residuals
    # ----------------------------
    try:
        if truth_path and truth_path.exists():
            # Task 6
            cmd_t6 = [
                sys.executable,
                str(HERE / "task6_plot_truth.py"),
                "--est-file", str(est_mat if est_mat.exists() else est_npz),
                "--gnss-file", str(gnss_path),
                "--truth-file", str(truth_path),
                "--output", str(results_dir),
                "--decimate-maxpoints", "200000",
                "--ylim-percentile", "99.5",
            ]
            if args.show_measurements:
                cmd_t6.append("--show-measurements")
            print("Starting Task 6 overlay:", cmd_t6)
            subprocess.run(cmd_t6, check=True)
            # Task 7
            print("Running Task 7 evaluation …")
            try:
                run_evaluation_npz(str(est_npz), str(results_dir), run_id)
            except Exception as e:
                print(f"Task 7 failed: {e}")
    except Exception as ex:
        print(f"Task 6/7 failed: {ex}")

    # ----------------------------
    # Validation of expected artifacts
    # ----------------------------
    try:
        def _exists(name: str) -> bool:
            p_top = results_dir / name
            p_sub = results_dir / run_id / 'task6' / name
            return p_top.exists() or p_sub.exists()

        must_have = [
            f"{run_id}_task1_location_map.png",
            f"{run_id}_task2_static_interval.png",
            f"{run_id}_task2_vectors.png",
            f"{run_id}_task3_quaternions.png",
            f"{run_id}_task3_errors.png",
            f"{run_id}_task4_comparison_ned.png",
            f"{run_id}_task4_mixed_frames.png",
            f"{run_id}_task4_all_ned.png",
            f"{run_id}_task4_all_ecef.png",
            f"{run_id}_task4_all_body.png",
            f"{run_id}_task5_results_{method}.png",
            f"{run_id}_task5_mixed_frames.png",
            f"{run_id}_task5_all_ned.png",
            f"{run_id}_task5_all_ecef.png",
            f"{run_id}_task5_all_body.png",
            f"{run_id}_task6_overlay_NED.png",
            f"{run_id}_task6_overlay_ECEF.png",
            f"{run_id}_task6_overlay_BODY.png",
            f"{run_id}_task6_diff_truth_fused_over_time_NED.png",
            f"{run_id}_task6_diff_truth_fused_over_time_ECEF.png",
            f"{run_id}_task6_diff_truth_fused_over_time_Body.png",
            f"{run_id}_task7_3_residuals_position_velocity.png",
            f"{run_id}_task7_3_error_norms.png",
            f"{run_id}_task7_4_attitude_angles_euler.png",
            f"{run_id}_task7_5_diff_truth_fused_over_time_NED.png",
            f"{run_id}_task7_5_diff_truth_fused_over_time_ECEF.png",
            f"{run_id}_task7_5_diff_truth_fused_over_time_Body.png",
        ]
        missing = [m for m in must_have if not _exists(m)]
        if missing:
            print("[VALIDATE] Missing expected artifacts:")
            for m in missing:
                print("  -", m)
        else:
            print("[VALIDATE] All required Task1–Task7 artifacts present.")
    except Exception as e:
        print(f"[WARN] Artifact validation skipped: {e}")

    print("Davenport processing complete for X002")


if __name__ == "__main__":
    main()
