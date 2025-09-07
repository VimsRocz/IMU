#!/usr/bin/env python3
"""Run every (IMU, GNSS, method) combo and log the output.

The script processes each IMU/GNSS pair with all selected methods and writes
the console output to ``results/<IMU>_<GNSS>_<method>.log``.  By default the
bundled ``IMU_X`` data sets are used, but a YAML configuration file can
override the data files and the list of methods.

Example ``config.yml``::

    datasets:
      - imu: IMU_X001.dat
        gnss: GNSS_X001.csv
      - imu: IMU_X002.dat
        gnss: GNSS_X002.csv
    methods: [TRIAD, Davenport, SVD]

Run the script with ``--config config.yml`` to process those files.
"""

import argparse
from pathlib import Path as _Path
import sys as _sys
_SRC = _Path(__file__).resolve().parent
if str(_SRC) not in _sys.path:
    _sys.path.insert(0, str(_SRC))
REPO_ROOT = _SRC.parents[2]
import itertools
import os
import pathlib
import subprocess
import sys
from typing import Iterable, Tuple, Dict, Any, List, Optional
import logging
import re
import time
import zipfile
import pandas as pd
from tabulate import tabulate
import numpy as np
from scipy.spatial.transform import Rotation as R  # type: ignore
from utils import save_mat
from contextlib import redirect_stdout
import io
from evaluate_filter_results import run_evaluation_npz

from utils import compute_C_ECEF_to_NED

# Type aliases to improve type checking
YamlData = Dict[str, Any]
CandidateType = Tuple[int, float, np.ndarray]
ScoreType = Tuple[float, str]
QuatCandidateType = Tuple[str, np.ndarray]

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
from paths import (
    ensure_results_dir as _ensure_results,
    imu_path as _imu_path_helper,
    gnss_path as _gnss_path_helper,
)

# Import helper utilities from the utils package
from utils.timeline import print_timeline_summary  # type: ignore
from utils.resolve_truth_path import resolve_truth_path  # type: ignore

try:
    import yaml
except ModuleNotFoundError:  # allow running without PyYAML installed
    yaml = None

DEFAULT_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
    ("IMU_X002.dat", "GNSS_X002.csv"),
    ("IMU_X003.dat", "GNSS_X002.csv"),  # dataset X003 shares GNSS_X002
]

DEFAULT_METHODS = ["TRIAD", "SVD", "Davenport"]  # All methods now support comprehensive Task 1-7.6 analysis including quaternion error plots


SMALL_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X001_small.dat", "GNSS_X001_small.csv"),
    ("IMU_X002_small.dat", "GNSS_X002_small.csv"),
    ("IMU_X003_small.dat", "GNSS_X002_small.csv"),
]


SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def load_config(path: str) -> Tuple[List[Tuple[str, str]], List[str]]:
    """Return (datasets, methods) from a YAML config file."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to use --config")
    with open(path) as fh:
        data: YamlData = yaml.safe_load(fh) or {}
    datasets = [
        (str(item["imu"]), str(item["gnss"])) for item in data.get("datasets", [])
    ] or list(DEFAULT_DATASETS)
    methods = data.get("methods", DEFAULT_METHODS)
    return datasets, methods


def compute_C_NED_to_ECEF(lat: float, lon: float) -> np.ndarray:
    """Return rotation matrix from NED to ECEF frame."""
    return compute_C_ECEF_to_NED(lat, lon).T


def run_case(cmd: List[str], log_path: pathlib.Path) -> Tuple[int, List[str]]:
    """Run a single fusion command and log output live to console and file."""
    logger.info("Executing fusion command: %s", cmd)
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        summary_lines: List[str] = []
        if proc.stdout:
            for line in proc.stdout:
                print(line, end="")
                log.write(line)
                m = SUMMARY_RE.search(line)
                if m:
                    summary_lines.append(m.group(1))
        proc.wait()
    logger.info("Fusion command completed with return code %s", proc.returncode)
    return proc.returncode, summary_lines


# ---------------------------------------------------------------------------
# Task7 Attitude plotting helpers (mirrors logic from run_triad_only.py)
# ---------------------------------------------------------------------------
from typing import Dict, Any, List, Tuple


def _save_png_and_mat(base_png_path: str, data_dict: Dict[str, Any]):
    """Save current matplotlib figure to PNG and accompanying MAT file.

    The MAT filename mirrors the PNG but with .mat extension. Failures are
    logged but non-fatal.
    """
    try:
        import matplotlib.pyplot as plt  # type: ignore  # local import to avoid global dependency if never used
        png_path = pathlib.Path(base_png_path)
        png_path.parent.mkdir(parents=True, exist_ok=True)
        plt.gcf().savefig(png_path, dpi=150, bbox_inches="tight")  # type: ignore
        print(f"[PNG] {png_path}")
    except Exception as e:  # pragma: no cover - best-effort
        logger.warning(f"Failed to save PNG {base_png_path}: {e}")
    try:
        from scipy.io import savemat  # type: ignore
        mat_path = str(pathlib.Path(base_png_path).with_suffix(".mat"))
        savemat(mat_path, {k: (np.asarray(v) if not isinstance(v, (int, float)) else v) for k, v in data_dict.items()})  # type: ignore
        print(f"[MAT] {mat_path}")
    except Exception as e:  # pragma: no cover
        logger.warning(f"Failed to save MAT for {base_png_path}: {e}")
    return pathlib.Path(base_png_path)


def _task7_attitude_plots(est_npz: pathlib.Path, truth_file: pathlib.Path, tag: str, results_dir: pathlib.Path,
                          div_threshold_deg: float = 30.0, div_persist_sec: float = 10.0,
                          length_scan: str = "60,120,300,600,900,1200",
                          subdir: str = "task7_attitude") -> None:
    """Replicate Task7 attitude plots produced by run_triad_only for batch runs.

    Generates quaternion/euler truth-vs-estimate plots, component residuals,
    angle error over time, divergence summary and divergence-vs-length study.
    Uses the same sophisticated quaternion processing as run_triad_only.py.
    """
    try:
        import matplotlib.pyplot as plt  # type: ignore  # local import
    except Exception as e:  # pragma: no cover
        logger.warning(f"Matplotlib unavailable; skipping Task7 attitude plots: {e}")
        return

    if not est_npz.exists() or not truth_file or not truth_file.exists():
        return
    try:
        data = np.load(est_npz, allow_pickle=True)
    except Exception as e:
        logger.warning(f"Failed loading estimate NPZ for attitude plots: {e}")
        return

    # Extract time + quaternion (wxyz) from estimator
    time_s = data.get("time_s") or data.get("time")
    quat = data.get("att_quat") or data.get("attitude_q") or data.get("quat_log") or data.get("quat")
    if quat is None or time_s is None:
        logger.info("Estimator NPZ missing quaternion/time; skipping Task7 attitude plots")
        return
    time_s = np.asarray(time_s).ravel()
    quat = np.asarray(quat)
    if quat.shape[0] != time_s.shape[0]:
        n = min(len(time_s), len(quat))
        time_s = time_s[:n]
        quat = quat[:n]

    # Load truth using the same logic as run_triad_only.py _read_state_x001
    try:
        M = np.loadtxt(truth_file, dtype=float)
        if M.ndim != 2 or M.shape[1] < 10:
            raise ValueError("truth file unexpected shape")
        
        # Heuristics to find quaternion columns: last 4 numeric columns
        q_truth = M[:, -4:]  # assume (qw,qx,qy,qz)
        
        # Candidate time columns: prefer one whose range overlaps small window (seconds)
        candidates: List[CandidateType] = []
        for c in range(min(3, M.shape[1])):  # probe first up to 3 cols
            t_try = M[:, c].astype(float)
            if np.all(np.isfinite(t_try)):
                span = float(np.nanmax(t_try) - np.nanmin(t_try))
                candidates.append((c, span, t_try))
        if not candidates:
            raise ValueError("No finite time-like column found in truth file.")
        
        # Choose the candidate with the smallest positive span >= 1.0 sec
        positive_candidates = [(c, s, tcol) for (c, s, tcol) in candidates if s > 0]
        positive_candidates.sort(key=lambda x: x[1])
        _, _, t_raw = positive_candidates[0]
        
        # Zero-base time if it appears offset (e.g., starts >> 0 and span looks reasonable)
        t0 = float(np.nanmin(t_raw))
        t_truth = t_raw - t0 if t0 > 1.0 else t_raw
        
        # Extract positions for frame detection
        six_before_quat = M.shape[1] - 4 - 6
        if six_before_quat >= 0:
            pv = M[:, six_before_quat:six_before_quat+6]
            pos_ecef = pv[:, :3]
        else:
            # Fallback if not enough columns
            pos_ecef = np.zeros((len(t_truth), 3))
    except Exception as e:
        logger.warning(f"Failed loading truth for attitude: {e}")
        return

    # Advanced quaternion processing functions from run_triad_only.py
    def _norm_quat(q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, float)
        n = np.linalg.norm(q, axis=-1, keepdims=True)
        n[n == 0] = 1.0
        return q / n

    def _fix_hemisphere(qt: np.ndarray, qe: np.ndarray) -> np.ndarray:
        dot = np.sum(qt * qe, axis=1)
        s = np.sign(dot)
        s[s == 0] = 1.0
        return qe * s[:, None]

    def _quat_angle_deg(qt: np.ndarray, qe: np.ndarray) -> np.ndarray:
        d = np.clip(np.abs(np.sum(qt * qe, axis=1)), 0.0, 1.0)
        return 2.0 * np.degrees(np.arccos(d))

    # Utility function for future use - currently not called
    # def _quat_to_euler_zyx_deg(q: np.ndarray) -> np.ndarray:  # yaw, pitch, roll from [w,x,y,z]
    #     """Convert quaternion to Euler angles (ZYX order) in degrees. Helper function for potential future use."""
    #     w, x, y, z = q.T
    #     yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    #     s = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    #     pitch = np.degrees(np.arcsin(s))
    #     roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    #     return np.vstack([yaw, pitch, roll]).T

    # Utility function for potential future use - currently not called
    # def _align_to_same_len(x: np.ndarray, *ys: np.ndarray) -> Tuple[np.ndarray, Tuple[np.ndarray, ...]]:
    #     """Helper to align x and multiple y-arrays to the shortest common length. Utility function for future use."""
    #     n = int(min(len(x), *[len(y) for y in ys]))
    #     if n <= 0:
    #         return x, ys
    #     x2 = x[:n]
    #     ys2 = tuple(y[:n] for y in ys)
    #     return x2, ys2

    def q_conj(q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, float)
        return np.column_stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]])

    def q_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        w1, x1, y1, z1 = a.T
        w2, x2, y2, z2 = b.T
        return np.column_stack([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    def dcm_to_quat(C: np.ndarray) -> np.ndarray:
        dcm = np.asarray(C, float)
        t = np.trace(dcm)
        if t <= 0:
            i = int(np.argmax(np.diag(dcm)))
            if i == 0:
                s = 2.0 * np.sqrt(max(1e-12, 1 + dcm[0, 0] - dcm[1, 1] - dcm[2, 2]))
                w = (dcm[2, 1] - dcm[1, 2]) / s
                x = 0.25 * s
                y = (dcm[0, 1] + dcm[1, 0]) / s
                z = (dcm[0, 2] + dcm[2, 0]) / s
            elif i == 1:
                s = 2.0 * np.sqrt(max(1e-12, 1 + dcm[1, 1] - dcm[0, 0] - dcm[2, 2]))
                w = (dcm[0, 2] - dcm[2, 0]) / s
                x = (dcm[0, 1] + dcm[1, 0]) / s
                y = 0.25 * s
                z = (dcm[1, 2] + dcm[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(max(1e-12, 1 + dcm[2, 2] - dcm[0, 0] - dcm[1, 1]))
                w = (dcm[1, 0] - dcm[0, 1]) / s
                x = (dcm[0, 2] + dcm[2, 0]) / s
                y = (dcm[1, 2] + dcm[2, 1]) / s
                z = 0.25 * s
        else:
            s = 0.5 / np.sqrt(max(1e-12, t + 1.0))
            w = 0.25 / s
            x = (dcm[2, 1] - dcm[1, 2]) * s
            y = (dcm[0, 2] - dcm[2, 0]) * s
            z = (dcm[1, 0] - dcm[0, 1]) * s
        q = np.array([w, x, y, z], float)
        q = q / (np.linalg.norm(q) + 1e-12)
        if q[0] < 0:
            q = -q
        return q

    def quat_err_angle_deg(q_est_b2n: np.ndarray, q_truth_b2n: np.ndarray) -> np.ndarray:
        q_err = q_mul(q_est_b2n, q_conj(q_truth_b2n))
        q_err = _norm_quat(q_err)
        ang = 2*np.arccos(np.clip(np.abs(q_err[:, 0]), 0.0, 1.0)) * 180.0 / np.pi
        return ang

    def ned_to_enu_quat() -> np.ndarray:
        C_ne = np.array([[0.0, 1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, -1.0]])
        return dcm_to_quat(C_ne)

    def ecef_to_ned_quat(lat_rad: float, lon_rad: float) -> np.ndarray:
        sL, cL = np.sin(lat_rad), np.cos(lat_rad)
        sO, cO = np.sin(lon_rad), np.cos(lon_rad)
        C_ne = np.array([[-sL*cO, -sL*sO,  cL],
                         [    -sO,     cO,  0.0],
                         [-cL*cO, -cL*sO, -sL]], dtype=float)
        C_en = C_ne.T
        return dcm_to_quat(C_en)

    def wrap_world(q_b2W: np.ndarray, q_W2N: np.ndarray, q_N2W: np.ndarray) -> np.ndarray:
        return q_mul(np.tile(q_W2N, (q_b2W.shape[0], 1)), q_mul(q_b2W, np.tile(q_N2W, (q_b2W.shape[0], 1))))

    def average_quat(q: np.ndarray) -> np.ndarray:
        M = (q[:, :, None] @ q[:, None, :]).sum(axis=0)
        w, v = np.linalg.eigh(M)
        q_mean = v[:, np.argmax(w)]
        if q_mean[0] < 0:
            q_mean = -q_mean
        return q_mean

    # Align truth and estimate time bases
    try:
        # Use numpy searchsorted for speed - interpolate truth to estimate timeline
        idx = np.searchsorted(t_truth, time_s)
        idx[idx >= len(t_truth)] = len(t_truth) - 1
        qT_raw = q_truth[idx]
        pos_ecef_interp = pos_ecef[idx]
    except Exception:
        # Fallback: truncate to min length
        n = min(len(q_truth), len(quat))
        qT_raw = q_truth[:n]
        quat = quat[:n]
        time_s = time_s[:n]
        pos_ecef_interp = pos_ecef[:n] if len(pos_ecef) >= n else np.zeros((n, 3))

    # Normalize quaternions
    q_est = _norm_quat(quat)
    qT_raw = _norm_quat(qT_raw)

    # World transforms and frame detection (same as run_triad_only.py)
    try:
        from utils import ecef_to_geodetic
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_ecef_interp[0])
        ref_lat_rad = np.deg2rad(lat_deg)
        ref_lon_rad = np.deg2rad(lon_deg)
    except Exception:
        ref_lat_rad = 0.0
        ref_lon_rad = 0.0
    
    q_ne = ned_to_enu_quat()
    q_en = q_conj(np.atleast_2d(q_ne))[0]
    q_en_ecef = ecef_to_ned_quat(ref_lat_rad, ref_lon_rad)
    q_ne_ecef = q_conj(np.atleast_2d(q_en_ecef))[0]

    # Candidate interpretations of truth (same logic as run_triad_only.py)
    cands: List[QuatCandidateType] = []
    cands.append(("truth b2NED", qT_raw))
    cands.append(("truth NED2b (conj)", q_conj(qT_raw)))
    cands.append(("truth b2ENU → b2NED", wrap_world(qT_raw, q_en, q_ne)))
    cands.append(("truth ENU2b → b2NED", wrap_world(q_conj(qT_raw), q_en, q_ne)))
    cands.append(("truth b2ECEF → b2NED", wrap_world(qT_raw, q_en_ecef, q_ne_ecef)))
    cands.append(("truth ECEF2b → b2NED", wrap_world(q_conj(qT_raw), q_en_ecef, q_ne_ecef)))

    # Score on early/static window
    t0, t1 = 0.0, 60.0
    idx = np.where((time_s >= t0) & (time_s <= t1))[0]
    if idx.size < 100:
        idx = np.arange(min(1000, time_s.size))
    scores: List[ScoreType] = []
    for name, qT in cands:
        ang = quat_err_angle_deg(q_est[idx], qT[idx])
        scores.append((float(np.nanmean(ang)), name))
    best_i = int(np.argmin([s[0] for s in scores]))
    best_name, best_mean = scores[best_i][1], scores[best_i][0]
    qT_best = cands[best_i][1]
    print(f"[AutoAlign] best world/frame hypothesis: {best_name} (mean {best_mean:.3f}° over [{t0},{t1}]s)")

    # Estimate constant body-side delta over the window and apply to truth
    q_err_static = q_mul(q_est[idx], q_conj(qT_best[idx]))
    q_delta = average_quat(q_err_static)
    if q_delta[0] < 0:
        q_delta = -q_delta
    q_delta_tiled = np.tile(q_delta, (qT_best.shape[0], 1))
    qT_aligned = q_mul(q_delta_tiled, qT_best)
    print(f"[AutoAlign] applied fixed body-side Δ: {np.array2string(q_delta, precision=4)}")

    # Use aligned truth and sign-align estimate for plotting
    qT = _norm_quat(qT_aligned)
    qE = _fix_hemisphere(qT, q_est)
    # Angle error between aligned truth and estimate across full timeline
    ang = _quat_angle_deg(qT, qE)

    generated: List[pathlib.Path] = []

    # Quaternion components (truth vs est)
    plt.figure(figsize=(10, 6))  # type: ignore
    labels = ['w', 'x', 'y', 'z']
    for i, lab in enumerate(labels):
        ax = plt.subplot(2, 2, i + 1)  # type: ignore
        ax.plot(time_s, qT[:, i], '-', label='Truth')  # type: ignore
        ax.plot(time_s, qE[:, i], '--', label='KF')  # type: ignore
        ax.set_title(f'q_{lab}')  # type: ignore
        ax.grid(True)  # type: ignore
    plt.legend(loc='upper right')  # type: ignore
    plt.suptitle(f'{tag} Task7 (Body→NED): Quaternion Truth vs KF')  # type: ignore
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_BodyToNED_attitude_truth_vs_estimate_quaternion.png'),
                                       {'t': time_s, 'q_truth': qT, 'q_kf': qE}))

    # Quaternion component residuals
    dq = qE - qT
    plt.figure(figsize=(10, 6))  # type: ignore
    for i, lab in enumerate(labels):
        ax = plt.subplot(2, 2, i + 1)  # type: ignore
        ax.plot(time_s, dq[:, i], '-', label='Δq est − truth')  # type: ignore
        ax.set_title(f'Δq_{lab}')  # type: ignore
        ax.grid(True)  # type: ignore
        if i == 0:
            ax.legend(loc='upper right')  # type: ignore
    plt.suptitle(f'{tag} Task7.6 (Body→NED): Quaternion Component Error')  # type: ignore
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_BodyToNED_attitude_quaternion_error_components.png'),
                                       {'t': time_s, 'dq_wxyz': dq}))

    # Euler angles (ZYX) using scipy for consistency
    try:
        rotT = R.from_quat(qT[:, [1, 2, 3, 0]])  # type: ignore
        rotE = R.from_quat(qE[:, [1, 2, 3, 0]])  # type: ignore
        eT: np.ndarray = rotT.as_euler('zyx', degrees=True)  # type: ignore  # yaw pitch roll
        eE: np.ndarray = rotE.as_euler('zyx', degrees=True)  # type: ignore
    except Exception as e:
        logger.warning(f"Failed Euler conversion: {e}")
        return
    plt.figure(figsize=(10, 6))  # type: ignore
    e_labels = ['Yaw(Z)', 'Pitch(Y)', 'Roll(X)']
    for i, lab in enumerate(e_labels):
        ax = plt.subplot(3, 1, i + 1)  # type: ignore
        ax.plot(time_s, eT[:, i], '-', label='Truth')  # type: ignore
        ax.plot(time_s, eE[:, i], '--', label='KF')  # type: ignore
        ax.set_ylabel(lab + ' [deg]')  # type: ignore
        ax.grid(True)  # type: ignore
        if i == 0:
            ax.legend()  # type: ignore
    plt.xlabel('Time [s]')  # type: ignore
    plt.suptitle(f'{tag} Task7 (Body→NED): Euler (ZYX) Truth vs KF')  # type: ignore
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_BodyToNED_attitude_truth_vs_estimate_euler.png'),
                                       {'t': time_s, 'e_truth_zyx_deg': eT, 'e_kf_zyx_deg': eE}))

    def _wrap_deg(x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=float)
        x = (x + 180.0) % 360.0 - 180.0
        x[x == -180.0] = 180.0
        return x
    e_err: np.ndarray = _wrap_deg(eE - eT)  # type: ignore  # scipy types not fully available
    plt.figure(figsize=(10, 6))  # type: ignore
    for i, lab in enumerate(e_labels):
        ax = plt.subplot(3, 1, i + 1)  # type: ignore
        ax.plot(time_s, e_err[:, i], '-', label='Estimate − Truth')  # type: ignore
        ax.set_ylabel(lab + ' err [deg]')  # type: ignore
        ax.grid(True)  # type: ignore
        if i == 0:
            ax.legend(loc='upper right')  # type: ignore
    plt.xlabel('Time [s]')  # type: ignore
    plt.suptitle(f'{tag} Task7.6 (Body→NED): Euler Error (ZYX) vs Time')  # type: ignore
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_BodyToNED_attitude_euler_error_over_time.png'),
                                       {'t': time_s, 'e_error_zyx_deg': e_err}))

    # Quaternion angle error over time (Task7.6 + back-compat Task7)
    plt.figure(figsize=(10, 3))  # type: ignore
    plt.plot(time_s, ang, '-')  # type: ignore
    plt.grid(True)  # type: ignore
    plt.xlabel('Time [s]')  # type: ignore
    plt.ylabel('Angle Error [deg]')  # type: ignore
    plt.title(f'{tag} Task7.6: Quaternion Error (angle) vs Time')  # type: ignore
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_attitude_error_angle_over_time.png'),
                                       {'t': time_s, 'att_err_deg': ang}))
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_attitude_error_angle_over_time.png'),
                                       {'t': time_s, 'att_err_deg': ang}))

    # Divergence detection utilities
    def _divergence_time(t: np.ndarray, err: np.ndarray, thresh_deg: float, persist_s: float) -> float:
        t = np.asarray(t)
        err = np.asarray(err)
        dt = np.median(np.diff(t)) if t.size > 1 else 0.0
        if dt <= 0:
            return np.nan
        above = err >= thresh_deg
        need = max(1, int(round(persist_s / dt)))
        i = 0
        while i < above.size:
            if above[i]:
                j = i
                while j < above.size and above[j]:
                    j += 1
                if (j - i) >= need:
                    return t[i]
                i = j
            else:
                i += 1
        return np.nan

    div_t = _divergence_time(time_s, ang, div_threshold_deg, div_persist_sec)
    with open(results_dir / f'{tag}_Task7_divergence_summary.csv', 'w') as f:
        f.write('threshold_deg,persist_s,divergence_s\n')
        f.write(f"{div_threshold_deg},{div_persist_sec},{'' if np.isnan(div_t) else f'{div_t:.6f}'}\n")

    # Divergence vs length scan
    lengths = [float(x) for x in str(length_scan).split(',') if x.strip()]
    rows: List[Tuple[float, float]] = []
    for length in lengths:
        tend = min(time_s[0] + length, time_s[-1])
        m = time_s <= tend
        dv = _divergence_time(time_s[m], ang[m], div_threshold_deg, div_persist_sec)
        rows.append((length, np.nan if (dv != dv) else dv))
    with open(results_dir / f'{tag}_Task7_divergence_vs_length.csv', 'w') as f:
        f.write('tmax_s,divergence_s\n')
        for L, dv in rows:
            f.write(f"{L},{'' if (dv != dv) else dv}\n")
    plt.figure(figsize=(6, 4))  # type: ignore
    plt.plot([r[0] for r in rows], [np.nan if (r[1] != r[1]) else r[1] for r in rows], '-o')  # type: ignore
    plt.grid(True)  # type: ignore
    plt.xlabel('Dataset length used [s]')  # type: ignore
    plt.ylabel('Divergence time [s]')  # type: ignore
    plt.title(f'{tag} Task7: Divergence vs Length')  # type: ignore
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_divergence_vs_length.png'),
                                       {'tmax_s': np.array([r[0] for r in rows], float),
                                        'divergence_s': np.array([np.nan if (r[1] != r[1]) else r[1] for r in rows], float)}))
    logger.info("Generated Task7 attitude plots for %s", tag)
    # Copy into per-run subdirectory and create manifest
    try:
        import json, shutil
        subdir_path = results_dir / tag / subdir
        subdir_path.mkdir(parents=True, exist_ok=True)
        unified_task7 = results_dir / tag / 'task7'
        unified_task7.mkdir(parents=True, exist_ok=True)
        manifest: List[str] = []
        for p in generated:
            for ext in ('.png', '.mat'):
                src = p.with_suffix(ext)
                if src.exists():
                    dst = subdir_path / src.name
                    try:
                        shutil.copy2(src, dst)
                    except Exception:
                        pass
                    # also copy into unified task7 folder
                    try:
                        shutil.copy2(src, unified_task7 / src.name)
                    except Exception:
                        pass
            manifest.append(p.name)
        manifest_path = subdir_path / 'task7_attitude_manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump({
                'tag': tag,
                'generated_pngs': [p.with_suffix('.png').name for p in generated],
                'generated_mats': [p.with_suffix('.mat').name for p in generated],
                'count': len(generated)
            }, f, indent=2)
        logger.info('[TASK 7] Attitude plots (truth vs estimate + errors):')
        for p in generated:
            logger.info('  %s', p.with_suffix('.png').name)
        logger.info('[TASK 7] Attitude manifest: %s', manifest_path)
    except Exception as e:  # pragma: no cover
        logger.warning('Failed to create Task7 attitude manifest: %s', e)
    # close any excess figures to avoid memory leak
    try:
        import matplotlib.pyplot as plt  # type: ignore
        plt.close('all')  # type: ignore
    except Exception:
        pass


def main(argv: Optional[List[str]] = None) -> None:
    results_dir = _ensure_results()
    logger.info("Ensured '%s' directory exists.", results_dir)
    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with multiple datasets and methods",
    )
    parser.add_argument(
        "--config",
        help="YAML file specifying datasets and methods",
    )
    parser.add_argument(
        "--include-small",
        action="store_true",
        help="Include *_small dataset variants",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation for faster execution",
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
        help="Enable verbose debug output",
    )
    args = parser.parse_args(argv)

    if args.debug:
        logger.setLevel(logging.DEBUG)

    if args.task == 7:
        from evaluate_filter_results import run_evaluation

    # Task 7 results must be stored directly in ``results/``
        # according to the updated project requirements.
        run_evaluation(
            prediction_file="outputs/predicted_states.csv",
            gnss_file="outputs/gnss_measurements.csv",
            attitude_file="outputs/estimated_attitude.csv",
            save_path=str(results_dir),
        )
        return

    if args.config:
        cases, methods = load_config(args.config)
    else:
        cases, methods = list(DEFAULT_DATASETS), list(DEFAULT_METHODS)
        if args.include_small:
            cases.extend(SMALL_DATASETS)

    logger.debug(f"Datasets: {cases}")
    logger.debug(f"Methods: {methods}")

    # Type-annotated results list for better type inference
    results: List[Dict[str, Any]] = []
    for (imu, gnss), m in itertools.product(cases, methods):
        imu_path = _imu_path_helper(imu)
        gnss_path = _gnss_path_helper(gnss)
        tag = f"{imu_path.stem}_{gnss_path.stem}_{m}"
        log_path = results_dir / f"{tag}.log"
        print(f"\u25b6 {tag}")

        # Print and save a concise timeline summary for the current dataset.
        try:
            truth_path = resolve_truth_path()
        except Exception:
            truth_path = None
        try:
            print_timeline_summary(tag, str(imu_path), str(gnss_path), truth_path, out_dir=results_dir)
        except Exception:
            # Timeline is best-effort; continue even if it fails
            pass

        if logger.isEnabledFor(logging.DEBUG):
            try:
                gnss_preview = np.loadtxt(
                    gnss_path, delimiter=",", skiprows=1, max_rows=1
                )
                imu_preview = np.loadtxt(imu_path, max_rows=1)
                logger.debug(
                    f"GNSS preview: shape {gnss_preview.shape}, first row: {gnss_preview}"
                )
                logger.debug(
                    f"IMU preview: shape {imu_preview.shape}, first row: {imu_preview}"
                )
            except Exception as e:
                logger.warning(f"Failed data preview for {imu} or {gnss}: {e}")
        cmd = [
            sys.executable,
            str(HERE / "GNSS_IMU_Fusion.py"),
            "--imu-file",
            str(imu_path),
            "--gnss-file",
            str(gnss_path),
            "--method",
            m,
        ]
        # Pass truth to fusion when available and allow dataset mismatches
        try:
            if truth_path and pathlib.Path(truth_path).exists():
                cmd += ["--truth-file", str(truth_path), "--allow-truth-mismatch"]
        except Exception:
            pass
        if args.no_plots:
            cmd.append("--no-plots")
        start_t = time.time()
        ret, summaries = run_case(cmd, log_path)
        runtime = time.time() - start_t
        if ret != 0:
            raise subprocess.CalledProcessError(ret, cmd)
        for summary in summaries:
            kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
            results.append(
                {
                    "dataset": pathlib.Path(imu).stem,
                    "method": kv.get("method", m),
                    "rmse_pos": float(kv.get("rmse_pos", "nan").replace("m", "")),
                    "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                    "rms_resid_pos": float(
                        kv.get("rms_resid_pos", "nan").replace("m", "")
                    ),
                    "max_resid_pos": float(
                        kv.get("max_resid_pos", "nan").replace("m", "")
                    ),
                    "rms_resid_vel": float(
                        kv.get("rms_resid_vel", "nan").replace("m", "")
                    ),
                    "max_resid_vel": float(
                        kv.get("max_resid_vel", "nan").replace("m", "")
                    ),
                    "runtime": runtime,
                }
            )
        # ------------------------------------------------------------------
        # Convert NPZ output to a MATLAB file with explicit frame variables
        # ------------------------------------------------------------------
        npz_path = results_dir / f"{tag}_kf_output.npz"
        if npz_path.exists():
            try:
                data = np.load(npz_path, allow_pickle=True)
            except (OSError, zipfile.BadZipFile) as exc:
                logger.error("Failed to load %s: %s", npz_path, exc)
                continue
            logger.debug(f"Loaded output {npz_path} with keys: {list(data.keys())}")
            time_s = data.get("time_s")
            if time_s is None:
                time_s = data.get("time")

            pos_ned = data.get("pos_ned_m")
            if pos_ned is None:
                pos_ned = data.get("pos_ned")
            if pos_ned is None:
                pos_ned = data.get("fused_pos")

            vel_ned = data.get("vel_ned_ms")
            if vel_ned is None:
                vel_ned = data.get("vel_ned")
            if vel_ned is None:
                vel_ned = data.get("fused_vel")
            if logger.isEnabledFor(logging.DEBUG):
                logger.debug(
                    f"Output time range: {time_s[0] if time_s is not None else 'N/A'}"
                    f" to {time_s[-1] if time_s is not None else 'N/A'} s"
                )
                logger.debug(
                    f"Position shape: {pos_ned.shape if pos_ned is not None else 'None'}"
                )
            ref_lat = data.get("ref_lat_rad")
            ref_lon = data.get("ref_lon_rad")
            ref_r0 = data.get("ref_r0_m")
            if ref_lat is None:
                ref_lat = data.get("ref_lat")
            if ref_lon is None:
                ref_lon = data.get("ref_lon")
            if ref_r0 is None:
                ref_r0 = data.get("ref_r0")
            ref_lat = float(np.squeeze(ref_lat))
            ref_lon = float(np.squeeze(ref_lon))
            ref_r0 = np.asarray(ref_r0)

            C_NED_ECEF = compute_C_NED_to_ECEF(ref_lat, ref_lon)
            pos_ecef = (C_NED_ECEF @ pos_ned.T).T + ref_r0
            vel_ecef = (C_NED_ECEF @ vel_ned.T).T

            quat = data.get("att_quat")
            if quat is None:
                quat = data.get("attitude_q")
            if quat is None:
                quat = data.get("quat_log")
            if quat is not None:
                rot = R.from_quat(quat[:, [1, 2, 3, 0]])  # type: ignore
                C_B_N: np.ndarray = rot.as_matrix()  # type: ignore
                pos_body = np.einsum("nij,nj->ni", C_B_N.transpose(0, 2, 1), pos_ned)  # type: ignore
                vel_body = np.einsum("nij,nj->ni", C_B_N.transpose(0, 2, 1), vel_ned)  # type: ignore
            else:
                pos_body = np.zeros_like(pos_ned)
                vel_body = np.zeros_like(vel_ned)

            mat_out: Dict[str, Any] = {
                "time_s": time_s,
                "pos_ned_m": pos_ned,
                "vel_ned_ms": vel_ned,
                "pos_ecef_m": pos_ecef,
                "vel_ecef_ms": vel_ecef,
                "pos_body_m": pos_body,
                "vel_body_ms": vel_body,
                "ref_lat_rad": ref_lat,
                "ref_lon_rad": ref_lon,
                "ref_r0_m": ref_r0,
                "att_quat": quat,
                "method_name": m,
                # Consistent fused data for Tasks 5 and 6
                "fused_pos": pos_ned,
                "fused_vel": vel_ned,
                "quat_log": quat,
            }
            save_mat(npz_path.with_suffix(".mat"), mat_out)

            logger.info(
                "Subtask 6.8.2: Plotted %s position North: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 0],
                pos_ned[-1, 0],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s position East: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 1],
                pos_ned[-1, 1],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s position Down: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 2],
                pos_ned[-1, 2],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity North: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 0],
                vel_ned[-1, 0],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity East: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 1],
                vel_ned[-1, 1],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity Down: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 2],
                vel_ned[-1, 2],
            )

            # ----------------------------
            # Task 6: Truth overlay plots
            # ----------------------------
            truth_file = None
            m_ds = re.search(r"IMU_(X\d+)", imu_path.stem)
            if m_ds:
                ds_id = m_ds.group(1)
                for cand in [
                    ROOT / f"STATE_{ds_id}.txt",
                    ROOT / f"STATE_{ds_id}_small.txt",
                ]:
                    if cand.is_file():
                        truth_file = cand
                        break

            overlay_cmd = [
                sys.executable,
                str(HERE / "task6_plot_truth.py"),
                "--est-file",
                str(npz_path.with_suffix(".mat")),
                "--gnss-file",
                str(gnss_path),
                "--output",
                "results",
            ]
            # Apply Task 6 plotting defaults (decimation + y-limit sync)
            overlay_cmd += ["--decimate-maxpoints", "200000", "--ylim-percentile", "99.5"]
            if truth_file is not None:
                overlay_cmd.extend(["--truth-file", str(truth_file.resolve())])
            if args.show_measurements:
                overlay_cmd.append("--show-measurements")
            with open(log_path, "a") as log:
                log.write("\nTASK 6: Overlay fused output with truth\n")
                msg = "Starting Task 6 overlay ..."
                logger.info(msg)
                log.write(msg + "\n")
                proc = subprocess.Popen(
                    overlay_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                if proc.stdout:
                    for line in proc.stdout:
                        print(line, end="")
                        log.write(line)
                proc.wait()

            # ----------------------------
            # Task 7: Evaluation
            # ----------------------------
            # Updated: Task 7 outputs are saved directly in ``results/``
            task7_dir = results_dir
            with open(log_path, "a") as log:
                log.write("\nTASK 7: Evaluate residuals\n")
                msg = "Running Task 7 evaluation ..."
                logger.info(msg)
                log.write(msg + "\n")
                buf = io.StringIO()
                with redirect_stdout(buf):
                    try:
                        run_evaluation_npz(str(npz_path), str(task7_dir), tag)
                    except Exception as e:
                        print(f"Task 7 failed: {e}")
                output = buf.getvalue()
                print(output, end="")
                log.write(output)
                print(
                    f"Saved Task 7.5 diff-truth plots (NED/ECEF/Body) under: results/{tag}/"
                )

            # ----------------------------
            # Task 7.6 Attitude plots (match run_triad_only output set)
            # ----------------------------
            try:
                if truth_file is not None and truth_file.exists():
                    logger.info("Generating Task 7.6 quaternion component error plots...")
                    _task7_attitude_plots(npz_path, truth_file, tag, results_dir)
                    logger.info("✅ Task 7.6 quaternion component error plots generated successfully")
                else:
                    logger.warning("⚠️ Truth file not available for Task 7.6 plots")
            except Exception as ex:
                logger.warning(f"⚠️ Task 7.6 attitude plots failed for {tag}: {ex}")

            # ----------------------------
            # Attitude comparison (KF vs truth, DR true-init)
            # ----------------------------
            try:
                est_npz = results_dir / f"{tag}_kf_output.npz"
                if est_npz.exists() and truth_path and pathlib.Path(truth_path).exists():
                    att_cmd = [
                        sys.executable,
                        str(HERE / "../scripts/plot_attitude_kf_vs_no_kf.py"),
                        "--imu-file", str(imu_path),
                        "--est-file", str(est_npz),
                        "--truth-file", str(truth_path),
                        "--out-dir", str(results_dir),
                        "--true-init",
                    ]
                    env = os.environ.copy()
                    # Ensure 'src' is importable from the script
                    try:
                        env["PYTHONPATH"] = str(ROOT) + os.pathsep + str(HERE) + os.pathsep + env.get("PYTHONPATH", "")
                    except Exception:
                        pass
                    with open(log_path, "a") as log:
                        logger.info("Running attitude comparison plots: %s", att_cmd)
                        proc = subprocess.Popen(att_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
                        if proc.stdout:
                            for line in proc.stdout:
                                print(line, end="")
                                log.write(line)
                        proc.wait()
                        print("Generated attitude comparison plots (KF vs truth, DR vs truth).")
                        log.write("Generated attitude comparison plots (KF vs truth, DR vs truth).\n")
            except Exception as ex:
                logger.info(f"[WARN] Attitude comparison plotting failed: {ex}")

        # Print dataset completion summary with Task 7.6 confirmation
        logger.info(f"✅ {tag} processing complete with all Tasks 1-7.6 including quaternion error analysis")
    
    # --- nicely formatted summary table for Tasks 1-7.6 results --------------------------------------
    if results:
        logger.info("📊 Final Summary: IMU/GNSS Fusion Results (Tasks 1-7.6 Complete)")
        key_order = {m: i for i, m in enumerate(methods)}
        # Fix type annotation: explicitly type the lambda argument as Dict[str, Any]
        results.sort(key=lambda r: (str(r["dataset"]), int(key_order.get(str(r["method"]), 0))))
        rows = [
            [
                e["dataset"],
                e["method"],
                e["rmse_pos"],
                e["final_pos"],
                e["rms_resid_pos"],
                e["max_resid_pos"],
                e["rms_resid_vel"],
                e["max_resid_vel"],
                e["runtime"],
            ]
            for e in results
        ]
        print(
            tabulate(
                rows,
                headers=[
                    "Dataset",
                    "Method",
                    "RMSEpos [m]",
                    "End-Error [m]",
                    "RMSresidPos [m]",
                    "MaxresidPos [m]",
                    "RMSresidVel [m/s]",
                    "MaxresidVel [m/s]",
                    "Runtime [s]",
                ],
                floatfmt=".2f",
            )
        )
        logger.info("📈 Task 7.6 quaternion component error plots generated for all datasets")
        df = pd.DataFrame(
            rows,
            columns=[
                "Dataset",
                "Method",
                "RMSEpos_m",
                "EndErr_m",
                "RMSresidPos_m",
                "MaxresidPos_m",
                "RMSresidVel_mps",
                "MaxresidVel_mps",
                "Runtime_s",
            ],
        )
        df.to_csv(results_dir / "summary.csv", index=False)
        logger.info("💾 Summary CSV saved with comprehensive Task 1-7.6 metrics")


if __name__ == "__main__":
    main()
