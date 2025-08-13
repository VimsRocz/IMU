"""Task 6 overlay helpers supporting multiple frames and methods.

This module provides utilities to load estimator and truth data, perform
coordinate-frame conversions and generate overlay plots for Task 6.  The
implementation has been extended to support time-varying quaternion histories
so that BODY-frame overlays may rotate each sample individually.  When a
quaternion history is missing, a constant quaternion provided by the caller is
used as a fallback, otherwise the identity rotation is assumed.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt


# ---------------------------------------------------------------------------
# Quaternion utilities
# ---------------------------------------------------------------------------

def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalise quaternion array along the last axis."""

    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0.0] = 1.0
    return q / n


def quat_make_hemisphere_continuous(q: np.ndarray) -> np.ndarray:
    """Enforce sign-continuity for a quaternion sequence."""

    q = np.asarray(q, dtype=float).copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0.0:
            q[i] *= -1.0
    return q


def slerp_series(t_src: np.ndarray, q_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Interpolate a quaternion series from ``t_src`` to ``t_dst``.

    Quaternions are provided in ``[qw, qx, qy, qz]`` order.  The sequence is
    normalised and made hemisphere-continuous prior to interpolation.  SciPy's
    :class:`~scipy.spatial.transform.Slerp` is used when available and falls
    back to component-wise linear interpolation followed by renormalisation
    otherwise.
    """

    q_src = quat_make_hemisphere_continuous(quat_normalize(q_src))
    t_src = np.asarray(t_src, dtype=float).ravel()
    t_dst = np.asarray(t_dst, dtype=float).ravel()

    try:  # pragma: no cover - SciPy may be missing on some systems
        from scipy.spatial.transform import Rotation, Slerp

        rot = Rotation.from_quat(q_src[:, [1, 2, 3, 0]])  # SciPy expects xyzw
        slerp = Slerp(t_src, rot)
        interp_rot = slerp(t_dst)
        q_xyzw = interp_rot.as_quat()
        q_dst = np.column_stack((q_xyzw[:, 3], q_xyzw[:, 0:3]))
    except Exception:  # pragma: no cover - best effort fallback
        q_dst = np.column_stack(
            [np.interp(t_dst, t_src, q_src[:, i]) for i in range(4)]
        )
        q_dst = quat_normalize(q_dst)
        q_dst = quat_make_hemisphere_continuous(q_dst)
    return q_dst


def quat_to_dcm_batch(q: np.ndarray) -> np.ndarray:
    """Convert a batch of quaternions ``[N,4]`` to DCMs ``[N,3,3]``."""

    q = quat_normalize(np.asarray(q, dtype=float))
    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (qy * qy + qz * qz)
    R[:, 0, 1] = 2 * (qx * qy - qw * qz)
    R[:, 0, 2] = 2 * (qx * qz + qw * qy)
    R[:, 1, 0] = 2 * (qx * qy + qw * qz)
    R[:, 1, 1] = 1 - 2 * (qx * qx + qz * qz)
    R[:, 1, 2] = 2 * (qy * qz - qw * qx)
    R[:, 2, 0] = 2 * (qx * qz - qw * qy)
    R[:, 2, 1] = 2 * (qy * qz + qw * qx)
    R[:, 2, 2] = 1 - 2 * (qx * qx + qy * qy)
    return R


def rotate_series_by_quat_series(
    v_ned: np.ndarray, q_b2n: np.ndarray, *, to_body: bool = True
) -> np.ndarray:
    """Rotate a vector series using a quaternion series."""

    if v_ned is None or q_b2n is None:
        return None

    R = quat_to_dcm_batch(q_b2n)
    if to_body:
        Rn2b = np.transpose(R, (0, 2, 1))
        return np.einsum("nij,nj->ni", Rn2b, v_ned)
    return np.einsum("nij,nj->ni", R, v_ned)


# ---------------------------------------------------------------------------
# Generic helpers
# ---------------------------------------------------------------------------

def _ensure_dir(p: Path) -> None:
    p.mkdir(parents=True, exist_ok=True)


def _to_np(a):
    return np.asarray(a) if a is not None else None


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

def load_estimates(est_file: str) -> Dict[str, np.ndarray]:
    """Load estimator output from ``.mat`` or ``.npz`` files."""

    p = Path(est_file)
    if p.suffix == ".npz":
        d = dict(np.load(p))
    else:
        d = {
            k: v
            for k, v in sio.loadmat(p, squeeze_me=True).items()
            if not k.startswith("__")
        }

    def pick(*names):
        for n in names:
            for k in d.keys():
                if k.lower() == n.lower():
                    return _to_np(d[k])
        return None

    out = {
        "time": pick("t", "time", "Time", "TIME", "time_s", "t_est"),
        "pos_ned": pick("pos_ned", "posNED", "position_ned", "pos_ned_m"),
        "vel_ned": pick("vel_ned", "velNED", "velocity_ned", "vel_ned_ms"),
        "acc_ned": pick("acc_ned", "accNED", "acceleration_ned", "acc_ned_ms"),
        "pos_ecef": pick("pos_ecef", "posECEF", "position_ecef", "pos_ecef_m"),
        "vel_ecef": pick("vel_ecef", "velECEF", "velocity_ecef", "vel_ecef_ms"),
        "acc_ecef": pick("acc_ecef", "accECEF", "acceleration_ecef", "acc_ecef_ms"),
        "pos_body": pick("pos_body", "posBODY", "pos_body_m"),
        "vel_body": pick("vel_body", "velBODY", "vel_body_ms"),
        "acc_body": pick("acc_body", "accBODY", "acc_body_ms"),
    }

    # Quaternion history
    q_key = None
    for k in d.keys():
        lk = k.lower()
        if lk in {
            "q_b2n",
            "quat_b2n",
            "qbn",
            "q_b2ned",
            "quatbn",
            "quats",
            "quat_wxyz",
            "quat_xyzw",
        }:
            q_key = k
            break

    if q_key is not None:
        q_hist = _to_np(d[q_key])
        q_hist = np.atleast_2d(q_hist)
        if q_hist.shape[0] == 4 and q_hist.shape[1] != 4:
            q_hist = q_hist.T
        q_hist = q_hist.reshape((-1, 4))

        if q_key.lower().endswith("xyzw"):
            q_hist = q_hist[:, [3, 0, 1, 2]]
        elif q_key.lower().endswith("wxyz"):
            pass
        else:
            if np.abs(q_hist[:, 0]).mean() < np.abs(q_hist[:, 3]).mean():
                q_hist = q_hist[:, [3, 0, 1, 2]]

        q_hist = quat_normalize(q_hist)
        q_hist = quat_make_hemisphere_continuous(q_hist)
        out["q_b2n"] = q_hist
        t_q = pick(
            "t_q",
            "t_quat",
            "time_q",
            "time_quat",
            "t_att",
            "time_att",
            "t_q_b2n",
            "time_q_b2n",
        )
        if t_q is not None:
            out["q_b2n_time"] = _to_np(t_q).ravel()

    return out


def load_truth(truth_file: str) -> Dict[str, np.ndarray]:
    """Load ground-truth data from text or CSV files."""

    import pandas as pd

    try:
        df = pd.read_csv(truth_file, sep=None, engine="python")
        cols = {c.lower(): c for c in df.columns}

        out: Dict[str, np.ndarray] = {}
        for cand in ["time", "t", "posix_time", "sec", "seconds"]:
            if cand in cols:
                out["time"] = df[cols[cand]].to_numpy()
                break
        if "time" not in out:
            out["time"] = np.arange(len(df))

        def grab(prefixes, axes):
            m = {}
            for ax in axes:
                for pref in prefixes:
                    key = f"{pref}{ax}"
                    if key in cols:
                        m[ax] = cols[key]
                        break
            return m

        ecef_map = grab(
            ["pos_ecef_", "ecef_", "x_ecef", "y_ecef", "z_ecef", ""],
            ["x", "y", "z"],
        )
        if len(ecef_map) == 3:
            out["pos_ecef"] = df[[ecef_map["x"], ecef_map["y"], ecef_map["z"]]].to_numpy()

        vel_ecef_map = grab(
            ["vel_ecef_", "ecef_v", "v_ecef", ""],
            ["x", "y", "z"],
        )
        if len(vel_ecef_map) == 3:
            out["vel_ecef"] = df[
                [vel_ecef_map["x"], vel_ecef_map["y"], vel_ecef_map["z"]]
            ].to_numpy()

        ned_map = grab(["pos_ned_", "ned_", "pos_"], ["n", "e", "d"])
        if len(ned_map) == 3:
            out["pos_ned"] = df[[ned_map["n"], ned_map["e"], ned_map["d"]]].to_numpy()

        # Quaternion (optional)
        q_cols = None
        order = "wxyz"
        for base in [
            "q_b2n_",
            "quat_b2n_",
            "qbn_",
            "q_b2ned_",
            "quatbn_",
            "quat_",
            "q_b2n_truth_",
        ]:
            candidates = [f"{base}{c}" for c in ["w", "x", "y", "z"]]
            if all(c in cols for c in candidates):
                q_cols = [cols[c] for c in candidates]
                order = "wxyz"
                break
            candidates = [f"{base}{c}" for c in ["x", "y", "z", "w"]]
            if all(c in cols for c in candidates):
                q_cols = [cols[c] for c in candidates]
                order = "xyzw"
                break
        if q_cols:
            q = df[q_cols].to_numpy()
            if order == "xyzw":
                q = q[:, [3, 0, 1, 2]]
            q = quat_normalize(q)
            q = quat_make_hemisphere_continuous(q)
            out["q_b2n_truth"] = q

        return out
    except Exception:
        pass

    # Fallback: whitespace separated text without headers
    arr = np.loadtxt(truth_file, comments="#")
    out: Dict[str, np.ndarray] = {"time": arr[:, 1]}
    if arr.shape[1] >= 8:
        out["pos_ecef"] = arr[:, 2:5]
        out["vel_ecef"] = arr[:, 5:8]
    return out


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------

def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    sL, cL = np.sin(lat_rad), np.cos(lat_rad)
    sO, cO = np.sin(lon_rad), np.cos(lon_rad)
    return np.array([
        [-sL * cO, -sL * sO, cL],
        [-sO, cO, 0.0],
        [-cL * cO, -cL * sO, -sL],
    ])

def ecef_to_ned_vec(v_ecef: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R @ v_ecef.T).T

def ned_to_ecef_vec(v_ned: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R.T @ v_ned.T).T

def interp_to(t_src, X_src, t_dst):
    X_src = np.asarray(X_src)
    out = np.zeros((len(t_dst), X_src.shape[1]))
    for i in range(X_src.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, X_src[:, i])
    return out

# ---------------------------------------------------------------------------
# Plotting helpers
# ---------------------------------------------------------------------------

def plot_overlay_3x3(time: np.ndarray,
                      est_xyz: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
                      truth_xyz: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
                      title: str,
                      ylabels: List[str],
                      outfile: str) -> None:
    fig, axes = plt.subplots(3, 3, figsize=(16, 10), sharex=True)
    comps = ['X', 'Y', 'Z']
    for r, yl in enumerate(ylabels):
        est_block = est_xyz[r]
        truth_block = truth_xyz[r]
        for c in range(3):
            ax = axes[r, c]
            if est_block is None or truth_block is None:
                ax.text(0.5, 0.5, 'n/a', ha='center', va='center', transform=ax.transAxes)
            else:
                ax.plot(time, est_block[:, c], linewidth=1.2, label='Estimated')
                ax.plot(time, truth_block[:, c], linestyle='--', linewidth=1.0, label='Truth')
            ax.set_ylabel(f'{yl} {comps[c]}')
            if r == 2:
                ax.set_xlabel('Time [s]')
            ax.grid(True, alpha=0.3)
    handles, labels = axes[0,0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=min(4, len(labels)), loc='upper center')
    fig.suptitle(title)
    fig.tight_layout(rect=[0,0,1,0.92])
    fig.savefig(outfile, dpi=150)
    plt.close(fig)

def plot_methods_overlay_3x3(time: np.ndarray,
                             methods_xyz: Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]],
                             truth_xyz: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
                             title: str,
                             ylabels: List[str],
                             outfile: str) -> None:
    fig, axes = plt.subplots(3, 3, figsize=(16, 10), sharex=True)
    comps = ['X', 'Y', 'Z']
    method_names = list(methods_xyz.keys())
    for r, yl in enumerate(ylabels):
        truth_block = truth_xyz[r]
        for c in range(3):
            ax = axes[r, c]
            plotted = False
            for name in method_names:
                block = methods_xyz[name][r]
                if block is None:
                    continue
                ax.plot(time, block[:, c], linewidth=1.2, label=name)
                plotted = True
            if truth_block is not None:
                ax.plot(time, truth_block[:, c], linestyle='--', linewidth=1.0, label='Truth')
                plotted = True
            if not plotted:
                ax.text(0.5, 0.5, 'n/a', ha='center', va='center', transform=ax.transAxes)
            ax.set_ylabel(f'{yl} {comps[c]}')
            if r == 2:
                ax.set_xlabel('Time [s]')
            ax.grid(True, alpha=0.3)
    handles, labels = [], []
    for ax in axes.flat:
        h,l = ax.get_legend_handles_labels()
        for hi,li in zip(h,l):
            if li not in labels:
                handles.append(hi); labels.append(li)
    fig.legend(handles, labels, ncol=min(4,len(labels)), loc='upper center')
    fig.suptitle(title)
    fig.tight_layout(rect=[0,0,1,0.92])
    fig.savefig(outfile, dpi=150)
    plt.close(fig)

# ---------------------------------------------------------------------------
# Manifest helper
# ---------------------------------------------------------------------------

def _update_manifest(manifest_path: Path, files: List[str]) -> None:
    if manifest_path.exists():
        try:
            manifest = json.load(manifest_path.open())
            if not isinstance(manifest, list):
                manifest = []
        except Exception:
            manifest = []
    else:
        manifest = []
    for f in files:
        if f not in manifest:
            manifest.append(f)
    with manifest_path.open('w', encoding='utf-8') as fh:
        json.dump(manifest, fh, indent=2)

# ---------------------------------------------------------------------------
# Core runners
# ---------------------------------------------------------------------------

def run_task6_overlay_all_frames(*,
                                 est_file: str,
                                 truth_file: str,
                                 output_dir: str,
                                 lat_deg: float | None = None,
                                 lon_deg: float | None = None,
                                 gnss_file: str | None = None,
                                 q_b2n_const: Tuple[float,float,float,float] | None = None
                                 ) -> List[str]:
    outp = Path(output_dir)
    _ensure_dir(outp)
    manifest_path = outp / 'task6_overlay_manifest.json'
    est = load_estimates(est_file)
    tru = load_truth(truth_file)
    t_est = est.get('time')
    if t_est is None:
        raise ValueError('Estimator output lacks time array')
    q_hist = est.get('q_b2n')
    if q_hist is not None:
        t_q = est.get('q_b2n_time', t_est)
        if len(q_hist) != len(t_est):
            q_hist = slerp_series(t_q, q_hist, t_est)
    pos_ned_est = est.get('pos_ned')
    vel_ned_est = est.get('vel_ned')
    acc_ned_est = est.get('acc_ned')
    if ((pos_ned_est is None or vel_ned_est is None) and est.get('pos_ecef') is not None
            and lat_deg is not None and lon_deg is not None):
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        if pos_ned_est is None:
            pos_ned_est = ecef_to_ned_vec(est['pos_ecef'], lat, lon)
        if vel_ned_est is None and est.get('vel_ecef') is not None:
            vel_ned_est = ecef_to_ned_vec(est['vel_ecef'], lat, lon)
        if acc_ned_est is None and est.get('acc_ecef') is not None:
            acc_ned_est = ecef_to_ned_vec(est['acc_ecef'], lat, lon)
    pos_ned_tru = tru.get('pos_ned')
    vel_ned_tru = tru.get('vel_ned')
    acc_ned_tru = tru.get('acc_ned')
    pos_ecef_tru = tru.get('pos_ecef')
    vel_ecef_tru = tru.get('vel_ecef')
    acc_ecef_tru = tru.get('acc_ecef')
    t_truth = tru.get('time', t_est)
    if (pos_ned_tru is None and pos_ecef_tru is not None and lat_deg is not None and lon_deg is not None):
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        pos_ned_tru = ecef_to_ned_vec(pos_ecef_tru, lat, lon)
        if vel_ecef_tru is not None:
            vel_ned_tru = ecef_to_ned_vec(vel_ecef_tru, lat, lon)
    if pos_ned_tru is not None:
        pos_ned_tru = interp_to(t_truth, pos_ned_tru, t_est)
    if vel_ned_tru is not None:
        vel_ned_tru = interp_to(t_truth, vel_ned_tru, t_est)
    if acc_ned_tru is not None:
        acc_ned_tru = interp_to(t_truth, acc_ned_tru, t_est)
    if pos_ecef_tru is not None:
        pos_ecef_tru = interp_to(t_truth, pos_ecef_tru, t_est)
    if vel_ecef_tru is not None:
        vel_ecef_tru = interp_to(t_truth, vel_ecef_tru, t_est)
    if acc_ecef_tru is not None:
        acc_ecef_tru = interp_to(t_truth, acc_ecef_tru, t_est)
    if q_hist is not None:
        BODY_est = (rotate_series_by_quat_series(pos_ned_est, q_hist, True),
                    rotate_series_by_quat_series(vel_ned_est, q_hist, True),
                    rotate_series_by_quat_series(acc_ned_est, q_hist, True))
    elif q_b2n_const is not None:
        q_const = np.repeat([q_b2n_const], len(t_est), axis=0)
        BODY_est = (rotate_series_by_quat_series(pos_ned_est, q_const, True),
                    rotate_series_by_quat_series(vel_ned_est, q_const, True),
                    rotate_series_by_quat_series(acc_ned_est, q_const, True))
    else:
        print('Warning: BODY-frame overlay using identity quaternion')
        BODY_est = (pos_ned_est, vel_ned_est, acc_ned_est)
    q_truth_hist = tru.get('q_b2n_truth')
    if q_truth_hist is not None:
        if len(q_truth_hist) != len(t_truth):
            q_truth_hist = slerp_series(t_truth, q_truth_hist, t_truth)
        q_truth_hist = slerp_series(t_truth, q_truth_hist, t_est)
        BODY_tru = (rotate_series_by_quat_series(pos_ned_tru, q_truth_hist, True),
                    rotate_series_by_quat_series(vel_ned_tru, q_truth_hist, True),
                    rotate_series_by_quat_series(acc_ned_tru, q_truth_hist, True))
    elif q_hist is not None or q_b2n_const is not None:
        q_use = q_hist if q_hist is not None else np.repeat([q_b2n_const], len(t_est), axis=0)
        BODY_tru = (rotate_series_by_quat_series(pos_ned_tru, q_use, True),
                    rotate_series_by_quat_series(vel_ned_tru, q_use, True),
                    rotate_series_by_quat_series(acc_ned_tru, q_use, True))
        print('Info: Truth BODY rotated using estimator quaternion')
    else:
        BODY_tru = (pos_ned_tru, vel_ned_tru, acc_ned_tru)
    def ensure_ecef(Pn, Vn, An):
        if Pn is None or lat_deg is None or lon_deg is None:
            return None, None, None
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        Pe = ned_to_ecef_vec(Pn, lat, lon)
        Ve = ned_to_ecef_vec(Vn, lat, lon) if Vn is not None else None
        Ae = ned_to_ecef_vec(An, lat, lon) if An is not None else None
        return Pe,Ve,Ae
    ECEF_est = (est.get('pos_ecef'), est.get('vel_ecef'), est.get('acc_ecef'))
    ECEF_tru = (pos_ecef_tru, vel_ecef_tru, acc_ecef_tru)
    if ECEF_est[0] is None and pos_ned_est is not None:
        ECEF_est = ensure_ecef(pos_ned_est, vel_ned_est, acc_ned_est)
    if ECEF_tru[0] is None and pos_ned_tru is not None:
        ECEF_tru = ensure_ecef(pos_ned_tru, vel_ned_tru, acc_ned_tru)
    NED_est = (pos_ned_est, vel_ned_est, acc_ned_est)
    NED_tru = (pos_ned_tru, vel_ned_tru, acc_ned_tru)
    files: List[str] = []
    if NED_est[0] is not None and NED_tru[0] is not None:
        f = str(outp / 'task6_overlay_NED.png')
        plot_overlay_3x3(t_est, NED_est, NED_tru, 'Task 6: Overlay (NED)',
                         ['Pos [m]','Vel [m/s]','Acc [m/s²]'], f)
        files.append(f)
    if ECEF_est[0] is not None and ECEF_tru[0] is not None:
        f = str(outp / 'task6_overlay_ECEF.png')
        plot_overlay_3x3(t_est, ECEF_est, ECEF_tru, 'Task 6: Overlay (ECEF)',
                         ['Pos [m]','Vel [m/s]','Acc [m/s²]'], f)
        files.append(f)
    if BODY_est[0] is not None and BODY_tru[0] is not None:
        f = str(outp / 'task6_overlay_BODY.png')
        plot_overlay_3x3(t_est, BODY_est, BODY_tru, 'Task 6: Overlay (BODY)',
                         ['Pos [m]','Vel [m/s]','Acc [m/s²]'], f)
        files.append(f)
    _update_manifest(manifest_path, files)
    return files

def run_task6_compare_methods_all_frames(*,
                                          method_files: Dict[str,str],
                                          truth_file: str,
                                          output_dir: str,
                                          lat_deg: float | None = None,
                                          lon_deg: float | None = None,
                                          gnss_file: str | None = None,
                                          q_b2n_const: Tuple[float,float,float,float] | None = None
                                          ) -> List[str]:
    outp = Path(output_dir)
    _ensure_dir(outp)
    manifest_path = outp / 'task6_overlay_manifest.json'
    if not method_files:
        return []
    first_name = next(iter(method_files))
    est_ref = load_estimates(method_files[first_name])
    t_ref = est_ref.get('time')
    if t_ref is None:
        raise ValueError('Estimator output lacks time array')
    q_ref = est_ref.get('q_b2n')
    if q_ref is not None:
        t_q_ref = est_ref.get('q_b2n_time', t_ref)
        if len(q_ref) != len(t_ref):
            q_ref = slerp_series(t_q_ref, q_ref, t_ref)
    tru = load_truth(truth_file)
    t_truth = tru.get('time', t_ref)
    pos_ned_tru = tru.get('pos_ned')
    vel_ned_tru = tru.get('vel_ned')
    acc_ned_tru = tru.get('acc_ned')
    pos_ecef_tru = tru.get('pos_ecef')
    vel_ecef_tru = tru.get('vel_ecef')
    acc_ecef_tru = tru.get('acc_ecef')
    if (pos_ned_tru is None and pos_ecef_tru is not None and lat_deg is not None and lon_deg is not None):
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        pos_ned_tru = ecef_to_ned_vec(pos_ecef_tru, lat, lon)
        if vel_ecef_tru is not None:
            vel_ned_tru = ecef_to_ned_vec(vel_ecef_tru, lat, lon)
    if pos_ned_tru is not None:
        pos_ned_tru = interp_to(t_truth, pos_ned_tru, t_ref)
    if vel_ned_tru is not None:
        vel_ned_tru = interp_to(t_truth, vel_ned_tru, t_ref)
    if acc_ned_tru is not None:
        acc_ned_tru = interp_to(t_truth, acc_ned_tru, t_ref)
    if pos_ecef_tru is not None:
        pos_ecef_tru = interp_to(t_truth, pos_ecef_tru, t_ref)
    if vel_ecef_tru is not None:
        vel_ecef_tru = interp_to(t_truth, vel_ecef_tru, t_ref)
    if acc_ecef_tru is not None:
        acc_ecef_tru = interp_to(t_truth, acc_ecef_tru, t_ref)
    q_truth_hist = tru.get('q_b2n_truth')
    if q_truth_hist is not None and len(q_truth_hist) != len(t_truth):
        q_truth_hist = slerp_series(t_truth, q_truth_hist, t_truth)
    if q_truth_hist is not None:
        q_truth_hist = slerp_series(t_truth, q_truth_hist, t_ref)
    methods_ned: Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]] = {}
    methods_ecef: Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]] = {}
    methods_body: Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]] = {}
    def ensure_ecef(Pn, Vn, An):
        if Pn is None or lat_deg is None or lon_deg is None:
            return None, None, None
        lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
        Pe = ned_to_ecef_vec(Pn, lat, lon)
        Ve = ned_to_ecef_vec(Vn, lat, lon) if Vn is not None else None
        Ae = ned_to_ecef_vec(An, lat, lon) if An is not None else None
        return Pe,Ve,Ae
    for name,path in method_files.items():
        est = load_estimates(path)
        t = est.get('time')
        if t is None:
            continue
        pos_ned = est.get('pos_ned')
        vel_ned = est.get('vel_ned')
        acc_ned = est.get('acc_ned')
        if ((pos_ned is None or vel_ned is None) and est.get('pos_ecef') is not None
                and lat_deg is not None and lon_deg is not None):
            lat = np.deg2rad(lat_deg); lon = np.deg2rad(lon_deg)
            if pos_ned is None:
                pos_ned = ecef_to_ned_vec(est['pos_ecef'], lat, lon)
            if vel_ned is None and est.get('vel_ecef') is not None:
                vel_ned = ecef_to_ned_vec(est['vel_ecef'], lat, lon)
            if acc_ned is None and est.get('acc_ecef') is not None:
                acc_ned = ecef_to_ned_vec(est['acc_ecef'], lat, lon)
        if len(t) != len(t_ref) or np.any(t != t_ref):
            if pos_ned is not None:
                pos_ned = interp_to(t, pos_ned, t_ref)
            if vel_ned is not None:
                vel_ned = interp_to(t, vel_ned, t_ref)
            if acc_ned is not None:
                acc_ned = interp_to(t, acc_ned, t_ref)
            if est.get('pos_ecef') is not None:
                est['pos_ecef'] = interp_to(t, est['pos_ecef'], t_ref)
            if est.get('vel_ecef') is not None:
                est['vel_ecef'] = interp_to(t, est['vel_ecef'], t_ref)
            if est.get('acc_ecef') is not None:
                est['acc_ecef'] = interp_to(t, est['acc_ecef'], t_ref)
            qh = est.get('q_b2n')
            if qh is not None:
                tq = est.get('q_b2n_time', t)
                qh = slerp_series(tq, qh, t_ref)
                est['q_b2n'] = qh
            t = t_ref
        qh = est.get('q_b2n')
        if qh is not None and len(qh) != len(t_ref):
            tq = est.get('q_b2n_time', t_ref)
            qh = slerp_series(tq, qh, t_ref)
        if qh is None and q_b2n_const is not None:
            qh = np.repeat([q_b2n_const], len(t_ref), axis=0)
        methods_ned[name] = (pos_ned, vel_ned, acc_ned)
        methods_ecef[name] = (est.get('pos_ecef'), est.get('vel_ecef'), est.get('acc_ecef'))
        methods_body[name] = (rotate_series_by_quat_series(pos_ned, qh, True) if qh is not None else pos_ned,
                              rotate_series_by_quat_series(vel_ned, qh, True) if qh is not None else vel_ned,
                              rotate_series_by_quat_series(acc_ned, qh, True) if qh is not None else acc_ned)
        if methods_ecef[name][0] is None and pos_ned is not None:
            methods_ecef[name] = ensure_ecef(pos_ned, vel_ned, acc_ned)
    if q_truth_hist is not None:
        BODY_tru = (rotate_series_by_quat_series(pos_ned_tru, q_truth_hist, True),
                    rotate_series_by_quat_series(vel_ned_tru, q_truth_hist, True),
                    rotate_series_by_quat_series(acc_ned_tru, q_truth_hist, True))
    elif q_ref is not None or q_b2n_const is not None:
        q_use = q_ref if q_ref is not None else np.repeat([q_b2n_const], len(t_ref), axis=0)
        BODY_tru = (rotate_series_by_quat_series(pos_ned_tru, q_use, True),
                    rotate_series_by_quat_series(vel_ned_tru, q_use, True),
                    rotate_series_by_quat_series(acc_ned_tru, q_use, True))
        print('Info: Truth BODY rotated using estimator quaternion')
    else:
        BODY_tru = (pos_ned_tru, vel_ned_tru, acc_ned_tru)
    NED_tru = (pos_ned_tru, vel_ned_tru, acc_ned_tru)
    ECEF_tru = (pos_ecef_tru, vel_ecef_tru, acc_ecef_tru)
    files: List[str] = []
    if any(v[0] is not None for v in methods_ned.values()) and NED_tru[0] is not None:
        f = str(outp / 'task6_methods_overlay_NED.png')
        plot_methods_overlay_3x3(t_ref, methods_ned, NED_tru,
                                 'Task 6: Methods Overlay (NED)',
                                 ['Pos [m]','Vel [m/s]','Acc [m/s²]'], f)
        files.append(f)
    if any(v[0] is not None for v in methods_ecef.values()) and ECEF_tru[0] is not None:
        f = str(outp / 'task6_methods_overlay_ECEF.png')
        plot_methods_overlay_3x3(t_ref, methods_ecef, ECEF_tru,
                                 'Task 6: Methods Overlay (ECEF)',
                                 ['Pos [m]','Vel [m/s]','Acc [m/s²]'], f)
        files.append(f)
    if any(v[0] is not None for v in methods_body.values()) and BODY_tru[0] is not None:
        f = str(outp / 'task6_methods_overlay_BODY.png')
        plot_methods_overlay_3x3(t_ref, methods_body, BODY_tru,
                                 'Task 6: Methods Overlay (BODY)',
                                 ['Pos [m]','Vel [m/s]','Acc [m/s²]'], f)
        files.append(f)
    _update_manifest(manifest_path, files)
    return files

