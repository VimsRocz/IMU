"""Helpers for Task 6 overlay plots across multiple reference frames.

The functions in this module operate purely on static matplotlib plots to
avoid importing the heavier interactive plotting stack unless explicitly
requested elsewhere.  They provide utilities to load estimator outputs and
truth files, perform basic frame conversions and generate consistent overlay
figures for NED, ECEF and BODY frames.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Dict, Tuple, Optional

import numpy as np
import pandas as pd
import scipy.io as sio
import matplotlib.pyplot as plt
import warnings


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion array along the last axis."""

    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def quat_make_hemisphere_continuous(q: np.ndarray) -> np.ndarray:
    """Ensure sign continuity for a quaternion sequence."""

    q = np.asarray(q, dtype=float).copy()
    for i in range(1, len(q)):
        if np.dot(q[i], q[i - 1]) < 0.0:
            q[i] *= -1.0
    return q


def slerp_series(t_src: np.ndarray, q_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Interpolate quaternion series using Slerp with linear fallback."""

    q_src = quat_make_hemisphere_continuous(quat_normalize(np.asarray(q_src)))
    t_src = np.asarray(t_src, dtype=float)
    t_dst = np.asarray(t_dst, dtype=float)
    try:  # pragma: no cover - SciPy optional
        from scipy.spatial.transform import Rotation, Slerp

        rot = Rotation.from_quat(np.column_stack([q_src[:, 1:], q_src[:, :1]]))
        slerp = Slerp(t_src, rot)
        interp = slerp(t_dst).as_quat()
        q_dst = np.column_stack([interp[:, 3], interp[:, 0], interp[:, 1], interp[:, 2]])
        return quat_make_hemisphere_continuous(quat_normalize(q_dst))
    except Exception:  # pragma: no cover - best effort
        q_lin = np.empty((len(t_dst), 4))
        for i in range(4):
            q_lin[:, i] = np.interp(t_dst, t_src, q_src[:, i])
        return quat_make_hemisphere_continuous(quat_normalize(q_lin))


def quat_to_dcm_batch(q: np.ndarray) -> np.ndarray:
    """Convert quaternions [N,4] to direction cosine matrices [N,3,3]."""

    q = quat_normalize(np.asarray(q))
    qw, qx, qy, qz = q.T
    dcm = np.empty((q.shape[0], 3, 3))
    dcm[:, 0, 0] = 1 - 2 * (qy * qy + qz * qz)
    dcm[:, 0, 1] = 2 * (qx * qy - qw * qz)
    dcm[:, 0, 2] = 2 * (qx * qz + qw * qy)
    dcm[:, 1, 0] = 2 * (qx * qy + qw * qz)
    dcm[:, 1, 1] = 1 - 2 * (qx * qx + qz * qz)
    dcm[:, 1, 2] = 2 * (qy * qz - qw * qx)
    dcm[:, 2, 0] = 2 * (qx * qz - qw * qy)
    dcm[:, 2, 1] = 2 * (qy * qz + qw * qx)
    dcm[:, 2, 2] = 1 - 2 * (qx * qx + qy * qy)
    return dcm


def rotate_series_by_quat_series(v_ned: np.ndarray, q_b2n: np.ndarray, to_body: bool = True) -> np.ndarray:
    """Rotate vector series using per-sample quaternions."""

    if v_ned is None or q_b2n is None:
        return None
    v = np.asarray(v_ned)
    q = np.asarray(q_b2n)
    if q.ndim == 1:
        q = np.repeat(q[np.newaxis, :], len(v), axis=0)
    dcm = quat_to_dcm_batch(q)
    if to_body:
        dcm = np.transpose(dcm, (0, 2, 1))
    return np.einsum("nij,nj->ni", dcm, v)


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------


def load_estimates(est_file: str) -> Dict[str, np.ndarray]:
    """Load estimator output from ``.mat`` or ``.npz`` files.

    The loader accepts a range of key names to cope with slightly differing
    conventions.  Returned arrays are converted to :class:`numpy.ndarray`
    instances.  Time is expected to be one-dimensional.
    """

    p = Path(est_file)
    if p.suffix == ".npz":
        data = dict(np.load(p))
    else:
        data = {
            k: v
            for k, v in sio.loadmat(p, squeeze_me=True).items()
            if not k.startswith("__")
        }

    keys = {k.lower(): k for k in data.keys()}

    def pick(*names):
        for n in names:
            k = keys.get(n.lower())
            if k:
                return np.asarray(data[k])
        return None

    def pick_with_key(*names):
        for n in names:
            k = keys.get(n.lower())
            if k:
                return np.asarray(data[k]), k
        return None, None

    out: Dict[str, np.ndarray] = {
        "time": pick("time", "t"),
        "pos_ned": pick("pos_ned"),
        "vel_ned": pick("vel_ned"),
        "acc_ned": pick("acc_ned"),
        "pos_ecef": pick("pos_ecef"),
        "vel_ecef": pick("vel_ecef"),
        "acc_ecef": pick("acc_ecef"),
        "pos_body": pick("pos_body"),
        "vel_body": pick("vel_body"),
        "acc_body": pick("acc_body"),
    }

    q_raw, q_key = pick_with_key(
        "q_b2n",
        "quat_b2n",
        "qbn",
        "q_b2ned",
        "quatbn",
        "quats",
        "quat_wxyz",
    )
    if q_raw is not None:
        q_arr = np.atleast_2d(q_raw)
        if not (q_key and "wxyz" in q_key.lower()):
            if np.mean(np.abs(q_arr[:, 0])) < np.mean(np.abs(q_arr[:, 3])):
                q_arr = q_arr[:, [3, 0, 1, 2]]
        q_arr = quat_make_hemisphere_continuous(quat_normalize(q_arr))
        out["q_b2n"] = q_arr

    t_q = pick("time_q", "t_q", "time_quat", "t_quat", "time_att", "t_att")
    if t_q is not None:
        out["time_q"] = np.asarray(t_q).ravel()

    return out


def load_truth(truth_file: str) -> Dict[str, np.ndarray]:
    """Parse truth files ``STATE_X*.txt`` (CSV or whitespace separated)."""

    out: Dict[str, np.ndarray] = {}
    try:
        df = pd.read_csv(truth_file, sep=None, engine="python")
        cols = {c.lower(): c for c in df.columns}

        def trip(prefix: str, keys: Tuple[str, str, str]) -> Optional[np.ndarray]:
            names = [f"{prefix}_{k}" for k in keys]
            if all(n in cols for n in names):
                return df[[cols[n] for n in names]].to_numpy()
            return None

        def pick_quat(prefixes: Tuple[str, ...]) -> Optional[np.ndarray]:
            for p in prefixes:
                names_wxyz = [f"{p}_{c}" for c in ("w", "x", "y", "z")]
                if all(n in cols for n in names_wxyz):
                    q = df[[cols[n] for n in names_wxyz]].to_numpy()
                    return quat_make_hemisphere_continuous(quat_normalize(q))
                names_xyzw = [f"{p}_{c}" for c in ("x", "y", "z", "w")]
                if all(n in cols for n in names_xyzw):
                    q = df[[cols[n] for n in names_xyzw]].to_numpy()
                    q = q[:, [3, 0, 1, 2]]
                    return quat_make_hemisphere_continuous(quat_normalize(q))
            return None

        tcol = cols.get("time") or cols.get("t")
        out["time"] = df[tcol].to_numpy() if tcol else np.arange(len(df))
        out["pos_ecef"] = trip("pos_ecef", ("x", "y", "z"))
        out["vel_ecef"] = trip("vel_ecef", ("x", "y", "z"))
        out["pos_ned"] = trip("pos_ned", ("n", "e", "d"))
        out["vel_ned"] = trip("vel_ned", ("n", "e", "d"))
        q_truth = pick_quat((
            "q_b2n_truth",
            "quat_b2n_truth",
            "q_b2n",
            "quat_b2n",
            "qbn",
            "quatbn",
            "quats",
            "quat",
        ))
        if q_truth is not None:
            out["q_b2n_truth"] = q_truth
        t_qcol = (
            cols.get("time_q")
            or cols.get("t_q")
            or cols.get("time_quat")
            or cols.get("t_quat")
        )
        if t_qcol is not None:
            out["time_q"] = df[t_qcol].to_numpy()
        return out
    except Exception:
        arr = np.loadtxt(truth_file)
        out["time"] = arr[:, 0]
        if arr.shape[1] >= 7:
            out["pos_ecef"] = arr[:, 1:4]
            out["vel_ecef"] = arr[:, 4:7]
        return out


def load_lat_lon_from_gnss(gnss_csv: str | None) -> Tuple[Optional[float], Optional[float]]:
    """Return average latitude/longitude from a GNSS CSV file."""

    if not gnss_csv:
        return None, None
    try:
        df = pd.read_csv(gnss_csv)
        lat = df["Latitude_deg"].mean() if "Latitude_deg" in df.columns else None
        lon = df["Longitude_deg"].mean() if "Longitude_deg" in df.columns else None
        return float(lat) if lat is not None else None, float(lon) if lon is not None else None
    except Exception:  # pragma: no cover - best effort
        return None, None


# ---------------------------------------------------------------------------
# Frame helpers
# ---------------------------------------------------------------------------


def R_ecef_to_ned(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat = np.deg2rad(lat_deg)
    lon = np.deg2rad(lon_deg)
    sL, cL = np.sin(lat), np.cos(lat)
    sO, cO = np.sin(lon), np.cos(lon)
    return np.array(
        [
            [-sL * cO, -sL * sO, cL],
            [-sO, cO, 0.0],
            [-cL * cO, -cL * sO, -sL],
        ]
    )


def ecef_to_ned_vec(v_ecef: np.ndarray, lat_deg: float, lon_deg: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_deg, lon_deg)
    return (R @ v_ecef.T).T


def ned_to_ecef_vec(v_ned: np.ndarray, lat_deg: float, lon_deg: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_deg, lon_deg)
    return (R.T @ v_ned.T).T


# ---------------------------------------------------------------------------
# Misc helpers
# ---------------------------------------------------------------------------


def interp_to(t_src: np.ndarray, X_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Piecewise-linear interpolation column-wise."""

    X_src = np.asarray(X_src)
    out = np.zeros((len(t_dst), X_src.shape[1]))
    for i in range(X_src.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, X_src[:, i])
    return out


def plot_overlay_3x3(
    time: np.ndarray,
    est_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    tru_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    title: str,
    outfile: str | Path,
) -> None:
    """Create a 3x3 overlay figure comparing estimate and truth."""

    comps = ["X", "Y", "Z"]
    ylabels = ["Position [m]", "Velocity [m/s]", "Acceleration [m/s²]"]
    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)
    for j in range(3):  # columns: pos/vel/acc
        est = est_triplet[j]
        tru = tru_triplet[j]
        for i in range(3):  # rows: components
            ax = axes[i, j]
            if est is not None:
                ax.plot(time, est[:, i], label="Estimated" if (i == 0 and j == 0) else None, linewidth=1.2)
            if tru is not None:
                ax.plot(
                    time,
                    tru[:, i],
                    linestyle="--",
                    label="Truth" if (i == 0 and j == 0) else None,
                    linewidth=1.0,
                )
            if j == 0:
                ax.set_ylabel(f"{comps[i]} {ylabels[j]}")
            if i == 2:
                ax.set_xlabel("Time [s]")
            ax.grid(alpha=0.3)
    axes[0, 0].set_title("Position")
    axes[0, 1].set_title("Velocity")
    axes[0, 2].set_title("Acceleration")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, ncol=3, loc="upper center")
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


def plot_methods_overlay_3x3(
    time: np.ndarray,
    methods_triplets: Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]],
    tru_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    title: str,
    outfile: str | Path,
) -> None:
    """Plot multiple methods against truth in a 3x3 grid."""

    comps = ["X", "Y", "Z"]
    ylabels = ["Position [m]", "Velocity [m/s]", "Acceleration [m/s²]"]
    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)
    for j in range(3):
        tru = tru_triplet[j]
        for i in range(3):
            ax = axes[i, j]
            for name, trip in methods_triplets.items():
                est = trip[j]
                if est is not None:
                    ax.plot(time, est[:, i], label=name if (i == 0 and j == 0) else None, linewidth=1.2)
            if tru is not None:
                ax.plot(
                    time,
                    tru[:, i],
                    linestyle="--",
                    label="Truth" if (i == 0 and j == 0) else None,
                    linewidth=1.0,
                )
            if j == 0:
                ax.set_ylabel(f"{comps[i]} {ylabels[j]}")
            if i == 2:
                ax.set_xlabel("Time [s]")
            ax.grid(alpha=0.3)
    axes[0, 0].set_title("Position")
    axes[0, 1].set_title("Velocity")
    axes[0, 2].set_title("Acceleration")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, ncol=4, loc="upper center")
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# High-level runners
# ---------------------------------------------------------------------------


def _build_ned_ecef(
    data: Dict[str, np.ndarray],
    lat_deg: float | None,
    lon_deg: float | None,
) -> Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]:
    """Construct NED and ECEF triplets from raw data."""

    lat = lat_deg
    lon = lon_deg
    pos_ned = data.get("pos_ned")
    vel_ned = data.get("vel_ned")
    acc_ned = data.get("acc_ned")

    if pos_ned is None and lat is not None and data.get("pos_ecef") is not None:
        pos_ned = ecef_to_ned_vec(data["pos_ecef"], lat, lon)
    if vel_ned is None and lat is not None and data.get("vel_ecef") is not None:
        vel_ned = ecef_to_ned_vec(data["vel_ecef"], lat, lon)
    if acc_ned is None and lat is not None and data.get("acc_ecef") is not None:
        acc_ned = ecef_to_ned_vec(data["acc_ecef"], lat, lon)

    pos_ecef = data.get("pos_ecef")
    vel_ecef = data.get("vel_ecef")
    acc_ecef = data.get("acc_ecef")
    if pos_ecef is None and lat is not None and pos_ned is not None:
        pos_ecef = ned_to_ecef_vec(pos_ned, lat, lon)
    if vel_ecef is None and lat is not None and vel_ned is not None:
        vel_ecef = ned_to_ecef_vec(vel_ned, lat, lon)
    if acc_ecef is None and lat is not None and acc_ned is not None:
        acc_ecef = ned_to_ecef_vec(acc_ned, lat, lon)

    return {
        "NED": (pos_ned, vel_ned, acc_ned),
        "ECEF": (pos_ecef, vel_ecef, acc_ecef),
    }


def run_task6_overlay_all_frames(
    est_file: str,
    truth_file: str,
    output_dir: str,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    gnss_file: str | None = None,
    q_b2n_const: Tuple[float, float, float, float] | None = None,
) -> None:
    """Generate single-method overlays for all frames."""

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if (lat_deg is None or lon_deg is None) and gnss_file:
        g_lat, g_lon = load_lat_lon_from_gnss(gnss_file)
        lat_deg = lat_deg if lat_deg is not None else g_lat
        lon_deg = lon_deg if lon_deg is not None else g_lon

    est = load_estimates(est_file)
    tru = load_truth(truth_file)
    t = est.get("time")
    if t is None:
        raise ValueError("Estimator output lacks time array")

    q_hist = est.get("q_b2n")
    if q_hist is not None:
        t_q = est.get("time_q")
        if q_hist.shape[0] != len(t):
            t_src = t_q if t_q is not None else np.linspace(t[0], t[-1], len(q_hist))
            q_hist = slerp_series(t_src, q_hist, t)

    frames_est = _build_ned_ecef(est, lat_deg, lon_deg)
    frames_tru = _build_ned_ecef(tru, lat_deg, lon_deg)

    t_truth = tru.get("time", t)
    for trip in frames_tru.values():
        for i in range(3):
            if trip[i] is not None:
                trip[i] = interp_to(t_truth, trip[i], t)

    q_truth = tru.get("q_b2n_truth")
    if q_truth is not None:
        t_q_truth = tru.get("time_q")
        if q_truth.shape[0] != len(t_truth):
            t_src = t_q_truth if t_q_truth is not None else np.linspace(t_truth[0], t_truth[-1], len(q_truth))
            q_truth = slerp_series(t_src, q_truth, t_truth)
        q_truth = slerp_series(t_truth, q_truth, t)

    if q_hist is not None:
        q_body = q_hist
    elif q_b2n_const is not None:
        q_body = np.repeat(np.asarray(q_b2n_const)[None, :], len(t), axis=0)
    else:
        q_body = None
        warnings.warn("No quaternion provided; BODY overlays mirror NED")

    if q_body is not None:
        body_est = (
            rotate_series_by_quat_series(frames_est["NED"][0], q_body, True) if frames_est["NED"][0] is not None else None,
            rotate_series_by_quat_series(frames_est["NED"][1], q_body, True) if frames_est["NED"][1] is not None else None,
            rotate_series_by_quat_series(frames_est["NED"][2], q_body, True) if frames_est["NED"][2] is not None else None,
        )
    else:
        body_est = frames_est["NED"]

    if q_truth is not None:
        q_tru_body = q_truth
    elif q_body is not None:
        q_tru_body = q_body
        print("Truth BODY rotated using estimator quaternion")
    else:
        q_tru_body = None

    if q_tru_body is not None:
        body_tru = (
            rotate_series_by_quat_series(frames_tru["NED"][0], q_tru_body, True) if frames_tru["NED"][0] is not None else None,
            rotate_series_by_quat_series(frames_tru["NED"][1], q_tru_body, True) if frames_tru["NED"][1] is not None else None,
            rotate_series_by_quat_series(frames_tru["NED"][2], q_tru_body, True) if frames_tru["NED"][2] is not None else None,
        )
    else:
        body_tru = frames_tru["NED"]

    frames_est_all = {"NED": frames_est["NED"], "ECEF": frames_est["ECEF"], "BODY": body_est}
    frames_tru_all = {"NED": frames_tru["NED"], "ECEF": frames_tru["ECEF"], "BODY": body_tru}

    manifest_path = out_dir / "task6_overlay_manifest.json"
    manifest: Dict[str, str] = {}
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())

    for name in ["NED", "ECEF", "BODY"]:
        est_trip = frames_est_all[name]
        tru_trip = frames_tru_all[name]
        if est_trip[0] is None or tru_trip[0] is None:
            continue
        out = out_dir / f"task6_overlay_{name}.png"
        plot_overlay_3x3(t, est_trip, tru_trip, f"Task 6: Overlay ({name})", out)
        manifest[f"task6_overlay_{name}"] = str(out.resolve())

    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)


def run_task6_compare_methods_all_frames(
    method_files: Dict[str, str],
    truth_file: str,
    output_dir: str,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    gnss_file: str | None = None,
    q_b2n_const: Tuple[float, float, float, float] | None = None,
) -> None:
    """Overlay multiple methods against truth for all frames."""

    if not method_files:
        return

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if (lat_deg is None or lon_deg is None) and gnss_file:
        g_lat, g_lon = load_lat_lon_from_gnss(gnss_file)
        lat_deg = lat_deg if lat_deg is not None else g_lat
        lon_deg = lon_deg if lon_deg is not None else g_lon

    truth = load_truth(truth_file)

    # Choose the first method as timebase
    first_path = next(iter(method_files.values()))
    base = load_estimates(first_path)
    t_base = base.get("time")
    if t_base is None:
        raise ValueError("Estimator output lacks time array")

    q_base = base.get("q_b2n")
    if q_base is not None:
        t_q_base = base.get("time_q")
        if q_base.shape[0] != len(t_base):
            t_src = t_q_base if t_q_base is not None else np.linspace(t_base[0], t_base[-1], len(q_base))
            q_base = slerp_series(t_src, q_base, t_base)
    elif q_b2n_const is not None:
        q_base = np.repeat(np.asarray(q_b2n_const)[None, :], len(t_base), axis=0)
    else:
        q_base = None

    frames_truth = _build_ned_ecef(truth, lat_deg, lon_deg)
    t_truth = truth.get("time", t_base)
    for trip in frames_truth.values():
        for i in range(3):
            if trip[i] is not None:
                trip[i] = interp_to(t_truth, trip[i], t_base)

    q_truth = truth.get("q_b2n_truth")
    if q_truth is not None:
        t_q_truth = truth.get("time_q")
        if q_truth.shape[0] != len(t_truth):
            t_src = t_q_truth if t_q_truth is not None else np.linspace(t_truth[0], t_truth[-1], len(q_truth))
            q_truth = slerp_series(t_src, q_truth, t_truth)
        q_truth = slerp_series(t_truth, q_truth, t_base)

    if q_truth is not None:
        q_tru_body = q_truth
    elif q_base is not None:
        q_tru_body = q_base
        print("Truth BODY rotated using estimator quaternion")
    else:
        q_tru_body = None

    if q_tru_body is not None:
        frames_truth["BODY"] = (
            rotate_series_by_quat_series(frames_truth["NED"][0], q_tru_body, True) if frames_truth["NED"][0] is not None else None,
            rotate_series_by_quat_series(frames_truth["NED"][1], q_tru_body, True) if frames_truth["NED"][1] is not None else None,
            rotate_series_by_quat_series(frames_truth["NED"][2], q_tru_body, True) if frames_truth["NED"][2] is not None else None,
        )
    else:
        frames_truth["BODY"] = frames_truth["NED"]

    methods_frames: Dict[str, Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]] = {}
    for name, path in method_files.items():
        est = load_estimates(path)
        t_est = est.get("time")
        if t_est is None:
            continue
        q_m = est.get("q_b2n")
        if q_m is not None:
            t_qm = est.get("time_q")
            if q_m.shape[0] != len(t_est):
                t_src = t_qm if t_qm is not None else np.linspace(t_est[0], t_est[-1], len(q_m))
                q_m = slerp_series(t_src, q_m, t_est)
            q_m_interp = slerp_series(t_est, q_m, t_base)
        elif q_b2n_const is not None:
            q_m_interp = np.repeat(np.asarray(q_b2n_const)[None, :], len(t_base), axis=0)
        else:
            q_m_interp = None

        frames = _build_ned_ecef(est, lat_deg, lon_deg)
        for trip in frames.values():
            for i in range(3):
                if trip[i] is not None:
                    trip[i] = interp_to(t_est, trip[i], t_base)
        if q_m_interp is not None:
            frames["BODY"] = (
                rotate_series_by_quat_series(frames["NED"][0], q_m_interp, True) if frames["NED"][0] is not None else None,
                rotate_series_by_quat_series(frames["NED"][1], q_m_interp, True) if frames["NED"][1] is not None else None,
                rotate_series_by_quat_series(frames["NED"][2], q_m_interp, True) if frames["NED"][2] is not None else None,
            )
        else:
            frames["BODY"] = frames["NED"]
        methods_frames[name] = frames

    manifest_path = out_dir / "task6_overlay_manifest.json"
    manifest = {}
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())

    for frame in ["NED", "ECEF", "BODY"]:
        methods_triplets = {m: f[frame] for m, f in methods_frames.items() if f[frame][0] is not None}
        if not methods_triplets:
            continue
        out = out_dir / f"task6_methods_overlay_{frame}.png"
        plot_methods_overlay_3x3(
            t_base,
            methods_triplets,
            frames_truth[frame],
            f"Task 6: Methods Overlay ({frame})",
            out,
        )
        manifest[f"task6_methods_overlay_{frame}"] = str(out.resolve())

    with manifest_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

