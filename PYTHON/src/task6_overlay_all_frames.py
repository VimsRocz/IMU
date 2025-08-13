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

    def pick(*names):
        for n in names:
            if n in data:
                return np.asarray(data[n])
        return None

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
        "q_b2n": pick("q_b2n", "quat_b2n"),
    }
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

        tcol = cols.get("time") or cols.get("t")
        out["time"] = df[tcol].to_numpy() if tcol else np.arange(len(df))
        out["pos_ecef"] = trip("pos_ecef", ("x", "y", "z"))
        out["vel_ecef"] = trip("vel_ecef", ("x", "y", "z"))
        out["pos_ned"] = trip("pos_ned", ("n", "e", "d"))
        out["vel_ned"] = trip("vel_ned", ("n", "e", "d"))
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


def quat_to_dcm(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    """Quaternion (body->NED) to direction cosine matrix."""

    q0, q1, q2, q3 = qw, qx, qy, qz
    return np.array(
        [
            [1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)],
        ]
    )


def ned_to_body(v_ned: np.ndarray, q_b2n: np.ndarray | Tuple[float, float, float, float]) -> np.ndarray:
    """Rotate NED vectors into the body frame.

    ``q_b2n`` may be a constant quaternion or an array of shape (N,4) providing a
    quaternion per sample.  In both cases the returned array matches the shape of
    ``v_ned``.
    """

    v = np.asarray(v_ned)
    q = np.asarray(q_b2n)
    if q.ndim == 1:
        Rb2n = quat_to_dcm(*q)
        Rn2b = Rb2n.T
        return (Rn2b @ v.T).T
    out = np.zeros_like(v)
    for i in range(len(v)):
        Rb2n = quat_to_dcm(*q[i])
        out[i] = Rb2n.T @ v[i]
    return out


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


def _build_frames(data: Dict[str, np.ndarray], lat_deg: float | None, lon_deg: float | None, q_b2n: np.ndarray | Tuple[float, float, float, float] | None) -> Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]:
    """Construct NED, ECEF and BODY triplets from raw data."""

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

    q = data.get("q_b2n")
    if q is None:
        q = q_b2n
    if q is None:
        q = (1.0, 0.0, 0.0, 0.0)
    body = (
        ned_to_body(pos_ned, q) if pos_ned is not None else None,
        ned_to_body(vel_ned, q) if vel_ned is not None else None,
        ned_to_body(acc_ned, q) if acc_ned is not None else None,
    )

    return {
        "NED": (pos_ned, vel_ned, acc_ned),
        "ECEF": (pos_ecef, vel_ecef, acc_ecef),
        "BODY": body,
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

    q = est.get("q_b2n") or q_b2n_const
    frames_est = _build_frames(est, lat_deg, lon_deg, q)
    frames_tru = _build_frames(tru, lat_deg, lon_deg, q)
    t_truth = tru.get("time", t)
    for trip in frames_tru.values():
        for i in range(3):
            if trip[i] is not None:
                trip[i] = interp_to(t_truth, trip[i], t)

    manifest: Dict[str, str] = {}
    for name in ["NED", "ECEF", "BODY"]:
        est_trip = frames_est[name]
        tru_trip = frames_tru[name]
        if est_trip[0] is None or tru_trip[0] is None:
            continue
        out = out_dir / f"task6_overlay_{name}.png"
        plot_overlay_3x3(t, est_trip, tru_trip, f"Task 6: Overlay ({name})", out)
        manifest[f"task6_overlay_{name}"] = str(out.resolve())

    manifest_path = out_dir / "task6_overlay_manifest.json"
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

    q_base = base.get("q_b2n") or q_b2n_const
    frames_truth = _build_frames(truth, lat_deg, lon_deg, q_base)
    t_truth = truth.get("time", t_base)
    for trip in frames_truth.values():
        for i in range(3):
            if trip[i] is not None:
                trip[i] = interp_to(t_truth, trip[i], t_base)

    methods_frames: Dict[str, Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]] = {}
    for name, path in method_files.items():
        est = load_estimates(path)
        t_est = est.get("time")
        if t_est is None:
            continue
        q_m = est.get("q_b2n") or q_base
        frames = _build_frames(est, lat_deg, lon_deg, q_m)
        for trip in frames.values():
            for i in range(3):
                if trip[i] is not None:
                    trip[i] = interp_to(t_est, trip[i], t_base)
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

