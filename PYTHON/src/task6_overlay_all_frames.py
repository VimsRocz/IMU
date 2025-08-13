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
# Quaternion helpers
# ---------------------------------------------------------------------------


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion array along the last axis."""

    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def quat_make_hemisphere_continuous(q: np.ndarray) -> np.ndarray:
    """Ensure quaternion sequence is sign-continuous."""

    q = np.asarray(q, dtype=float)
    if q.ndim == 1:
        return q
    out = q.copy()
    for i in range(1, len(out)):
        if np.dot(out[i], out[i - 1]) < 0:
            out[i] *= -1
    return out


def slerp_series(t_src: np.ndarray, q_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Interpolate quaternion time series from ``t_src`` to ``t_dst``."""

    q_src = quat_make_hemisphere_continuous(quat_normalize(q_src))
    try:  # pragma: no cover - relies on SciPy
        from scipy.spatial.transform import Rotation, Slerp

        rot = Rotation.from_quat(q_src[:, [1, 2, 3, 0]])
        slerp = Slerp(t_src, rot)
        rot_dst = slerp(t_dst)
        q_xyzw = rot_dst.as_quat()
        q_dst = q_xyzw[:, [3, 0, 1, 2]]
    except Exception:  # pragma: no cover - graceful fallback
        q_dst = np.column_stack(
            [np.interp(t_dst, t_src, q_src[:, i]) for i in range(q_src.shape[1])]
        )
        q_dst = quat_normalize(q_dst)
        q_dst = quat_make_hemisphere_continuous(q_dst)
    return q_dst


def quat_to_dcm_batch(q: np.ndarray) -> np.ndarray:
    """Convert quaternions ``[N,4]`` to direction cosine matrices ``[N,3,3]``."""

    q = quat_normalize(q)
    qw, qx, qy, qz = q.T
    r00 = 1 - 2 * (qy * qy + qz * qz)
    r01 = 2 * (qx * qy - qw * qz)
    r02 = 2 * (qx * qz + qw * qy)
    r10 = 2 * (qx * qy + qw * qz)
    r11 = 1 - 2 * (qx * qx + qz * qz)
    r12 = 2 * (qy * qz - qw * qx)
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 1 - 2 * (qx * qx + qy * qy)
    return np.stack(
        [
            np.stack([r00, r01, r02], axis=-1),
            np.stack([r10, r11, r12], axis=-1),
            np.stack([r20, r21, r22], axis=-1),
        ],
        axis=-2,
    )


def rotate_series_by_quat_series(
    v_ned: np.ndarray, q_b2n: np.ndarray, to_body: bool = True
) -> np.ndarray:
    """Rotate vector series ``v_ned`` using per-sample quaternions ``q_b2n``."""

    if v_ned is None or q_b2n is None:
        return None
    Rb2n = quat_to_dcm_batch(q_b2n)
    if to_body:
        R = np.transpose(Rb2n, (0, 2, 1))  # R_n2b
    else:
        R = Rb2n
    return np.einsum("nij,nj->ni", R, v_ned)


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------


def load_estimates(est_file: str) -> Dict[str, np.ndarray]:
    """Load estimator output from ``.mat`` or ``.npz`` files."""

    p = Path(est_file)
    if p.suffix == ".npz":
        data = dict(np.load(p))
    else:
        data = {
            k: v
            for k, v in sio.loadmat(p, squeeze_me=True).items()
            if not k.startswith("__")
        }

    data_lower = {k.lower(): v for k, v in data.items()}

    def pick(*names):
        for n in names:
            v = data_lower.get(n.lower())
            if v is not None:
                return np.asarray(v)
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
        "time_q": pick("time_q", "t_q", "time_quat", "t_quat", "quat_time"),
    }

    q_raw = None
    q_key = None
    for k in data_lower:
        if k in {"q_b2n", "quat_b2n", "qbn", "q_b2ned", "quatbn", "quats"}:
            q_raw = np.asarray(data_lower[k])
            q_key = k
            break

    if q_raw is not None:
        q_raw = q_raw.reshape(-1, 4)
        if "wxyz" in q_key:
            q = q_raw
        else:
            if np.mean(np.abs(q_raw[:, 0])) < np.mean(np.abs(q_raw[:, 3])):
                q = q_raw[:, [3, 0, 1, 2]]
            else:
                q = q_raw
        q = quat_make_hemisphere_continuous(quat_normalize(q))
        out["q_b2n"] = q
    else:
        out["q_b2n"] = None

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

        def quat_pref(prefix: str) -> Optional[np.ndarray]:
            names_wxyz = [f"{prefix}_{c}" for c in ("w", "x", "y", "z")]
            names_xyzw = [f"{prefix}_{c}" for c in ("x", "y", "z", "w")]
            if all(n in cols for n in names_wxyz):
                return df[[cols[n] for n in names_wxyz]].to_numpy()
            if all(n in cols for n in names_xyzw):
                arr = df[[cols[n] for n in names_xyzw]].to_numpy()
                return arr[:, [3, 0, 1, 2]]
            return None

        q = None
        for pref in ["q_b2n_truth", "q_b2n", "quat_b2n", "qbn", "q_b2ned", "quatbn", "quats"]:
            q = quat_pref(pref)
            if q is not None:
                break
        if q is None:
            if all(n in cols for n in ["qw", "qx", "qy", "qz"]):
                q = df[[cols[n] for n in ["qw", "qx", "qy", "qz"]]].to_numpy()
            elif all(n in cols for n in ["qx", "qy", "qz", "qw"]):
                arr = df[[cols[n] for n in ["qx", "qy", "qz", "qw"]]].to_numpy()
                q = arr[:, [3, 0, 1, 2]]
        if q is not None:
            q = quat_make_hemisphere_continuous(quat_normalize(q))
            out["q_b2n_truth"] = q
        return out
    except Exception:
        arr = np.loadtxt(truth_file)
        out["time"] = arr[:, 0]
        if arr.shape[1] >= 7:
            out["pos_ecef"] = arr[:, 1:4]
            out["vel_ecef"] = arr[:, 4:7]
        if arr.shape[1] >= 11:
            q = arr[:, 7:11]
            q = quat_make_hemisphere_continuous(quat_normalize(q))
            out["q_b2n_truth"] = q
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

    return quat_to_dcm_batch(np.array([[qw, qx, qy, qz]], dtype=float))[0]


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


def _build_frames(
    data: Dict[str, np.ndarray], lat_deg: float | None, lon_deg: float | None
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


def apply_body_rotation(
    ned_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    q_hist: Optional[np.ndarray],
    q_const: Optional[Tuple[float, float, float, float]],
    label: str = "est",
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """Rotate NED triplet into the body frame using either a quaternion history
    or a single constant quaternion."""

    pos, vel, acc = ned_triplet
    if q_hist is not None:
        pos_b = rotate_series_by_quat_series(pos, q_hist, True) if pos is not None else None
        vel_b = rotate_series_by_quat_series(vel, q_hist, True) if vel is not None else None
        acc_b = rotate_series_by_quat_series(acc, q_hist, True) if acc is not None else None
        return pos_b, vel_b, acc_b
    if q_const is not None:
        Rn2b = quat_to_dcm(*q_const).T
        pos_b = (Rn2b @ pos.T).T if pos is not None else None
        vel_b = (Rn2b @ vel.T).T if vel is not None else None
        acc_b = (Rn2b @ acc.T).T if acc is not None else None
        return pos_b, vel_b, acc_b
    if label == "est":  # pragma: no cover - warning path
        print("Warning: No quaternion provided; BODY frame equals NED.")
    return pos, vel, acc


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
    t_est = est.get("time")
    if t_est is None:
        raise ValueError("Estimator output lacks time array")

    q_hist = est.get("q_b2n")
    t_q = est.get("time_q") or t_est
    if q_hist is not None and len(q_hist) != len(t_est):
        q_hist = slerp_series(np.linspace(t_q[0], t_q[-1], len(q_hist)), q_hist, t_est)

    frames_est_base = _build_frames(est, lat_deg, lon_deg)

    frames_tru_base = _build_frames(tru, lat_deg, lon_deg)
    t_truth = tru.get("time")
    if t_truth is None:
        t_truth = t_est
    for trip in frames_tru_base.values():
        for i in range(3):
            if trip[i] is not None:
                trip[i] = interp_to(t_truth, trip[i], t_est)

    q_truth_hist = tru.get("q_b2n_truth")
    if q_truth_hist is not None and len(q_truth_hist) != len(t_truth):
        q_truth_hist = slerp_series(np.linspace(t_truth[0], t_truth[-1], len(q_truth_hist)), q_truth_hist, t_truth)
    if q_truth_hist is not None:
        q_truth_hist = slerp_series(t_truth, q_truth_hist, t_est)

    body_est = apply_body_rotation(frames_est_base["NED"], q_hist, q_b2n_const, label="est")
    if q_truth_hist is not None:
        body_tru = apply_body_rotation(frames_tru_base["NED"], q_truth_hist, None, label="truth")
    else:
        if q_hist is not None or q_b2n_const is not None:
            print("Note: Truth quaternion not provided; using estimator quaternion for BODY frame.")
        body_tru = apply_body_rotation(frames_tru_base["NED"], q_hist, q_b2n_const, label="truth")

    frames_est = {
        "NED": frames_est_base["NED"],
        "ECEF": frames_est_base["ECEF"],
        "BODY": body_est,
    }
    frames_tru = {
        "NED": frames_tru_base["NED"],
        "ECEF": frames_tru_base["ECEF"],
        "BODY": body_tru,
    }

    manifest: Dict[str, str] = {}
    manifest_path = out_dir / "task6_overlay_manifest.json"
    if manifest_path.exists():
        try:
            manifest = json.loads(manifest_path.read_text())
        except Exception:
            manifest = {}

    for name in ["NED", "ECEF", "BODY"]:
        est_trip = frames_est[name]
        tru_trip = frames_tru[name]
        if est_trip[0] is None or tru_trip[0] is None:
            continue
        out = out_dir / f"task6_overlay_{name}.png"
        plot_overlay_3x3(t_est, est_trip, tru_trip, f"Task 6: Overlay ({name})", out)
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

    first_path = next(iter(method_files.values()))
    base = load_estimates(first_path)
    t_base = base.get("time")
    if t_base is None:
        raise ValueError("Estimator output lacks time array")

    q_base_hist = base.get("q_b2n")
    t_q_base = base.get("time_q") or t_base
    if q_base_hist is not None and len(q_base_hist) != len(t_base):
        q_base_hist = slerp_series(np.linspace(t_q_base[0], t_q_base[-1], len(q_base_hist)), q_base_hist, t_base)

    frames_truth_base = _build_frames(truth, lat_deg, lon_deg)
    t_truth = truth.get("time")
    if t_truth is None:
        t_truth = t_base
    for trip in frames_truth_base.values():
        for i in range(3):
            if trip[i] is not None:
                trip[i] = interp_to(t_truth, trip[i], t_base)

    q_truth_hist = truth.get("q_b2n_truth")
    if q_truth_hist is not None and len(q_truth_hist) != len(t_truth):
        q_truth_hist = slerp_series(np.linspace(t_truth[0], t_truth[-1], len(q_truth_hist)), q_truth_hist, t_truth)
    if q_truth_hist is not None:
        q_truth_hist = slerp_series(t_truth, q_truth_hist, t_base)

    if q_truth_hist is not None:
        body_truth = apply_body_rotation(frames_truth_base["NED"], q_truth_hist, None, label="truth")
    else:
        if q_base_hist is not None or q_b2n_const is not None:
            print("Note: Truth quaternion not provided; using base estimator quaternion for BODY frame.")
        body_truth = apply_body_rotation(frames_truth_base["NED"], q_base_hist, q_b2n_const, label="truth")

    frames_truth = {
        "NED": frames_truth_base["NED"],
        "ECEF": frames_truth_base["ECEF"],
        "BODY": body_truth,
    }

    methods_frames: Dict[str, Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]] = {}
    for name, path in method_files.items():
        est = load_estimates(path)
        t_est = est.get("time")
        if t_est is None:
            continue
        q_hist = est.get("q_b2n")
        t_q = est.get("time_q") or t_est
        if q_hist is not None and len(q_hist) != len(t_est):
            q_hist = slerp_series(np.linspace(t_q[0], t_q[-1], len(q_hist)), q_hist, t_est)
        frames = _build_frames(est, lat_deg, lon_deg)
        for trip in frames.values():
            for i in range(3):
                if trip[i] is not None:
                    trip[i] = interp_to(t_est, trip[i], t_base)
        if q_hist is not None and len(q_hist) != len(t_base):
            q_hist = slerp_series(t_est, q_hist, t_base)
        body = apply_body_rotation(frames["NED"], q_hist, q_b2n_const, label="est")
        frames["BODY"] = body
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

