"""Helpers for Task 6 overlay plots across multiple reference frames.

This module focuses on defensive loading of estimator and truth files for the
Task‑6 overlay plots.  Data is parsed from ``.mat`` or ``.npz`` files using a
robust key search that tolerates MATLAB struct wrappers and differing
conventions.  NED, ECEF and BODY frame triplets (position/velocity/
acceleration) are extracted and, where possible, synthesised from one another.

Only static matplotlib is used so that the module can be imported in headless
environments.  Extensive debug printing can be enabled via the
``--debug-task6`` flag in :mod:`task6_plot_truth.py`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import scipy.io as sio

# ---------------------------------------------------------------------------
# Key definitions
# ---------------------------------------------------------------------------

TIME_KEYS = ["t", "time", "timestamp", "timestamps", "t_est", "time_s", "posix_time"]

# Preferred key sets for the various vector series we might encounter in the
# estimator output.  The first existing key is used.
KEYSETS = {
    "pos_ned": ["pos_ned_m", "fused_pos_ned", "pos_ned", "p_ned"],
    "vel_ned": ["vel_ned_ms", "fused_vel_ned", "vel_ned", "v_ned"],
    "acc_ned": ["acc_ned_mps2", "fused_acc_ned", "acc_ned", "a_ned"],
    "pos_ecef": ["pos_ecef_m", "fused_pos_ecef", "pos_ecef", "p_ecef"],
    "vel_ecef": ["vel_ecef_ms", "fused_vel_ecef", "vel_ecef", "v_ecef"],
    "acc_ecef": ["acc_ecef_mps2", "fused_acc_ecef", "acc_ecef", "a_ecef"],
    "pos_body": ["pos_body_m", "fused_pos_body", "pos_body", "p_body"],
    "vel_body": ["vel_body_ms", "fused_vel_body", "vel_body", "v_body"],
    "acc_body": ["acc_body_mps2", "fused_acc_body", "acc_body", "a_body"],
}


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def _dprint(debug: bool, msg: str) -> None:
    """Print ``msg`` if ``debug`` is true."""

    if debug:
        print(msg)


def _coerce_array(a: np.ndarray | object) -> np.ndarray:
    """Return a numeric :class:`numpy.ndarray` with MATLAB object wrappers removed."""

    if a is None:
        return None
    if isinstance(a, np.ndarray) and a.dtype == object:
        a = np.array([_coerce_array(x) for x in a])
    return np.asarray(a, dtype=float)


def extract_scalar(store: Dict[str, np.ndarray], keys: Iterable[str]) -> Optional[np.ndarray]:
    """Return the first scalar/1‑D array present in ``store`` using ``keys``."""

    for k in keys:
        if k in store:
            v = _coerce_array(store[k])
            v = np.squeeze(v)
            if v.ndim == 0:
                return float(v)
            if v.ndim == 1:
                return v.astype(float)
            if v.size >= 1:
                return v.reshape(-1).astype(float)
    return None


def extract_vec3(
    store: Dict[str, np.ndarray],
    prefer_keys: Iterable[str],
    name: str,
    time_len: Optional[int],
) -> Optional[np.ndarray]:
    """Robustly extract a 3‑vector series from ``store``.

    Handles MATLAB struct wrappers and varying shapes.  The returned array is
    ``float64`` with shape ``(N,3)``.  If ``time_len`` is provided, the array is
    truncated or zero padded to match this length and a warning is printed if
    resampling was necessary.
    """

    for key in prefer_keys:
        if key in store:
            arr = _coerce_array(store[key])
            arr = np.squeeze(arr)

            # Normalise to [N,3]
            if arr.ndim == 1:
                if arr.size == 3:
                    arr = np.tile(arr, (time_len if time_len else 1, 1))
                else:
                    continue
            elif arr.ndim == 2:
                if arr.shape[0] == 3 and arr.shape[1] != 3:
                    arr = arr.T
            elif arr.ndim == 3:
                if arr.shape[1] == 3:
                    arr = arr.reshape(arr.shape[0], 3)
                elif arr.shape[0] == 3:
                    arr = arr.reshape(3, -1).T

            if arr.ndim != 2 or arr.shape[1] != 3:
                continue

            n = arr.shape[0]
            if time_len is not None and n != time_len:
                if n > time_len:
                    arr = arr[:time_len]
                    print(
                        f"[Task6][WARN] Resampled {name} from len={n} to len={time_len} to match time base."
                    )
                else:
                    pad = np.zeros((time_len, 3))
                    pad[:n] = arr
                    arr = pad
                    print(
                        f"[Task6][WARN] Resampled {name} from len={n} to len={time_len} to match time base."
                    )
            return arr.astype(float)
    return None


# ---------------------------------------------------------------------------
# Frame conversions
# ---------------------------------------------------------------------------

def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
    sl, cl = np.sin(lat_rad), np.cos(lat_rad)
    so, co = np.sin(lon_rad), np.cos(lon_rad)
    return np.array(
        [[-sl * co, -sl * so, cl], [-so, co, 0.0], [-cl * co, -cl * so, -sl]]
    )


def ecef_to_ned(
    pos_ecef: np.ndarray, lat_rad: float, lon_rad: float, r0: np.ndarray
) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R @ (pos_ecef - r0).T).T


def ned_to_ecef(
    pos_ned: np.ndarray, lat_rad: float, lon_rad: float, r0: np.ndarray
) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R.T @ pos_ned.T).T + r0


def ecef_to_ned_vec(v_ecef: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R @ v_ecef.T).T


def ned_to_ecef_vec(v_ned: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
    R = R_ecef_to_ned(lat_rad, lon_rad)
    return (R.T @ v_ned.T).T


# ---------------------------------------------------------------------------
# Interpolation and plotting helpers
# ---------------------------------------------------------------------------

def interp_to(t_src: np.ndarray, X: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    out = np.zeros((len(t_dst), X.shape[1]))
    for i in range(X.shape[1]):
        out[:, i] = np.interp(t_dst, t_src, X[:, i])
    return out


def plot_overlay_3x3(
    time: np.ndarray,
    est_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    tru_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    title: str,
    outfile: Path,
    truth_missing: bool,
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
                ax.plot(
                    time,
                    est[:, i],
                    label="Fused" if (i == 0 and j == 0) else None,
                    linewidth=1.2,
                )
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
        if tru is None and truth_missing:
            axes[0, j].plot([], [], linestyle="--", label="Truth (missing)")
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, ncol=2, loc="upper center")
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    outfile.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


def plot_methods_overlay_3x3(
    time: np.ndarray,
    methods_triplets: Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]],
    tru_triplet: Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]],
    title: str,
    outfile: Path,
) -> None:
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
                    ax.plot(
                        time,
                        est[:, i],
                        label=(f"{name}" if (i == 0 and j == 0) else None),
                        linewidth=1.2,
                    )
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
    handles, labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, ncol=2, loc="upper center")
    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    outfile.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------------------
# File loading
# ---------------------------------------------------------------------------

def _load_any(path: str) -> Dict[str, np.ndarray]:
    p = Path(path)
    if p.suffix.lower() == ".npz":
        return {k: v for k, v in np.load(p).items()}
    data = sio.loadmat(p, squeeze_me=True)
    return {k: v for k, v in data.items() if not k.startswith("__")}


def load_truth(path: str) -> Dict[str, np.ndarray]:
    out: Dict[str, np.ndarray] = {}
    try:
        df = pd.read_csv(path, sep=None, engine="python")
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
        arr = np.loadtxt(path)
        out["time"] = arr[:, 0]
        if arr.shape[1] >= 7:
            out["pos_ecef"] = arr[:, 1:4]
            out["vel_ecef"] = arr[:, 4:7]
        return out


# ---------------------------------------------------------------------------
# Core helpers
# ---------------------------------------------------------------------------

def _resolve_time(data: Dict[str, np.ndarray]) -> Tuple[np.ndarray, str]:
    for k in TIME_KEYS:
        if k in data:
            t = np.asarray(data[k]).squeeze()
            if t.ndim == 1:
                return t.astype(float), k
    dt = extract_scalar(data, ["dt", "imu_dt", "delta_t"])
    n = None
    for ks in KEYSETS.values():
        arr = extract_vec3(data, ks, "tmp", None)
        if arr is not None:
            n = len(arr)
            break
    if n is None:
        raise ValueError("Estimator output lacks time array; no length info found.")
    if dt is None:
        dt = 0.01
    t = np.arange(n, dtype=float) * float(dt)
    print(f"[Task6] Reconstructed timebase using n={n}, dt={dt}")
    return t, "reconstructed"


def _derive_acc(debug: bool, vel: Optional[np.ndarray], dt: float) -> Optional[np.ndarray]:
    if vel is None:
        return None
    acc = np.diff(vel, axis=0, prepend=vel[0:1]) / float(dt)
    _dprint(debug, f"[Task6][DBG] Derived acc via finite difference (N={len(acc)}, dt={dt})")
    return acc


def _build_frames(
    store: Dict[str, np.ndarray],
    t: np.ndarray,
    lat: Optional[float],
    lon: Optional[float],
    r0: Optional[np.ndarray],
    debug: bool,
) -> Tuple[Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]], List[str]]:
    n = len(t)
    out = {name: extract_vec3(store, keys, name, n) for name, keys in KEYSETS.items()}
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    notes: List[str] = []

    if lat is not None and lon is not None and r0 is not None:
        if out["pos_ned"] is None and out["pos_ecef"] is not None:
            out["pos_ned"] = ecef_to_ned(out["pos_ecef"], lat, lon, r0)
            if out["vel_ned"] is None and out["vel_ecef"] is not None:
                out["vel_ned"] = ecef_to_ned_vec(out["vel_ecef"], lat, lon)
            if out["acc_ned"] is None and out["acc_ecef"] is not None:
                out["acc_ned"] = ecef_to_ned_vec(out["acc_ecef"], lat, lon)
            _dprint(debug, f"[Task6][DBG] Built NED from ECEF using reference lat={lat} lon={lon} r0={r0}")
            notes.append("NED built from ECEF")
        if out["pos_ecef"] is None and out["pos_ned"] is not None:
            out["pos_ecef"] = ned_to_ecef(out["pos_ned"], lat, lon, r0)
            if out["vel_ecef"] is None and out["vel_ned"] is not None:
                out["vel_ecef"] = ned_to_ecef_vec(out["vel_ned"], lat, lon)
            if out["acc_ecef"] is None and out["acc_ned"] is not None:
                out["acc_ecef"] = ned_to_ecef_vec(out["acc_ned"], lat, lon)
            _dprint(debug, f"[Task6][DBG] Built ECEF from NED using reference lat={lat} lon={lon} r0={r0}")
            notes.append("ECEF built from NED")

    if out["acc_ned"] is None:
        out["acc_ned"] = _derive_acc(debug, out["vel_ned"], dt)
        if out["acc_ned"] is not None:
            notes.append("acc_ned derived")
    if out["acc_ecef"] is None:
        out["acc_ecef"] = _derive_acc(debug, out["vel_ecef"], dt)
        if out["acc_ecef"] is not None:
            notes.append("acc_ecef derived")
    if out["acc_body"] is None:
        out["acc_body"] = _derive_acc(debug, out["vel_body"], dt)
        if out["acc_body"] is not None:
            notes.append("acc_body derived")

    if out["pos_body"] is None and out["vel_body"] is None:
        if out["pos_ned"] is not None or out["vel_ned"] is not None:
            print("[Task6][WARN] BODY missing; mirroring NED")
            out["pos_body"] = out["pos_ned"]
            out["vel_body"] = out["vel_ned"]
            out["acc_body"] = out["acc_body"] or out["acc_ned"]

    frames = {
        "NED": (out["pos_ned"], out["vel_ned"], out["acc_ned"]),
        "ECEF": (out["pos_ecef"], out["vel_ecef"], out["acc_ecef"]),
        "BODY": (out["pos_body"], out["vel_body"], out["acc_body"]),
    }
    return frames, notes


# ---------------------------------------------------------------------------
# Main overlay functions
# ---------------------------------------------------------------------------

def run_task6_overlay_all_frames(
    est_file: str,
    truth_file: str,
    output_dir: str,
    run_id: str,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    gnss_file: str | None = None,
    q_b2n_const: Tuple[float, float, float, float] | None = None,
    time_hint_path: str | None = None,
    debug: bool = False,
) -> Dict[str, object]:
    """Generate overlay plots for a single estimator output."""

    raw = _load_any(est_file)
    _dprint(debug, f"[Task6][DBG] Estimator file: {est_file}")
    _dprint(debug, f"[Task6][DBG] Estimator available keys: {sorted(raw.keys())}")

    t, time_desc = _resolve_time(raw)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.0
    _dprint(
        debug,
        f"[Task6][DBG] Time source: {time_desc} len={len(t)} t0={t[0]:.3f} t1={t[-1]:.3f} dt≈{dt:.6f}",
    )

    lat = lat_deg
    lon = lon_deg
    if lat is None:
        v = extract_scalar(raw, ["ref_lat_rad", "lat_rad", "lat"])
        if v is not None:
            lat = float(v if np.isscalar(v) else v[0])
    if lon is None:
        v = extract_scalar(raw, ["ref_lon_rad", "lon_rad", "lon"])
        if v is not None:
            lon = float(v if np.isscalar(v) else v[0])
    r0 = extract_scalar(raw, ["ref_r0_m", "r0", "ecef_ref"])
    if isinstance(r0, float):
        r0 = np.array([r0, 0.0, 0.0])

    frames_est, notes = _build_frames(raw, t, lat, lon, np.asarray(r0) if r0 is not None else None, debug)

    truth_raw = load_truth(truth_file)
    t_truth = truth_raw.get("time", t)
    frames_truth, _ = _build_frames(
        truth_raw,
        t_truth,
        lat,
        lon,
        np.asarray(r0) if r0 is not None else None,
        debug,
    )
    for fname, trip in frames_truth.items():
        trip_list = list(trip)
        for i in range(3):
            arr = trip_list[i]
            if arr is not None:
                trip_list[i] = interp_to(t_truth, arr, t)
        frames_truth[fname] = tuple(trip_list)

    ready = {
        name: bool(frames_est[name][0] is not None or frames_est[name][1] is not None)
        for name in ["NED", "ECEF", "BODY"]
    }
    print(f"[Task6] Ready to plot: NED={ready['NED']} ECEF={ready['ECEF']} BODY={ready['BODY']}")

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    manifest: Dict[str, object] = {
        "estimator_file": est_file,
        "time_source": time_desc,
        "frames": {"NED": {}, "ECEF": {}, "BODY": {}},
        "notes": notes,
    }

    saved_paths: List[str] = []
    for name in ["NED", "ECEF", "BODY"]:
        est_trip = frames_est[name]
        tru_trip = frames_truth.get(name, (None, None, None))
        est_has = est_trip[0] is not None or est_trip[1] is not None
        tru_has = tru_trip[0] is not None or tru_trip[1] is not None
        _dprint(
            debug,
            f"[Task6][DBG] {name} est pos shape: {None if est_trip[0] is None else est_trip[0].shape}  "
            f"vel: {None if est_trip[1] is None else est_trip[1].shape}  "
            f"acc: {None if est_trip[2] is None else est_trip[2].shape}",
        )
        _dprint(
            debug,
            f"[Task6][DBG] {name} truth pos shape: {None if tru_trip[0] is None else tru_trip[0].shape} "
            f"vel: {None if tru_trip[1] is None else tru_trip[1].shape} "
            f"acc: {None if tru_trip[2] is None else tru_trip[2].shape}",
        )
        manifest["frames"][name] = {
            "est": est_has,
            "truth": tru_has,
            "png": None,
        }
        if not est_has:
            print(
                f"[Task6][WARN] Skipped {name} because pos and vel were both missing after fallbacks."
            )
            continue
        outfile = out_dir / f"{run_id}_task6_overlay_{name}.png"
        plot_overlay_3x3(
            t,
            est_trip,
            tru_trip,
            f"Task 6: Overlay ({name})",
            outfile,
            not tru_has,
        )
        manifest["frames"][name]["png"] = str(outfile)
        saved_paths.append(str(outfile))

    manifest_path = out_dir / "task6_overlay_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))

    if saved_paths:
        print(f"[TASK 6] Plots saved to results/: {saved_paths}")

    return manifest


# ---------------------------------------------------------------------------


def run_task6_compare_methods_all_frames(
    method_files: Dict[str, str],
    truth_file: str,
    output_dir: str,
    run_id: str,
    lat_deg: float | None = None,
    lon_deg: float | None = None,
    gnss_file: str | None = None,
    q_b2n_const: Tuple[float, float, float, float] | None = None,
    time_hint_path: str | None = None,
    debug: bool = False,
) -> Dict[str, object]:
    """Overlay multiple methods against truth in all frames."""

    if not method_files:
        return {}

    first_path = next(iter(method_files.values()))
    base_raw = _load_any(first_path)
    t, time_desc = _resolve_time(base_raw)
    lat = lat_deg
    lon = lon_deg
    r0 = extract_scalar(base_raw, ["ref_r0_m", "r0", "ecef_ref"])
    if isinstance(r0, float):
        r0 = np.array([r0, 0.0, 0.0])

    truth_raw = load_truth(truth_file)
    t_truth = truth_raw.get("time", t)
    frames_truth, _ = _build_frames(
        truth_raw, t_truth, lat, lon, np.asarray(r0) if r0 is not None else None, debug
    )
    for fname, trip in frames_truth.items():
        trip_list = list(trip)
        for i in range(3):
            arr = trip_list[i]
            if arr is not None:
                trip_list[i] = interp_to(t_truth, arr, t)
        frames_truth[fname] = tuple(trip_list)

    methods_frames: Dict[str, Dict[str, Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]]] = {}
    for name, path in method_files.items():
        raw = _load_any(path)
        frames, _ = _build_frames(
            raw, t, lat, lon, np.asarray(r0) if r0 is not None else None, debug
        )
        methods_frames[name] = frames

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    run_id = out_dir.parent.name
    saved_paths: List[str] = []

    for frame in ["NED", "ECEF", "BODY"]:
        methods_triplets = {
            m: frames[frame]
            for m, frames in methods_frames.items()
            if frames[frame][0] is not None or frames[frame][1] is not None
        }
        if not methods_triplets:
            print(
                f"[Task6][WARN] Skipped {frame} because pos and vel were both missing after fallbacks."
            )
            continue
        outfile = out_dir / f"{run_id}_task6_compare_methods_{frame}.png"
        plot_methods_overlay_3x3(
            t,
            methods_triplets,
            frames_truth.get(frame, (None, None, None)),
            f"Task 6: Methods Overlay ({frame})",
            outfile,
        )
        saved_paths.append(str(outfile))

    if saved_paths:
        print(f"[TASK 6] Plots saved to results/: {saved_paths}")

    return {"compare_methods_outputs": saved_paths, "time_source": time_desc}


