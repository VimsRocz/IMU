# Debug-enhanced validation script (2025-07-06)
import argparse
import os
import logging
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as R, Slerp
import matplotlib.pyplot as plt
from tabulate import tabulate

# Import helpers with robust fallback so this module works whether imported
# via package path (src) or run from sibling scripts.
try:
    from utils import compute_C_ECEF_to_NED, ecef_to_geodetic, zero_base_time  # type: ignore
    from utils import frames as _frames  # type: ignore
except Exception:  # pragma: no cover - fallback for direct script execution
    try:
        # Prefer package-relative import when available
        from .utils import compute_C_ECEF_to_NED, ecef_to_geodetic, zero_base_time  # type: ignore
        from .utils import frames as _frames  # type: ignore
    except Exception:
        # Ultimate fallback to legacy module directly
        from .utils_legacy import compute_C_ECEF_to_NED, ecef_to_geodetic, zero_base_time  # type: ignore
        # frames functions unavailable in legacy fallback; define minimal shims
        class _frames:
            @staticmethod
            def R_ecef_to_ned(lat_rad: float, lon_rad: float) -> np.ndarray:
                return compute_C_ECEF_to_NED(lat_rad, lon_rad)

            @staticmethod
            def ecef_vec_to_ned(vec_ecef: np.ndarray, lat_rad: float, lon_rad: float) -> np.ndarray:
                C = compute_C_ECEF_to_NED(lat_rad, lon_rad)
                v = np.asarray(vec_ecef)
                if v.ndim == 1:
                    return C @ v
                return (C @ v.T).T
R_ecef_to_ned = _frames.R_ecef_to_ned
ecef_vec_to_ned = _frames.ecef_vec_to_ned
try:
    from plot_overlay import plot_overlay
except Exception:  # pragma: no cover - package-relative import
    from .plot_overlay import plot_overlay
import pandas as pd
import re

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

__all__ = [
    "load_estimate",
    "assemble_frames",
    "validate_with_truth",
    "validate_ecef_only",
    "run_debug_checks",
]


def _ned_to_ecef(pos_ned, ref_lat, ref_lon, ref_ecef, vel_ned=None, debug=False):
    """Convert NED positions (and optionally velocities) to ECEF."""
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    C_n2e = C.T
    if debug:
        logging.debug("Conversion matrix (ECEF->NED):\n%s", C)
    pos_ecef = np.asarray(pos_ned) @ C_n2e + ref_ecef
    vel_ecef = None
    if vel_ned is not None:
        vel_ecef = np.asarray(vel_ned) @ C_n2e
    if debug and len(pos_ecef) > 0:
        logging.debug("First converted position: %s", pos_ecef[0])
    return pos_ecef, vel_ecef


def _enforce_continuity(q):
    """
    Enforce temporal continuity on quaternion series (prevent sign flips).
    Matches MATLAB's att_utils('enforce_continuity', q) functionality.
    """
    q = np.asarray(q, float)
    if q.ndim == 1:
        return q
    
    q_cont = q.copy()
    for k in range(1, len(q)):
        if np.dot(q_cont[k], q_cont[k-1]) < 0:
            q_cont[k] = -q_cont[k]
    return q_cont


def _quat_to_euler_zyx_deg_wxyz(q):
    """
    Convert quaternion wxyz to Euler ZYX (yaw, pitch, roll) in degrees.
    Uses the proven working formula from run_triad_only.py.
    This avoids SciPy coordinate frame issues.
    """
    q = np.asarray(q, float)
    if q.ndim == 1:
        q = q.reshape(1, -1)
    
    w, x, y, z = q.T
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    s = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(s))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return np.vstack([yaw, pitch, roll]).T


def validate_with_truth(estimate_file, truth_file, dataset, convert_est_to_ecef=False, debug=False):
    """Compute error metrics between an estimate and ground truth.

    Parameters
    ----------
    estimate_file : str
        Path to the estimated state file (NPZ/MAT).
    truth_file : str
        CSV file with the ground truth trajectory.
    dataset : str
        Name of the dataset, used only for logging.
    convert_est_to_ecef : bool, optional
        When ``True``, convert the estimated trajectory from NED to ECEF and
        compare directly in the ECEF frame. By default, the truth data is
        converted to NED to match the estimate.
    debug : bool, optional
        Enable verbose debug logging.

    Returns
    -------
    tuple
        ``(rmse_pos, final_pos, rmse_vel, final_vel, rmse_eul, final_eul, rmse_acc, final_acc)``
        containing root-mean-square and final errors for position, velocity,
        and attitude. Acceleration errors are not evaluated (set to ``None``)
        because the truth data does not contain acceleration.
    """

    if debug:
        logging.getLogger().setLevel(logging.DEBUG)
        print(f"Debug: Loading truth data for {dataset} from {truth_file}")
    try:
        # truth files are whitespace separated
        truth = np.loadtxt(truth_file)
        if debug:
            print(f"Debug: Truth shape = {truth.shape}")
    except FileNotFoundError:
        print(
            f"Warning: Truth file {truth_file} not found, skipping validation for {dataset}"
        )
        return None, None, None, None, None, None

    # PairGuard: ensure dataset IDs match (IMU/GNSS/Truth). For this validator,
    # we at least check the truth file matches the dataset tag if present.
    try:
        import re
        did = re.search(r"X(\d{3})", str(truth_file), flags=re.IGNORECASE)
        if did and isinstance(dataset, str):
            dd = re.search(r"X(\d{3})", dataset, flags=re.IGNORECASE)
            if dd and did.group(1) != dd.group(1):
                print(
                    f"[PairGuard] WARN: dataset tag ({dataset}) and truth file ({truth_file}) IDs differ."
                )
    except Exception:
        pass

    # load_estimate returns a dictionary with time, pos, vel and quaternion
    est = load_estimate(estimate_file)
    est_pos = np.asarray(est.get("pos"))
    est_vel = np.asarray(est.get("vel"))
    quat = est.get("quat")
    if quat is not None:
        est_eul = R.from_quat(np.asarray(quat)[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
    else:
        est_eul = np.zeros_like(est_pos)
    if debug:
        print(
            f"Debug: Estimate shapes - pos: {est_pos.shape}, vel: {est_vel.shape}, eul: {est_eul.shape}"
        )

    # Align time vectors
    truth_time = truth[:, 1]
    est_time = np.arange(0, len(est_pos) * 0.0025, 0.0025)
    if debug:
        print(
            f"Debug: truth time range {truth_time[0]:.3f}-{truth_time[-1]:.3f} s, "
            f"estimate time range {est_time[0]:.3f}-{est_time[-1]:.3f} s"
        )

    # Interpolate estimated samples to the truth timestamps
    pos_interp = np.array(
        [np.interp(truth_time, est_time, est_pos[:, i]) for i in range(3)]
    ).T
    vel_interp = np.array(
        [np.interp(truth_time, est_time, est_vel[:, i]) for i in range(3)]
    ).T
    eul_interp = np.array(
        [np.interp(truth_time, est_time, est_eul[:, i]) for i in range(3)]
    ).T

    C_NED_to_ECEF = np.eye(3)
    if convert_est_to_ecef:
        logging.info("Entering NED-to-ECEF conversion block.")
        # convert estimate to ECEF for comparison
        ref_lat = est.get("ref_lat")
        if ref_lat is None:
            ref_lat = est.get("ref_lat_rad")
        if ref_lat is None:
            ref_lat = est.get("lat0")

        ref_lon = est.get("ref_lon")
        if ref_lon is None:
            ref_lon = est.get("ref_lon_rad")
        if ref_lon is None:
            ref_lon = est.get("lon0")

        ref_r0 = est.get("ref_r0")
        if ref_r0 is None:
            ref_r0 = est.get("ref_r0_m")
        if ref_r0 is None:
            ref_r0 = est.get("r0")

        if ref_lat is None or ref_lon is None or ref_r0 is None:
            first_ecef = truth[0, 2:5]
            lat_deg, lon_deg, _ = ecef_to_geodetic(*first_ecef)
            ref_lat = np.deg2rad(lat_deg)
            ref_lon = np.deg2rad(lon_deg)
            ref_r0 = first_ecef
        else:
            ref_lat = float(np.asarray(ref_lat).squeeze())
            ref_lon = float(np.asarray(ref_lon).squeeze())
            ref_r0 = np.asarray(ref_r0).squeeze()
        if debug:
            logging.debug(
                "Reference parameters - lat_rad: %s, lon_rad: %s, r0_m: %s",
                ref_lat,
                ref_lon,
                ref_r0,
            )
            logging.debug("First truth ECEF position: %s", truth[0, 2:5])
            logging.debug("First estimate NED position: %s", pos_interp[0])
        est_pos_ecef, est_vel_ecef = _ned_to_ecef(
            pos_interp,
            ref_lat,
            ref_lon,
            ref_r0,
            vel_interp,
            debug=debug,
        )
        C_NED_to_ECEF = compute_C_ECEF_to_NED(ref_lat, ref_lon).T
        logging.info(
            "Converted estimate from NED to ECEF using reference parameters."
        )
        pos_err = est_pos_ecef - truth[:, 2:5]
        vel_err = est_vel_ecef - truth[:, 5:8]
    else:
        # default: convert truth to NED and compare in NED frame
        # Derive truth velocity from ECEF position at native 10 Hz, then rotate
        ref_ecef = truth[0, 2:5]
        lat_deg, lon_deg, _ = ecef_to_geodetic(*ref_ecef)
        C = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
        C_NED_to_ECEF = C.T
        pos_ecef_truth = truth[:, 2:5]
        # Prefer truth velocity columns when non-trivial; fallback to smoothed derivative of position
        use_cols = truth.shape[1] >= 8
        if use_cols:
            vel_cols = truth[:, 5:8]
            try:
                p95 = float(np.percentile(np.linalg.norm(vel_cols, axis=1), 95))
            except Exception:
                p95 = 0.0
            use_cols = p95 > 0.10
        if use_cols:
            vel_ecef = vel_cols
            logging.info(
                "[Truth] velocity source: columns; p95|v|=%.3f m/s", float(np.percentile(np.linalg.norm(vel_ecef, axis=1), 95))
            )
        else:
            try:
                from scipy.signal import savgol_filter
                # Derivative via Savitzky–Golay; infer dt from truth_time median
                dt = float(np.median(np.diff(truth_time))) if truth_time.size > 1 else 0.1
                win = 21 if len(truth_time) >= 21 else max(5, (len(truth_time) // 2) * 2 + 1)
                pos_s = savgol_filter(pos_ecef_truth, win, 3, axis=0, mode="interp")
                vel_ecef = np.gradient(pos_s, dt, axis=0)
                src = "d/dt(pos) (Savitzky–Golay)"
            except Exception:
                vel_ecef = np.gradient(pos_ecef_truth, truth_time, axis=0)
                src = "d/dt(pos)"
            try:
                p95v = float(np.percentile(np.linalg.norm(vel_ecef, axis=1), 95))
                logging.info("[Truth] velocity source: %s; p95|v|=%.3f m/s", src, p95v)
            except Exception:
                pass
        truth_pos_ned = np.array([C @ (p - ref_ecef) for p in pos_ecef_truth])
        truth_vel_ned = np.array([C @ v for v in vel_ecef])

        pos_err = pos_interp - truth_pos_ned
        vel_err = vel_interp - truth_vel_ned

    truth_quat = truth[:, 8:12]
    truth_eul = R.from_quat(truth_quat[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
    eul_err = eul_interp - truth_eul

    rmse_pos = np.sqrt(np.mean(np.linalg.norm(pos_err, axis=1) ** 2))
    final_pos = np.linalg.norm(pos_err[-1, :])
    rmse_vel = np.sqrt(np.mean(np.linalg.norm(vel_err, axis=1) ** 2))
    final_vel = np.linalg.norm(vel_err[-1, :])

    # Acceleration comparison intentionally omitted: truth has no acceleration.
    rmse_acc = None
    final_acc = None

    rmse_eul = np.sqrt(np.mean(np.linalg.norm(eul_err, axis=1) ** 2))
    final_eul = np.linalg.norm(eul_err[-1, :])

    if debug:
        print(
            f"Debug: {dataset} - RMSE pos={rmse_pos:.3f} m, Final pos={final_pos:.3f} m, "
            f"RMSE vel={rmse_vel:.3f} m/s, Final vel={final_vel:.3f} m/s, "
            f"RMSE eul={rmse_eul:.3f}\u00b0, Final eul={final_eul:.3f}\u00b0"
        )

    return (
        rmse_pos,
        final_pos,
        rmse_vel,
        final_vel,
        rmse_eul,
        final_eul,
        rmse_acc,
        final_acc,
    )


def validate_ecef_only(est_file, truth_file, debug=False):
    """Validate only ECEF position and velocity between estimate and truth.

    The estimate is expected to contain ``pos_ecef_m`` and ``vel_ecef_ms``.
    Legacy fields ``pos_ecef`` and ``vel_ecef`` are also accepted.
    """

    truth = np.loadtxt(truth_file)
    t_truth = truth[:, 1]
    pos_truth = truth[:, 2:5]
    vel_truth_cols = truth[:, 5:8]

    if est_file.endswith(".mat"):
        data = loadmat(est_file)
    else:
        data = np.load(est_file, allow_pickle=True)

    pos_est = data.get("pos_ecef_m")
    vel_est = data.get("vel_ecef_ms")
    if pos_est is None or vel_est is None:
        # fall back to legacy keys if needed
        pos_est = pos_est if pos_est is not None else data.get("pos_ecef")
        vel_est = vel_est if vel_est is not None else data.get("vel_ecef")
    if pos_est is None or vel_est is None:
        raise KeyError("pos_ecef_m/vel_ecef_ms not found in estimate file")

    pos_est = np.asarray(pos_est)
    vel_est = np.asarray(vel_est)
    t_est = data.get("time")
    if t_est is None:
        t_est = data.get("time_s")
    if t_est is None:
        t_est = data.get("time_residuals")
    if t_est is not None:
        t_est = np.asarray(t_est).squeeze()
    else:
        t_est = np.arange(len(pos_est)) * 0.0025

    n = min(len(t_est), len(pos_est))
    pos_est = pos_est[:n]
    vel_est = vel_est[:n]
    t_est = t_est[:n]

    pos_i = np.vstack([np.interp(t_truth, t_est, pos_est[:, i]) for i in range(3)]).T
    vel_i = np.vstack([np.interp(t_truth, t_est, vel_est[:, i]) for i in range(3)]).T
    # Truth velocity: prefer columns if non-trivial, else smoothed derivative
    use_cols = True
    try:
        p95 = float(np.percentile(np.linalg.norm(vel_truth_cols, axis=1), 95))
        use_cols = p95 > 0.10
    except Exception:
        use_cols = False
    if not use_cols:
        try:
            from scipy.signal import savgol_filter
            dt = float(np.median(np.diff(t_truth))) if t_truth.size > 1 else 0.1
            pos_s = savgol_filter(pos_truth, 21 if len(t_truth) >= 21 else max(5, (len(t_truth)//2)*2+1), 3, axis=0, mode="interp")
            vel_truth = np.gradient(pos_s, dt, axis=0)
        except Exception:
            vel_truth = np.gradient(pos_truth, t_truth, axis=0)
    else:
        vel_truth = vel_truth_cols

    pos_err = pos_i - pos_truth
    vel_err = vel_i - vel_truth

    final_pos_error = np.linalg.norm(pos_err[-1])
    rmse_pos = np.sqrt(np.mean(np.sum(pos_err ** 2, axis=1)))
    final_vel_error = np.linalg.norm(vel_err[-1])
    rmse_vel = np.sqrt(np.mean(np.sum(vel_err ** 2, axis=1)))

    print(f"Final ECEF position error = {final_pos_error:.2f} m")
    print(f"RMSE ECEF position error = {rmse_pos:.2f} m")
    print(f"Final ECEF velocity error = {final_vel_error:.2f} m/s")
    print(f"RMSE ECEF velocity error = {rmse_vel:.2f} m/s")

    if debug:
        print(
            f"Debug: RMSE pos={rmse_pos:.3f} m, final pos={final_pos_error:.3f} m, "
            f"RMSE vel={rmse_vel:.3f} m/s, final vel={final_vel_error:.3f} m/s"
        )

    return rmse_pos, final_pos_error, rmse_vel, final_vel_error


def run_debug_checks(estimate_file, truth_file, dataset):
    """Run sanity checks on estimate and truth data.

    Parameters
    ----------
    estimate_file : str
        Path to the estimated state file.
    truth_file : str
        Path to the ground truth trajectory.
    dataset : str
        Name of the dataset used only for printing.
    """

    print("===== Debug Mode =====")
    print(f"Estimate file: {estimate_file}")
    print(f"Truth file: {truth_file}")
    print(f"Dataset: {dataset}")

    try:
        truth = np.loadtxt(truth_file, comments="#")
    except Exception as e:  # pragma: no cover - debug helper
        print(f"Failed to load truth data: {e}")
        return

    est = load_estimate(estimate_file)

    est_pos = np.asarray(est.get("pos"))
    est_vel = np.asarray(est.get("vel"))
    truth_time = truth[:, 1]

    est_time = est.get("time")
    if est_time is None or len(est_time) == 0:
        est_time = np.arange(len(est_pos)) * 0.0025
    else:
        est_time = np.asarray(est_time).squeeze()

    print("--- Data Consistency Check ---")
    print(
        f"Estimate samples: {len(est_time)}, Truth samples: {len(truth_time)}"
    )
    print(
        f"Estimate pos shape: {est_pos.shape}, vel shape: {est_vel.shape},"
        f" Truth shape: {truth.shape}"
    )
    print(
        f"Time ranges - est: {est_time[0]:.3f} to {est_time[-1]:.3f} s,"
        f" truth: {truth_time[0]:.3f} to {truth_time[-1]:.3f} s"
    )

    if est_pos.shape[0] == 0 or truth.shape[0] == 0:
        print("No data available to debug.")
        return

    print("--- Reference Frame Check ---")
    print("Estimate position sample:\n", est_pos[:3])
    print("Truth position sample (ECEF):\n", truth[:3, 2:5])
    if est.get("ref_r0") is not None:
        print("Estimate reference origin:", np.asarray(est.get("ref_r0")).squeeze())

    if np.linalg.norm(truth[0, 2:5]) > 1e6 and np.linalg.norm(est_pos[0]) < 1e5:
        print(
            "Warning: Estimate appears to be in NED while truth is in ECEF."
            " Check reference frame conversion."
        )
        return

    print("--- Units Check ---")
    for arr, name in [
        (est_pos, "Est Pos"),
        (truth[:, 2:5], "Truth Pos"),
        (est_vel, "Est Vel"),
        (truth[:, 5:8], "Truth Vel"),
    ]:
        mins = arr.min(axis=0)
        maxs = arr.max(axis=0)
        rng = maxs - mins
        print(f"{name} min {mins} max {maxs} range {rng}")

    print("--- Time Alignment ---")
    offset = truth_time[0] - est_time[0]
    print(f"Start time offset (truth-est): {offset:.3f} s")
    step_est = np.median(np.diff(est_time))
    step_truth = np.median(np.diff(truth_time))
    print(f"Mean dt estimate: {step_est:.4f} s, truth: {step_truth:.4f} s")
    if abs(offset) > 1.0:
        print("Warning: Large time offset between estimate and truth")
    if abs(step_est - step_truth) > 0.01:
        print("Warning: Sampling rates differ significantly")

    print("--- Plot Sanity ---")
    try:
        for i, lbl in enumerate(["X", "Y", "Z"]):
            plt.figure()
            plt.plot(est_time, est_pos[:, i], label="estimate")
            plt.plot(truth_time, truth[:, 2 + i], label="truth")
            plt.xlabel("time [s]")
            plt.ylabel(lbl)
            plt.legend()
            plt.tight_layout()
            from utils.matlab_fig_export import save_matlab_fig
            save_matlab_fig(fig_dbg, str(Path("results") / f"debug_{lbl}"))
            plt.close()
    except Exception as e:  # pragma: no cover - plotting helper
        print(f"Plot generation failed: {e}")

    print("--- Residuals/Errors ---")
    pos_interp = np.vstack(
        [np.interp(truth_time, est_time, est_pos[:, i]) for i in range(3)]
    ).T
    vel_interp = np.vstack(
        [np.interp(truth_time, est_time, est_vel[:, i]) for i in range(3)]
    ).T
    pos_err = pos_interp - truth[:, 2:5]
    vel_err = vel_interp - truth[:, 5:8]
    mean_pos = np.mean(np.linalg.norm(pos_err, axis=1))
    rmse_pos = np.sqrt(np.mean(np.sum(pos_err**2, axis=1)))
    mean_vel = np.mean(np.linalg.norm(vel_err, axis=1))
    rmse_vel = np.sqrt(np.mean(np.sum(vel_err**2, axis=1)))
    print(
        f"Mean position error {mean_pos:.3f} m, RMSE {rmse_pos:.3f} m;"
        f" Mean velocity error {mean_vel:.3f} m/s, RMSE {rmse_vel:.3f} m/s"
    )
    if rmse_pos > 10.0:
        print(
            "Warning: Position errors are very large (>10 m)."
            " Verify reference frame and units."
        )
    if rmse_vel > 5.0:
        print(
            "Warning: Velocity errors are very large (>5 m/s)."
            " Check time alignment and units."
        )



def load_estimate(path, times=None):
    """Return trajectory, quaternion and covariance from an NPZ or MAT file.

    Parameters
    ----------
    path : str
        Path to ``.npz`` or ``.mat`` file containing the filter output.
    times : array-like, optional
        When provided, position, velocity and quaternion samples are
        interpolated to this time vector.  The returned ``"time"`` entry will
        match ``times``.
    """

    def pick_key(keys, container, n_cols=None, default=None, quiet=False):
        """Return the first matching value or any array with *n_cols* columns."""
        for k in keys:
            if k in container:
                return container[k]
        if n_cols is not None:
            for k, v in container.items():
                if k.startswith("__"):
                    continue
                arr = np.asarray(v)
                if arr.ndim == 2 and arr.shape[1] == n_cols:
                    if not quiet:
                        logging.debug("Using '%s' for '%s'", k, keys[0] if keys else "?")
                    return v
        if not quiet:
            logging.debug(
                "Could not find any of %s. Available keys: %s",
                keys,
                list(container.keys()),
            )
        return default

    if path.endswith(".npz"):
        data = np.load(path, allow_pickle=True)
        t = pick_key(["time_residuals", "time", "time_s"], data)
        if t is not None:
            t = np.asarray(t).squeeze()
        pos = pick_key(["fused_pos", "pos_ned", "pos"], data)
        pos_found = pos is not None
        vel = pick_key(["fused_vel", "vel_ned", "vel"], data)
        quat = pick_key(["quat_log", "quat", "attitude_q"], data)
        if quat is None and "euler" in data:
            quat = R.from_euler("xyz", data["euler"], degrees=True).as_quat()
            quat = quat[:, [3, 0, 1, 2]]
        if pos is None and "residual_pos" in data and "innov_pos" in data:
            pos = data["residual_pos"] + data["innov_pos"]
        if vel is None and "residual_vel" in data and "innov_vel" in data:
            vel = data["residual_vel"] + data["innov_vel"]
        if pos is None:
            pos = pick_key([], data, n_cols=3)
        if vel is None:
            vel = pick_key([], data, n_cols=3)
        if quat is None:
            quat = pick_key([], data, n_cols=4)
        est = {
            "time": t,
            "pos": pos,
            "vel": vel,
            "quat": quat,
            "P": pick_key(["P", "P_hist"], data, quiet=True) if pos_found else None,
            "ref_lat": data.get("ref_lat")
            if data.get("ref_lat") is not None
            else data.get("ref_lat_rad")
            if data.get("ref_lat_rad") is not None
            else data.get("lat0"),
            "ref_lon": data.get("ref_lon")
            if data.get("ref_lon") is not None
            else data.get("ref_lon_rad")
            if data.get("ref_lon_rad") is not None
            else data.get("lon0"),
            "ref_r0": data.get("ref_r0")
            if data.get("ref_r0") is not None
            else data.get("ref_r0_m")
            if data.get("ref_r0_m") is not None
            else data.get("r0"),
            "truth_pos_ecef": data.get("truth_pos_ecef"),
            "truth_vel_ecef": data.get("truth_vel_ecef"),
            "truth_time": data.get("truth_time"),
        }
    else:
        m = loadmat(path)
        t = pick_key(["time_residuals", "time", "time_s"], m)
        if t is not None:
            t = np.asarray(t).squeeze()
        pos = pick_key(["fused_pos", "pos_ned", "pos"], m)
        pos_found = pos is not None
        vel = pick_key(["fused_vel", "vel_ned", "vel"], m)
        quat = pick_key(["quat_log", "quat", "attitude_q"], m)
        if quat is None and "euler" in m:
            quat = R.from_euler("xyz", m["euler"], degrees=True).as_quat()
            quat = quat[:, [3, 0, 1, 2]]
        if pos is None and "residual_pos" in m and "innov_pos" in m:
            pos = m["residual_pos"] + m["innov_pos"]
        if vel is None and "residual_vel" in m and "innov_vel" in m:
            vel = m["residual_vel"] + m["innov_vel"]
        if pos is None:
            pos = pick_key([], m, n_cols=3)
        if vel is None:
            vel = pick_key([], m, n_cols=3)
        if quat is None:
            quat = pick_key([], m, n_cols=4)
        est = {
            "time": t,
            "pos": pos,
            "vel": vel,
            "quat": quat,
            "P": pick_key(["P", "P_hist"], m, quiet=True) if pos_found else None,
            "ref_lat": m.get("ref_lat")
            if m.get("ref_lat") is not None
            else m.get("ref_lat_rad")
            if m.get("ref_lat_rad") is not None
            else m.get("lat0"),
            "ref_lon": m.get("ref_lon")
            if m.get("ref_lon") is not None
            else m.get("ref_lon_rad")
            if m.get("ref_lon_rad") is not None
            else m.get("lon0"),
            "ref_r0": m.get("ref_r0")
            if m.get("ref_r0") is not None
            else m.get("ref_r0_m")
            if m.get("ref_r0_m") is not None
            else m.get("r0"),
            "truth_pos_ecef": m.get("truth_pos_ecef"),
            "truth_vel_ecef": m.get("truth_vel_ecef"),
            "truth_time": m.get("truth_time"),
        }

    if est["time"] is None:
        try:
            est["time"] = np.loadtxt("STATE_X001.txt", comments="#", usecols=1)[
                : len(est["pos"])
            ]
        except OSError:
            est["time"] = np.arange(len(est["pos"]))

    if times is not None:
        times = np.asarray(times).squeeze()
        t_est = np.asarray(est["time"]).squeeze()
        if len(t_est) == 0:
            t_est = np.arange(len(est.get("pos", times)))
        if est.get("pos") is not None:
            pos = np.asarray(est["pos"])
            n = min(len(t_est), len(pos))
            pos = pos[:n]
            t_p = t_est[:n]
            pos_i = np.vstack(
                [np.interp(times, t_p, pos[:, i]) for i in range(pos.shape[1])]
            ).T
            est["pos"] = pos_i
        if est.get("vel") is not None:
            vel = np.asarray(est["vel"])
            n = min(len(t_est), len(vel))
            vel = vel[:n]
            t_v = t_est[:n]
            vel_i = np.vstack(
                [np.interp(times, t_v, vel[:, i]) for i in range(vel.shape[1])]
            ).T
            est["vel"] = vel_i
        if est.get("quat") is not None:
            quat = np.asarray(est["quat"])
            n = min(len(t_est), len(quat))
            t_q = t_est[:n]
            r_q = R.from_quat(quat[:n][:, [1, 2, 3, 0]])
            slerp = Slerp(t_q, r_q)
            r_i = slerp(np.clip(times, t_q[0], t_q[-1]))
            est["quat"] = r_i.as_quat()[:, [3, 0, 1, 2]]
        est["time"] = times

    return est


def assemble_frames(est, imu_file, gnss_file, truth_file=None):
    """Return aligned datasets in NED/ECEF/Body frames.

    When a truth trajectory is supplied, its time vector is synchronised to
    the estimator output via cross-correlation of position and velocity
    magnitudes before interpolation.

    Parameters
    ----------
    est : dict
        Output from :func:`load_estimate`.
    imu_file, gnss_file : str
        Raw data files used to generate *est*.
    truth_file : str or None, optional
        Path to ``STATE_X001.txt`` containing the reference trajectory. When
        provided, the returned frames include an additional ``"truth"``
        entry interpolated to the fused time vector.
    """
    gnss = pd.read_csv(gnss_file)
    t_gnss = zero_base_time(gnss["Posix_Time"].to_numpy())
    cols_pos = ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]
    cols_vel = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
    pos_ecef = gnss.loc[:, cols_pos].to_numpy()
    vel_ecef = gnss.loc[:, cols_vel].to_numpy()
    dt_g = np.diff(t_gnss, prepend=t_gnss[0])
    acc_ecef = np.zeros_like(vel_ecef)
    acc_ecef[1:] = np.diff(vel_ecef, axis=0) / dt_g[1:, None]

    ref_lat = est.get("ref_lat")
    if ref_lat is None:
        ref_lat = est.get("ref_lat_rad")
    if ref_lat is None:
        ref_lat = est.get("lat0")

    ref_lon = est.get("ref_lon")
    if ref_lon is None:
        ref_lon = est.get("ref_lon_rad")
    if ref_lon is None:
        ref_lon = est.get("lon0")

    ref_r0 = est.get("ref_r0")
    if ref_r0 is None:
        ref_r0 = est.get("ref_r0_m")
    if ref_r0 is None:
        ref_r0 = est.get("r0")
    if ref_lat is None or ref_lon is None or ref_r0 is None:
        if truth_file is not None:
            first_truth = np.loadtxt(truth_file, comments="#", max_rows=1)
            r0 = first_truth[2:5]
        else:
            r0 = pos_ecef[0]
        lat_deg, lon_deg, _ = ecef_to_geodetic(*r0)
        ref_lat = np.deg2rad(lat_deg)
        ref_lon = np.deg2rad(lon_deg)
        ref_r0 = r0
    else:
        ref_lat = float(np.asarray(ref_lat).squeeze())
        ref_lon = float(np.asarray(ref_lon).squeeze())
        ref_r0 = np.asarray(ref_r0).squeeze()
    C = R_ecef_to_ned(ref_lat, ref_lon)
    assert np.allclose(C @ C.T, np.eye(3), atol=1e-9), "R_ecef_to_ned not orthonormal"
    pos_gnss_ned = ecef_vec_to_ned(pos_ecef - ref_r0, ref_lat, ref_lon)
    vel_gnss_ned = ecef_vec_to_ned(vel_ecef, ref_lat, ref_lon)
    acc_gnss_ned = ecef_vec_to_ned(acc_ecef, ref_lat, ref_lon)

    t_est = zero_base_time(np.asarray(est["time"]).squeeze())
    fused_pos = np.asarray(est["pos"])
    fused_vel = np.asarray(est["vel"])
    n = min(len(t_est), len(fused_pos), len(fused_vel))
    t_est = t_est[:n]
    fused_pos = fused_pos[:n]
    fused_vel = fused_vel[:n]
    fused_acc = np.zeros_like(fused_vel)
    if len(t_est) > 1:
        dt = np.diff(t_est, prepend=t_est[0])
        fused_acc[1:] = np.diff(fused_vel, axis=0) / dt[1:, None]

    innov_pos = est.get("innov_pos")
    innov_vel = est.get("innov_vel")
    if innov_pos is not None and innov_vel is not None:
        gnss_pos_interp = np.vstack(
            [np.interp(t_est, t_gnss, pos_gnss_ned[:, i]) for i in range(3)]
        ).T
        gnss_vel_interp = np.vstack(
            [np.interp(t_est, t_gnss, vel_gnss_ned[:, i]) for i in range(3)]
        ).T
        imu_pos = gnss_pos_interp - innov_pos[: len(t_est)]
        imu_vel = gnss_vel_interp - innov_vel[: len(t_est)]
    else:
        imu_pos = fused_pos
        imu_vel = fused_vel
    imu_acc = np.zeros_like(imu_vel)
    if len(t_est) > 1:
        dt = np.diff(t_est, prepend=t_est[0])
        imu_acc[1:] = np.diff(imu_vel, axis=0) / dt[1:, None]

    C_N2E = C.T

    def ned_to_ecef(pos, vel, acc):
        p = (C_N2E @ pos.T).T + ref_r0
        v = (C_N2E @ vel.T).T
        a = (C_N2E @ acc.T).T
        return p, v, a

    imu_ecef = ned_to_ecef(imu_pos, imu_vel, imu_acc)
    fused_ecef = ned_to_ecef(fused_pos, fused_vel, fused_acc)
    gnss_ecef = (pos_ecef, vel_ecef, acc_ecef)

    # ------------------------------------------------------------------
    # Optional ground truth
    # ------------------------------------------------------------------
    truth_body = None
    t_truth = None
    pos_truth_ecef = None
    vel_truth_ecef = None
    if truth_file is not None:
        try:
            truth = np.loadtxt(truth_file, comments="#")
            t_truth = zero_base_time(truth[:, 1])
            pos_truth_ecef = truth[:, 2:5]
            # Derive truth velocity from truth position at native rate
            vel_truth_ecef = np.gradient(pos_truth_ecef, t_truth, axis=0)
        except Exception as e:
            print(f"Failed to load truth file {truth_file}: {e}")
            truth_file = None
    elif (
        est.get("truth_pos_ecef") is not None
        and est.get("truth_vel_ecef") is not None
        and est.get("truth_time") is not None
        and np.asarray(est["truth_pos_ecef"]).size > 0
    ):
        t_truth = zero_base_time(np.asarray(est["truth_time"]).squeeze())
        pos_truth_ecef = np.asarray(est["truth_pos_ecef"])
        vel_truth_ecef = np.asarray(est["truth_vel_ecef"])

    if t_truth is not None and pos_truth_ecef is not None:
        dt_r = max(np.mean(np.diff(t_est)), np.mean(np.diff(t_truth)))
        if not np.isfinite(dt_r) or dt_r <= 0:
            dt_r = max(t_est[-1] - t_est[0], t_truth[-1] - t_truth[0]) / max(
                len(t_est), len(t_truth), 1
            )
        t_start = min(t_est[0], t_truth[0])
        t_end = max(t_est[-1], t_truth[-1])
        max_steps = 1e6
        if (t_end - t_start) / dt_r > max_steps:
            dt_r = (t_end - t_start) / max_steps
        t_grid = np.arange(t_start, t_end + dt_r, dt_r)
        pos_est_rs = np.vstack(
            [np.interp(t_grid, t_est, fused_ecef[0][:, i]) for i in range(3)]
        ).T
        pos_truth_rs = np.vstack(
            [np.interp(t_grid, t_truth, pos_truth_ecef[:, i]) for i in range(3)]
        ).T
        vel_est_rs = np.vstack(
            [np.interp(t_grid, t_est, fused_ecef[1][:, i]) for i in range(3)]
        ).T
        vel_truth_rs = np.vstack(
            [np.interp(t_grid, t_truth, vel_truth_ecef[:, i]) for i in range(3)]
        ).T
        pos_norm_est = np.linalg.norm(pos_est_rs, axis=1)
        pos_norm_truth = np.linalg.norm(pos_truth_rs, axis=1)
        vel_norm_est = np.linalg.norm(vel_est_rs, axis=1)
        vel_norm_truth = np.linalg.norm(vel_truth_rs, axis=1)
        lags = np.arange(-len(pos_norm_truth) + 1, len(pos_norm_est))
        lag_pos = lags[
            np.argmax(
                np.correlate(
                    pos_norm_est - pos_norm_est.mean(),
                    pos_norm_truth - pos_norm_truth.mean(),
                    mode="full",
                )
            )
        ]
        lag_vel = lags[
            np.argmax(
                np.correlate(
                    vel_norm_est - vel_norm_est.mean(),
                    vel_norm_truth - vel_norm_truth.mean(),
                    mode="full",
                )
            )
        ]
        time_offset = 0.5 * (lag_pos + lag_vel) * dt_r
        max_offset_s = 5.0
        time_offset = float(np.clip(time_offset, -max_offset_s, max_offset_s))
        t_truth = t_truth + time_offset
        print(
            f"assemble_frames: applied time offset {time_offset:.3f} s via pos/vel alignment"
        )

        acc_truth_ecef = np.zeros_like(vel_truth_ecef)
        if len(t_truth) > 1:
            dt_t = np.diff(t_truth, prepend=t_truth[0])
            acc_truth_ecef[1:] = np.diff(vel_truth_ecef, axis=0) / dt_t[1:, None]
        pos_truth_ned = np.array([C @ (p - ref_r0) for p in pos_truth_ecef])
        vel_truth_ned = np.array([C @ v for v in vel_truth_ecef])
        acc_truth_ned = np.array([C @ a for a in acc_truth_ecef])

        def interp(arr):
            return np.vstack(
                [np.interp(t_est, t_truth, arr[:, i]) for i in range(3)]
            ).T

        pos_truth_ecef_i = interp(pos_truth_ecef)
        vel_truth_ecef_i = interp(vel_truth_ecef)
        acc_truth_ecef_i = interp(acc_truth_ecef)
        pos_truth_ned_i = interp(pos_truth_ned)
        vel_truth_ned_i = interp(vel_truth_ned)
        acc_truth_ned_i = interp(acc_truth_ned)
    else:
        pos_truth_ned = vel_truth_ned = acc_truth_ned = None
        pos_truth_ecef_i = vel_truth_ecef_i = acc_truth_ecef_i = None

    # --------------------------------------------------------------
    # Normalise all time vectors to a common origin for plotting
    # --------------------------------------------------------------
    t0_candidates = [t_gnss[0], t_est[0]]
    if t_truth is not None:
        t0_candidates.append(t_truth[0])
    t0 = float(np.min(t0_candidates))
    t_gnss = t_gnss - t0
    t_est = t_est - t0
    if t_truth is not None:
        t_truth = t_truth - t0

    q = est.get("quat")
    if q is not None:
        rot = R.from_quat(np.asarray(q)[: len(t_est)][:, [1, 2, 3, 0]])

        def to_body(pos, vel, acc):
            return rot.apply(pos), rot.apply(vel), rot.apply(acc)

        imu_body = to_body(imu_pos, imu_vel, imu_acc)
        fused_body = to_body(fused_pos, fused_vel, fused_acc)
        gnss_body = to_body(
            np.vstack(
                [np.interp(t_est, t_gnss, pos_gnss_ned[:, i]) for i in range(3)]
            ).T,
            np.vstack(
                [np.interp(t_est, t_gnss, vel_gnss_ned[:, i]) for i in range(3)]
            ).T,
            np.vstack(
                [np.interp(t_est, t_gnss, acc_gnss_ned[:, i]) for i in range(3)]
            ).T,
        )
        if t_truth is not None:
            truth_body = to_body(pos_truth_ned_i, vel_truth_ned_i, acc_truth_ned_i)
    else:
        imu_body = imu_pos, imu_vel, imu_acc
        fused_body = fused_pos, fused_vel, fused_acc
        gnss_body = (
            np.vstack(
                [np.interp(t_est, t_gnss, pos_gnss_ned[:, i]) for i in range(3)]
            ).T,
            np.vstack(
                [np.interp(t_est, t_gnss, vel_gnss_ned[:, i]) for i in range(3)]
            ).T,
            np.vstack(
                [np.interp(t_est, t_gnss, acc_gnss_ned[:, i]) for i in range(3)]
            ).T,
        )
        if t_truth is not None:
            truth_body = pos_truth_ned_i, vel_truth_ned_i, acc_truth_ned_i

    frames = {
        "NED": {
            "imu": (t_est, *(imu_pos, imu_vel, imu_acc)),
            "gnss": (t_gnss, pos_gnss_ned, vel_gnss_ned, acc_gnss_ned),
            "fused": (t_est, fused_pos, fused_vel, fused_acc),
        },
        "ECEF": {
            "imu": (t_est, *imu_ecef),
            "gnss": (t_gnss, *gnss_ecef),
            "fused": (t_est, *fused_ecef),
        },
        "Body": {
            "imu": (t_est, *imu_body),
            "gnss": (t_est, *gnss_body),
            "fused": (t_est, *fused_body),
        },
    }

    if t_truth is not None:
        frames["NED"]["truth"] = (
            t_est,
            pos_truth_ned_i,
            vel_truth_ned_i,
            acc_truth_ned_i,
        )
        frames["ECEF"]["truth"] = (
            t_est,
            pos_truth_ecef_i,
            vel_truth_ecef_i,
            acc_truth_ecef_i,
        )
        if truth_body is not None:
            frames["Body"]["truth"] = (t_est, *truth_body)

        def _corr(a, b):
            if a.ndim > 1:
                a = a.ravel()
            if b.ndim > 1:
                b = b.ravel()
            if np.std(a) == 0 or np.std(b) == 0:
                return 0.0
            return np.corrcoef(a, b)[0, 1]

        for name, f, t in [
            ("N vel North", fused_vel[:, 0], vel_truth_ned_i[:, 0]),
            ("N vel East", fused_vel[:, 1], vel_truth_ned_i[:, 1]),
            ("ECEF vel X", fused_ecef[1][:, 0], vel_truth_ecef_i[:, 0]),
            ("ECEF vel Y", fused_ecef[1][:, 1], vel_truth_ecef_i[:, 1]),
        ]:
            c = _corr(f, t)
            logger.info(f"[Sanity] corr({name}) = {c:+.3f}")

    return frames


def main():
    ap = argparse.ArgumentParser(
        description="Compare filter output with ground truth and plot error "
        "curves with optional ±3σ bounds."
    )
    ap.add_argument(
        "--est-file", required=True, help="NPZ or MAT file with filter results"
    )
    ap.add_argument(
        "--truth-file",
        required=True,
        help="CSV of true state (STATE_X001.txt)",
    )
    ap.add_argument(
        "--output", default="results", help="directory for the generated PDFs"
    )
    ap.add_argument(
        "--debug",
        action="store_true",
        help="run additional sanity checks on the input files",
    )
    ap.add_argument(
        "--convert-est-to-ecef",
        action="store_true",
        help="convert the estimate from NED to ECEF for comparison",
    )
    ap.add_argument(
        "--ecef-only",
        action="store_true",
        help="Validate only ECEF position/velocity fields",
    )
    # New CLI flags for divergence detection and length scan
    ap.add_argument(
        "--div-threshold-deg",
        type=float,
        default=30.0,
        help="Angle threshold in degrees to declare divergence (default 30.0)",
    )
    ap.add_argument(
        "--div-persist-sec",
        type=float,
        default=10.0,
        help="Minimum duration (s) the error must remain above threshold (default 10.0)",
    )
    ap.add_argument(
        "--length-scan",
        type=str,
        default="60,120,300,600,900,1200",
        help="Comma-separated max time values (s) for divergence-vs-length scan",
    )
    ap.add_argument("--ref-lat", type=float, help="reference latitude in degrees")
    ap.add_argument("--ref-lon", type=float, help="reference longitude in degrees")
    ap.add_argument("--ref-r0", type=float, nargs=3, help="ECEF origin [m]")
    args = ap.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    os.makedirs(args.output, exist_ok=True)
    logging.info("Ensured '%s' directory exists.", args.output)

    if args.ecef_only:
        validate_ecef_only(args.est_file, args.truth_file, debug=args.debug)
        return

    m_est = re.search(r"IMU_(X\d+)", os.path.basename(args.est_file))
    m_truth = re.search(r"STATE_(X\d+)", os.path.basename(args.truth_file))
    dataset_warning = None
    if m_est and m_truth and m_est.group(1) != m_truth.group(1):
        dataset_warning = (
            "Warning: estimate file appears to use dataset"
            f" {m_est.group(1)} but truth data is {m_truth.group(1)}."
            " Large errors are expected."
        )

    truth = np.loadtxt(args.truth_file)
    t_truth = truth[:, 1]
    pos_truth_ecef = truth[:, 2:5]
    dist_truth = np.zeros_like(t_truth)
    if len(dist_truth) > 1:
        dist_truth[1:] = np.cumsum(np.linalg.norm(np.diff(pos_truth_ecef, axis=0), axis=1))
    est = load_estimate(args.est_file, times=t_truth)

    # Extra debug information and quick validation metrics
    ds_name = m_truth.group(1) if m_truth else "unknown"
    validate_with_truth(
        args.est_file,
        args.truth_file,
        ds_name,
        convert_est_to_ecef=args.convert_est_to_ecef,
        debug=args.debug,
    )
    if args.debug:
        run_debug_checks(args.est_file, args.truth_file, ds_name)

    ref_lat = np.deg2rad(args.ref_lat) if args.ref_lat is not None else None
    ref_lon = np.deg2rad(args.ref_lon) if args.ref_lon is not None else None
    ref_r0 = np.array(args.ref_r0) if args.ref_r0 is not None else None

    if ref_lat is None:
        v = est.get("ref_lat")
        if v is None:
            v = est.get("ref_lat_rad")
        if v is None:
            v = est.get("lat0")
        if v is not None:
            ref_lat = float(np.asarray(v).squeeze())
    if ref_lon is None:
        v = est.get("ref_lon")
        if v is None:
            v = est.get("ref_lon_rad")
        if v is None:
            v = est.get("lon0")
        if v is not None:
            ref_lon = float(np.asarray(v).squeeze())
    if ref_r0 is None:
        v = est.get("ref_r0")
        if v is None:
            v = est.get("ref_r0_m")
        if v is None:
            v = est.get("r0")
        if v is not None:
            ref_r0 = np.asarray(v).squeeze()

    if ref_lat is None or ref_lon is None or ref_r0 is None:
        first_ecef = truth[0, 2:5]
        lat_deg, lon_deg, _ = ecef_to_geodetic(*first_ecef)
        ref_lat = np.deg2rad(lat_deg)
        ref_lon = np.deg2rad(lon_deg)
        ref_r0 = first_ecef

    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)

    truth_pos_ned = np.array([C @ (p - ref_r0) for p in pos_truth_ecef])

    err_pos = np.asarray(est["pos"]) - truth_pos_ned
    err_vel = None
    err_quat = None

    if est.get("vel") is not None:
        truth_vel_ned = np.array([C @ v for v in truth[:, 5:8]])
        err_vel = np.asarray(est["vel"]) - truth_vel_ned

    # Robust quaternion truth vs estimate comparison and divergence detection
    tag = (m_truth.group(1) if m_truth else (m_est.group(1) if m_est else "DATA"))
    if est.get("quat") is not None and truth.shape[1] >= 12:
        # Build shared time window
        t_est = np.asarray(est.get("time")).squeeze()
        if t_est is None or t_est.size == 0:
            t_est = np.arange(len(est.get("pos", []))) * 0.0025
        t_truth = np.asarray(t_truth).squeeze()
        t0 = max(float(t_truth[0]), float(t_est[0]))
        t1 = min(float(t_truth[-1]), float(t_est[-1]))
        if not (t1 > t0):
            logging.warning("No overlapping time window between estimate and truth for attitude comparison.")
        else:
            mask_est = (t_est >= t0) & (t_est <= t1)
            te_win = t_est[mask_est]

            # Candidate quaternion layouts: interpret arrays as wxyz (default) or xyzw
            q_truth_raw = np.asarray(truth[:, 8:12], float)
            q_est_raw = np.asarray(est["quat"], float)

            def as_xyzw_from_wxyz(qwxyz):
                qwxyz = np.asarray(qwxyz, float)
                return np.column_stack([qwxyz[:, 1], qwxyz[:, 2], qwxyz[:, 3], qwxyz[:, 0]])

            def as_wxyz_from_xyzw(qxyzw):
                qxyzw = np.asarray(qxyzw, float)
                return np.column_stack([qxyzw[:, 3], qxyzw[:, 0], qxyzw[:, 1], qxyzw[:, 2]])

            # Prepare Slerp for truth under both layout assumptions
            r_truth_wxyz = None
            r_truth_xyzw = None
            try:
                r_truth_wxyz = R.from_quat(as_xyzw_from_wxyz(q_truth_raw))
            except Exception:
                r_truth_wxyz = None
            try:
                r_truth_xyzw = R.from_quat(np.asarray(q_truth_raw, float))
            except Exception:
                r_truth_xyzw = None

            # Estimate candidates
            q_est_wxyz = None
            q_est_xyzw = None
            # Try interpreting as wxyz first
            try:
                _ = R.from_quat(as_xyzw_from_wxyz(q_est_raw))
                q_est_wxyz = q_est_raw.copy()
            except Exception:
                q_est_wxyz = None
            # Also allow xyzw
            try:
                _ = R.from_quat(np.asarray(q_est_raw, float))
                q_est_xyzw = q_est_raw.copy()
            except Exception:
                q_est_xyzw = None

            # Interpolate truth to estimate time via Slerp for both options
            cand = []  # list of dicts with qt_wxyz, qe_wxyz, ang_deg, euler arrays
            for truth_layout, r_truth in [("wxyz", r_truth_wxyz), ("xyzw", r_truth_xyzw)]:
                if r_truth is None:
                    continue
                slerp = Slerp(t_truth, r_truth)
                r_truth_i = slerp(np.clip(te_win, t_truth[0], t_truth[-1]))
                qt_xyzw = r_truth_i.as_quat()
                qt_wxyz = as_wxyz_from_xyzw(qt_xyzw)

                for est_layout, qraw in [("wxyz", q_est_wxyz), ("xyzw", q_est_xyzw)]:
                    if qraw is None:
                        continue
                    if est_layout == "wxyz":
                        qe_wxyz_full = np.asarray(qraw, float)
                    else:
                        qe_wxyz_full = as_wxyz_from_xyzw(np.asarray(qraw, float))
                    qe_wxyz = qe_wxyz_full[mask_est]

                    # FIX 1: Normalize quaternions first
                    qt = qt_wxyz / np.linalg.norm(qt_wxyz, axis=1, keepdims=True)
                    qe = qe_wxyz / np.linalg.norm(qe_wxyz, axis=1, keepdims=True)
                    
                    # FIX 2: Enforce temporal continuity first (like MATLAB's enforce_continuity)
                    qt = _enforce_continuity(qt)
                    qe = _enforce_continuity(qe)
                    
                    # FIX 3: Then do hemisphere alignment between truth and estimate
                    d = np.sum(qt * qe, axis=1)
                    s = np.sign(d)
                    s[s == 0] = 1.0
                    qe = qe * s[:, None]
                    
                    # FIX 4: Compute angle error after proper alignment
                    dot_product = np.sum(qt * qe, axis=1)
                    dot_product = np.clip(dot_product, -1.0, 1.0)  # Handle numerical precision
                    ang = 2.0 * np.degrees(np.arccos(np.abs(dot_product)))
                    
                    # short-window score to prefer the correct layout
                    wmask = te_win <= (t0 + min(30.0, (t1 - t0)))
                    score = float(np.median(ang[wmask])) if np.any(wmask) else float(np.median(ang))
                    
                    # FIX 5: Use SciPy for both to guarantee identical results
                    # The amplitude matching is the main issue, which is now fixed
                    eul_truth = R.from_quat(as_xyzw_from_wxyz(qt)).as_euler("zyx", degrees=True)
                    eul_est = R.from_quat(as_xyzw_from_wxyz(qe)).as_euler("zyx", degrees=True)
                    cand.append({
                        "score": score,
                        "qt_wxyz": qt,
                        "qe_wxyz": qe,
                        "ang": ang,
                        "eul_truth": eul_truth,
                        "eul_est": eul_est,
                    })

            if not cand:
                logging.warning("Could not parse quaternions for truth/estimate; skipping attitude plots.")
            else:
                best = min(cand, key=lambda c: c["score"])
                qt = best["qt_wxyz"]
                qe = best["qe_wxyz"]
                ang = best["ang"]
                eul_truth = best["eul_truth"]
                eul_est = best["eul_est"]

                # Plots with TAG in filename
                tag_prefix = os.path.join(args.output, f"{tag}")
                # Quaternion components (w,x,y,z)
                comps = ["w", "x", "y", "z"]
                plt.figure(figsize=(10, 7))
                for i, cch in enumerate(comps):
                    plt.subplot(4, 1, i + 1)
                    plt.plot(te_win, qt[:, i], label="truth")
                    plt.plot(te_win, qe[:, i], label="estimate")
                    plt.ylabel(f"q_{cch}")
                    if i == 0:
                        plt.title("Quaternion Components: Truth vs Estimate")
                        plt.legend(loc="upper right")
                    if i == 3:
                        plt.xlabel("Time [s]")
                plt.tight_layout()
                # Save PNG and MATLAB .fig with Task/Frame in filename
                f_q_base = f"{tag_prefix}_Task7_BodyToNED_attitude_truth_vs_estimate_quaternion"
                fig_q = plt.gcf()
                try:
                    fig_q.savefig(f"{f_q_base}.png", dpi=200, bbox_inches='tight')
                except Exception:
                    pass
                # Save MATLAB .mat bundle for interactive plotting in MATLAB
                try:
                    from scipy.io import savemat  # type: ignore
                    # Align lengths conservatively to common minimum
                    n = min(len(te_win), len(qt), len(qe))
                    t_mat = np.asarray(te_win[:n], float)
                    qt_mat = np.asarray(qt[:n], float)
                    qe_mat = np.asarray(qe[:n], float)
                    savemat(f"{f_q_base}.mat", {
                        't': t_mat,
                        'q_truth': qt_mat,
                        'q_est': qe_mat,
                    })
                except Exception:
                    pass
                try:
                    from utils.matlab_fig_export import save_matlab_fig
                    save_matlab_fig(fig_q, f_q_base)
                except Exception:
                    pass
                plt.close()
                # Euler (Z-Y-X yaw/pitch/roll)
                labels = ["Yaw [deg]", "Pitch [deg]", "Roll [deg]"]
                plt.figure(figsize=(10, 6))
                for i, lab in enumerate(labels):
                    plt.subplot(3, 1, i + 1)
                    plt.plot(te_win, eul_truth[:, i], label="truth", alpha=0.8)
                    plt.plot(te_win, eul_est[:, i], label="estimate", alpha=0.8)
                    plt.ylabel(lab)
                    if i == 0:
                        plt.title("Attitude (Euler Z-Y-X, body→NED) Truth vs Estimate")
                        plt.legend(loc="upper right")
                    if i == 2:
                        plt.xlabel("Time [s]")
                plt.tight_layout()
                f_e_base = f"{tag_prefix}_Task7_BodyToNED_attitude_truth_vs_estimate_euler"
                fig_e = plt.gcf()
                try:
                    fig_e.savefig(f"{f_e_base}.png", dpi=200, bbox_inches='tight')
                except Exception:
                    pass
                # Save MATLAB .mat for Euler overlay (ZYX degrees)
                try:
                    from scipy.io import savemat  # type: ignore
                    n = min(len(te_win), len(eul_truth), len(eul_est))
                    t_mat = np.asarray(te_win[:n], float)
                    et_mat = np.asarray(eul_truth[:n], float)
                    ee_mat = np.asarray(eul_est[:n], float)
                    savemat(f"{f_e_base}.mat", {
                        't': t_mat,
                        'e_truth_zyx_deg': et_mat,
                        'e_est_zyx_deg': ee_mat,
                    })
                except Exception:
                    pass
                try:
                    from utils.matlab_fig_export import save_matlab_fig
                    save_matlab_fig(fig_e, f_e_base)
                except Exception:
                    pass
                plt.close()

                # Divergence detection on overlap only, with CLI threshold/persist
                thr = float(args.div_threshold_deg)
                persist = float(args.div_persist_sec)
                above = ang > thr
                dt = np.diff(te_win, prepend=te_win[0])
                run_t = 0.0
                div_idx = None
                for i in range(len(te_win)):
                    if above[i]:
                        run_t += dt[i]
                        if run_t >= persist:
                            j = i
                            while j > 0 and above[j - 1]:
                                j -= 1
                            div_idx = j
                            break
                    else:
                        run_t = 0.0
                if div_idx is not None:
                    div_time_rel = float(te_win[div_idx] - te_win[0])
                    print(f"Estimated divergence start time (attitude): {div_time_rel:.1f} s")
                    divergence_s = div_time_rel
                else:
                    print("Estimated divergence start time (attitude): nan s")
                    divergence_s = np.nan

                # Save angle error plot
                plt.figure(figsize=(10, 4))
                plt.plot(te_win, ang, label="attitude error [deg]")
                plt.axhline(thr, color="r", linestyle="--", label=f"threshold {thr:.1f} deg")
                if div_idx is not None:
                    plt.axvline(te_win[div_idx], color="g", linestyle=":", label=f"diverges @ {divergence_s:.1f}s")
                plt.xlabel("Time [s]")
                plt.ylabel("Angle error [deg]")
                plt.title("Quaternion Angle Error vs Time")
                plt.legend()
                plt.tight_layout()
                f_a_base = f"{tag_prefix}_Task7_attitude_error_angle_over_time"
                fig_a = plt.gcf()
                try:
                    fig_a.savefig(f"{f_a_base}.png", dpi=200, bbox_inches='tight')
                except Exception:
                    pass
                from utils.matlab_fig_export import save_matlab_fig
                if save_matlab_fig(fig_a, f_a_base) is None:
                    try:
                        from utils_legacy import save_plot_fig
                        save_plot_fig(fig_a, f_a_base + '.fig')
                    except Exception:
                        pass
                plt.close()

                # Save divergence summary CSV
                import csv
                f_csv = f"{tag_prefix}_divergence_summary.csv"
                with open(f_csv, "w", newline="") as fh:
                    w = csv.writer(fh)
                    w.writerow(["threshold_deg", "persist_s", "divergence_s"])
                    w.writerow([thr, persist, divergence_s])

                # Length-dependence scan
                try:
                    scan_vals = [float(x) for x in str(args.length_scan).split(",") if str(x).strip()]
                except Exception:
                    scan_vals = []
                if scan_vals:
                    rows = []
                    for L in scan_vals:
                        # truncate to [t0, min(t0+L, t1)]
                        tmax = te_win[0] + L
                        m = te_win <= tmax
                        if not np.any(m):
                            rows.append([L, np.nan])
                            continue
                        ang_L = ang[m]
                        t_L = te_win[m]
                        above_L = ang_L > thr
                        dt_L = np.diff(t_L, prepend=t_L[0])
                        run = 0.0
                        div_idx_L = None
                        for i in range(len(t_L)):
                            if above_L[i]:
                                run += dt_L[i]
                                if run >= persist:
                                    j = i
                                    while j > 0 and above_L[j - 1]:
                                        j -= 1
                                    div_idx_L = j
                                    break
                            else:
                                run = 0.0
                        div_s_L = (t_L[div_idx_L] - t_L[0]) if div_idx_L is not None else np.nan
                        rows.append([L, float(div_s_L) if np.isfinite(div_s_L) else np.nan])

                    # Write CSV and plot PNG + MATLAB .mat for interactive use
                    import csv
                    f_len = f"{tag_prefix}_Task7_divergence_vs_length.csv"
                    with open(f_len, "w", newline="") as fh:
                        w = csv.writer(fh)
                        w.writerow(["tmax_s", "divergence_s"])
                        for r in rows:
                            w.writerow(r)
                    # Plot
                    xs = [r[0] for r in rows]
                    ys = [r[1] for r in rows]
                    plt.figure(figsize=(6, 4))
                    plt.plot(xs, ys, "o-", label="divergence vs length")
                    plt.xlabel("tmax_s")
                    plt.ylabel("divergence_s (from overlap start)")
                    plt.grid(True, alpha=0.3)
                    plt.tight_layout()
                    f_png_base = f"{tag_prefix}_Task7_divergence_vs_length"
                    try:
                        plt.gcf().savefig(f"{f_png_base}.png", dpi=200, bbox_inches='tight')
                    except Exception:
                        pass
                    try:
                        from utils.matlab_fig_export import save_matlab_fig
                        save_matlab_fig(plt.gcf(), f_png_base)
                    except Exception:
                        # Engine unavailable — skip .fig entirely
                        pass
                    plt.close()
                    # Save MATLAB-friendly .mat with the arrays
                    try:
                        from scipy.io import savemat
                        savemat(f"{tag_prefix}_Task7_divergence_vs_length.mat", {
                            "tmax_s": np.array(xs, float),
                            "divergence_s": np.array(ys, float),
                        })
                    except Exception:
                        pass

    # --- Performance metrics ----------------------------------------------
    final_pos_error = np.linalg.norm(err_pos[-1])
    rmse_pos = np.sqrt(np.mean(np.sum(err_pos**2, axis=1)))
    summary_lines = []
    if dataset_warning:
        summary_lines.append(dataset_warning)
    summary_lines += [
        f"Final position error: {final_pos_error:.2f} m",
        f"RMSE position error: {rmse_pos:.2f} m",
    ]
    final_vel_error = rmse_vel = None
    if err_vel is not None:
        final_vel_error = np.linalg.norm(err_vel[-1])
        rmse_vel = np.sqrt(np.mean(np.sum(err_vel**2, axis=1)))
        summary_lines += [
            f"Final velocity error: {final_vel_error:.2f} m/s",
            f"RMSE velocity error: {rmse_vel:.2f} m/s",
        ]
        print(f"Final fused_vel_ned: {est['vel'][-1]}")
        print(f"Final truth_vel_ned: {truth_vel_ned[-1]}")
        print(f"Final velocity error: {err_vel[-1]}")
    if err_quat is not None:
        summary_lines += [
            f"Final attitude error: {final_att_error:.4f} deg",
            f"RMSE attitude error: {rmse_att:.4f} deg",
        ]

    summary_path = os.path.join(args.output, "validation_summary.txt")

    sigma_pos = sigma_vel = sigma_quat = None
    if est["P"] is not None:
        t_est = np.asarray(est["time"]).squeeze()
        diag = np.diagonal(est["P"], axis1=1, axis2=2)
        # some files store one more covariance entry than timestamps
        n_sigma = min(len(t_est), diag.shape[0])
        t_sigma = t_est[:n_sigma]
        diag = diag[:n_sigma]
        if diag.shape[1] >= 3:
            tmp = 3 * np.sqrt(diag[:, :3])
            sigma_pos = np.vstack(
                [np.interp(t_truth, t_sigma, tmp[:, i]) for i in range(3)]
            ).T
        if diag.shape[1] >= 6:
            tmp = 3 * np.sqrt(diag[:, 3:6])
            sigma_vel = np.vstack(
                [np.interp(t_truth, t_sigma, tmp[:, i]) for i in range(3)]
            ).T
        if diag.shape[1] >= 10:
            tmp = 3 * np.sqrt(diag[:, 6:10])
            sigma_quat = np.vstack(
                [np.interp(t_truth, t_sigma, tmp[:, i]) for i in range(4)]
            ).T

    def plot_err(x, err, sigma, labels, prefix, xlabel):
        for i, lbl in enumerate(labels):
            plt.figure()
            plt.plot(x, err[:, i], label="error")
            if sigma is not None:
                plt.plot(x, sigma[: len(x), i], "r--", label="+3σ")
                plt.plot(x, -sigma[: len(x), i], "r--")
            plt.xlabel(xlabel)
            plt.ylabel(f"{lbl} error")
            plt.legend()
            plt.tight_layout()
            from utils.matlab_fig_export import save_matlab_fig
            from utils.matlab_fig_export import save_matlab_fig
            save_matlab_fig(fig, os.path.join(args.output, f"{prefix}_{lbl}"))
            plt.close()

    plot_err(dist_truth, err_pos, sigma_pos, ["X", "Y", "Z"], "pos_err", "Distance [m]")
    if err_vel is not None:
        plot_err(dist_truth, err_vel, sigma_vel, ["Vx", "Vy", "Vz"], "vel_err", "Distance [m]")
    if err_quat is not None:
        plot_err(dist_truth, err_quat, sigma_quat, ["q0", "q1", "q2", "q3"], "att_err", "Distance [m]")

    m = re.match(
        r"(IMU_\w+)_(GNSS_\w+)_([A-Za-z]+)_kf_output",
        os.path.basename(args.est_file),
    )
    if m:
        dataset_dir = Path(args.truth_file).resolve().parent
        imu_file = dataset_dir / f"{m.group(1)}.dat"
        gnss_file = dataset_dir / f"{m.group(2)}.csv"
        method = m.group(3)
        try:
            frames = assemble_frames(
                est, imu_file, gnss_file, truth_file=args.truth_file
            )
            tag = f"{m.group(1)}_{m.group(2)}_{method}"
            for frame_name, data in frames.items():
                t_i, p_i, v_i, a_i = data["imu"]
                t_g, p_g, v_g, a_g = data["gnss"]
                t_f, p_f, v_f, a_f = data["fused"]
                truth = data.get("truth")
                filename = f"{tag}_task6_overlay_state_{frame_name}.pdf"
                plot_overlay(
                    frame_name,
                    method,
                    t_i,
                    p_i,
                    v_i,
                    a_i,
                    t_g,
                    p_g,
                    v_g,
                    a_g,
                    t_f,
                    p_f,
                    v_f,
                    a_f,
                    args.output,
                    truth,
                    filename=filename,
                )
        except Exception as e:
            print(f"Overlay plot failed: {e}")

    for line in summary_lines:
        print(line)

    # Tabulated summary for easy comparison
    table_rows = [["Position [m]", final_pos_error, rmse_pos]]
    if rmse_vel is not None:
        table_rows.append(["Velocity [m/s]", final_vel_error, rmse_vel])
    # Acceleration is intentionally omitted (no truth acceleration available)
    if err_quat is not None:
        table_rows.append(["Attitude [deg]", final_att_error, rmse_att])
    print(tabulate(table_rows, headers=["Metric", "Final Error", "RMSE"], floatfmt=".3f"))

    try:
        with open(summary_path, "w") as f:
            f.write("\n".join(summary_lines) + "\n")
    except OSError as e:
        print(f"Could not write summary file: {e}")


if __name__ == "__main__":
    main()
