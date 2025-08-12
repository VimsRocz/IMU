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

from utils import compute_C_ECEF_to_NED, ecef_to_geodetic, zero_base_time
from utils import frames as _frames
R_ecef_to_ned = _frames.R_ecef_to_ned
ecef_vec_to_ned = _frames.ecef_vec_to_ned
from plot_overlay import plot_overlay
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
        attitude and acceleration.
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
        ref_ecef = truth[0, 2:5]
        lat_deg, lon_deg, _ = ecef_to_geodetic(*ref_ecef)
        C = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
        C_NED_to_ECEF = C.T
        truth_pos_ned = np.array([C @ (p - ref_ecef) for p in truth[:, 2:5]])
        # Smooth truth position and differentiate for a cleaner velocity
        from scipy.signal import savgol_filter

        window_length = 11
        polyorder = 2
        pos_sm = savgol_filter(truth_pos_ned, window_length, polyorder, axis=0)
        dt_truth = np.diff(truth_time)
        truth_vel_ned = np.zeros_like(pos_sm)
        truth_vel_ned[1:-1] = (
            pos_sm[2:] - pos_sm[:-2]
        ) / (dt_truth[1:, None] + dt_truth[:-1, None])
        truth_vel_ned[0] = (pos_sm[1] - pos_sm[0]) / dt_truth[0]
        truth_vel_ned[-1] = (pos_sm[-1] - pos_sm[-2]) / dt_truth[-1]

        pos_err = pos_interp - pos_sm
        vel_err = vel_interp - truth_vel_ned

    truth_quat = truth[:, 8:12]
    truth_eul = R.from_quat(truth_quat[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
    eul_err = eul_interp - truth_eul

    rmse_pos = np.sqrt(np.mean(np.linalg.norm(pos_err, axis=1) ** 2))
    final_pos = np.linalg.norm(pos_err[-1, :])
    rmse_vel = np.sqrt(np.mean(np.linalg.norm(vel_err, axis=1) ** 2))
    final_vel = np.linalg.norm(vel_err[-1, :])

    # --- acceleration error -------------------------------------------------
    dt = np.diff(truth_time, prepend=truth_time[0])
    acc_est = np.zeros_like(vel_interp)
    acc_est[1:] = np.diff(vel_interp, axis=0) / dt[1:, None]
    if convert_est_to_ecef:
        vel_truth = truth[:, 5:8]
    else:
        vel_truth = truth_vel_ned
    acc_truth = np.zeros_like(vel_truth)
    acc_truth[1:] = np.diff(vel_truth, axis=0) / dt[1:, None]
    acc_err = acc_est - acc_truth
    rmse_acc = np.sqrt(np.mean(np.linalg.norm(acc_err, axis=1) ** 2))
    final_acc = np.linalg.norm(acc_err[-1, :])

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
    vel_truth = truth[:, 5:8]

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
            plt.savefig(Path("results") / f"debug_{lbl}.pdf")
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
            vel_truth_ecef = truth[:, 5:8]
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

    if est.get("quat") is not None:
        q_true = truth[:, 8:12]
        r_true = R.from_quat(q_true[:, [1, 2, 3, 0]])
        r_est = R.from_quat(np.asarray(est["quat"])[:, [1, 2, 3, 0]])
        r_err = r_est * r_true.inv()
        err_quat = r_err.as_quat()[:, [3, 0, 1, 2]]
        # magnitude of the quaternion error in degrees
        err_angles = 2 * np.arccos(np.clip(np.abs(err_quat[:, 0]), -1.0, 1.0))
        err_deg = np.degrees(err_angles)
        final_att_error = err_deg[-1]
        rmse_att = np.sqrt(np.mean(err_deg**2))

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
        dt = np.diff(t_truth, prepend=t_truth[0])
        acc_est = np.zeros_like(err_vel)
        acc_est[1:] = np.diff(np.asarray(est["vel"]), axis=0) / dt[1:, None]
        acc_truth = np.zeros_like(err_vel)
        acc_truth[1:] = np.diff(truth_vel_ned, axis=0) / dt[1:, None]
        err_acc = acc_est - acc_truth
        final_acc_error = np.linalg.norm(err_acc[-1])
        rmse_acc = np.sqrt(np.mean(np.sum(err_acc**2, axis=1)))
        summary_lines += [
            f"Final acceleration error: {final_acc_error:.2f} m/s^2",
            f"RMSE acceleration error: {rmse_acc:.2f} m/s^2",
        ]
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
            plt.savefig(os.path.join(args.output, f"{prefix}_{lbl}.pdf"))
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
    if 'rmse_acc' in locals():
        table_rows.append(["Acceleration [m/s^2]", final_acc_error, rmse_acc])
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
