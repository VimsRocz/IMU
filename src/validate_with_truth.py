import argparse
import os
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as R, Slerp
import matplotlib.pyplot as plt

from utils import compute_C_ECEF_to_NED, ecef_to_geodetic
from plot_overlay import plot_overlay
import pandas as pd
import re

__all__ = ["load_estimate", "assemble_frames"]


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

    def pick_key(keys, container, n_cols=None, default=None):
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
                    print(f"Using '{k}' for '{keys[0] if keys else '?'}'")
                    return v
        print(f"Could not find any of {keys}. Available keys: {list(container.keys())}")
        return default

    if path.endswith(".npz"):
        data = np.load(path, allow_pickle=True)
        t = pick_key(["time_residuals", "time"], data)
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
            "P": pick_key(["P", "P_hist"], data) if pos_found else None,
            "ref_lat": data.get("ref_lat") or data.get("lat0"),
            "ref_lon": data.get("ref_lon") or data.get("lon0"),
            "ref_r0": data.get("ref_r0") or data.get("r0"),
        }
    else:
        m = loadmat(path)
        t = pick_key(["time_residuals", "time"], m)
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
            "P": pick_key(["P", "P_hist"], m) if pos_found else None,
            "ref_lat": m.get("ref_lat") or m.get("lat0"),
            "ref_lon": m.get("ref_lon") or m.get("lon0"),
            "ref_r0": m.get("ref_r0") or m.get("r0"),
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

    Parameters
    ----------
    est : dict
        Output from :func:`load_estimate`.
    imu_file, gnss_file : str
        Raw data files used to generate *est*.
    truth_file : str or None, optional
        Path to ``STATE_*.txt`` containing the reference trajectory. When
        provided, the returned frames include an additional ``"truth"``
        entry interpolated to the fused time vector.
    """
    gnss = pd.read_csv(gnss_file)
    t_gnss = gnss["Posix_Time"].to_numpy()
    pos_ecef = gnss[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].to_numpy()
    vel_ecef = gnss[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].to_numpy()
    dt_g = np.diff(t_gnss, prepend=t_gnss[0])
    acc_ecef = np.zeros_like(vel_ecef)
    acc_ecef[1:] = np.diff(vel_ecef, axis=0) / dt_g[1:, None]

    ref_lat = est.get("ref_lat")
    ref_lon = est.get("ref_lon")
    ref_r0 = est.get("ref_r0")
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
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    pos_gnss_ned = np.array([C @ (p - ref_r0) for p in pos_ecef])
    vel_gnss_ned = np.array([C @ v for v in vel_ecef])
    acc_gnss_ned = np.array([C @ a for a in acc_ecef])

    t_est = np.asarray(est["time"]).squeeze()
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
    if truth_file is not None:
        try:
            truth = np.loadtxt(truth_file, comments="#")
            t_truth = truth[:, 1]
            pos_truth_ecef = truth[:, 2:5]
            vel_truth_ecef = truth[:, 5:8]
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
        except Exception as e:
            print(f"Failed to load truth file {truth_file}: {e}")
            truth_file = None

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
        if truth_file is not None:
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
        if truth_file is not None:
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

    if truth_file is not None:
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
    ap.add_argument("--ref-lat", type=float, help="reference latitude in degrees")
    ap.add_argument("--ref-lon", type=float, help="reference longitude in degrees")
    ap.add_argument("--ref-r0", type=float, nargs=3, help="ECEF origin [m]")
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)

    truth = np.loadtxt(args.truth_file)
    t_truth = truth[:, 1]
    est = load_estimate(args.est_file, times=t_truth)

    ref_lat = np.deg2rad(args.ref_lat) if args.ref_lat is not None else None
    ref_lon = np.deg2rad(args.ref_lon) if args.ref_lon is not None else None
    ref_r0 = np.array(args.ref_r0) if args.ref_r0 is not None else None

    if ref_lat is None:
        v = est.get("ref_lat")
        if v is not None:
            ref_lat = float(np.asarray(v).squeeze())
    if ref_lon is None:
        v = est.get("ref_lon")
        if v is not None:
            ref_lon = float(np.asarray(v).squeeze())
    if ref_r0 is None:
        v = est.get("ref_r0")
        if v is not None:
            ref_r0 = np.asarray(v).squeeze()

    if ref_lat is None or ref_lon is None or ref_r0 is None:
        first_ecef = truth[0, 2:5]
        lat_deg, lon_deg, _ = ecef_to_geodetic(*first_ecef)
        ref_lat = np.deg2rad(lat_deg)
        ref_lon = np.deg2rad(lon_deg)
        ref_r0 = first_ecef

    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)

    truth_pos_ned = np.array([C @ (p - ref_r0) for p in truth[:, 2:5]])

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
    summary_lines = [
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
    if err_quat is not None:
        summary_lines += [
            f"Final attitude error: {final_att_error:.4f} deg",
            f"RMSE attitude error: {rmse_att:.4f} deg",
        ]

    for line in summary_lines:
        print(line)

    summary_path = os.path.join(args.output, "validation_summary.txt")
    try:
        with open(summary_path, "w") as f:
            f.write("\n".join(summary_lines) + "\n")
    except OSError as e:
        print(f"Could not write summary file: {e}")

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

    def plot_err(t, err, sigma, labels, prefix):
        for i, lbl in enumerate(labels):
            plt.figure()
            plt.plot(t, err[:, i], label="error")
            if sigma is not None:
                plt.plot(t, sigma[: len(t), i], "r--", label="+3σ")
                plt.plot(t, -sigma[: len(t), i], "r--")
            plt.xlabel("Time [s]")
            plt.ylabel(f"{lbl} error")
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(args.output, f"{prefix}_{lbl}.pdf"))
            plt.close()

    plot_err(t_truth, err_pos, sigma_pos, ["X", "Y", "Z"], "pos_err")
    if err_vel is not None:
        plot_err(t_truth, err_vel, sigma_vel, ["Vx", "Vy", "Vz"], "vel_err")
    if err_quat is not None:
        plot_err(t_truth, err_quat, sigma_quat, ["q0", "q1", "q2", "q3"], "att_err")

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
            for frame_name, data in frames.items():
                t_i, p_i, v_i, a_i = data["imu"]
                t_g, p_g, v_g, a_g = data["gnss"]
                t_f, p_f, v_f, a_f = data["fused"]
                truth = data.get("truth")
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
                )
        except Exception as e:
            print(f"Overlay plot failed: {e}")


if __name__ == "__main__":
    main()
