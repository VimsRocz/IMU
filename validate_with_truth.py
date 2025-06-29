import argparse
import os

import numpy as np
from scipy.io import loadmat
from scipy.spatial.transform import Rotation as R, Slerp


from utils import compute_C_ECEF_to_NED, ecef_to_geodetic
from plot_overlay import plot_overlay
from plots import plot_frame
import pandas as pd
import re


def load_estimate(path):
    """Return trajectory, quaternion and covariance from an NPZ or MAT file."""

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

    return est


def load_state_truth(path: str):
    """Load reference state as ECEF position and velocity."""

    data = np.loadtxt(path, comments="#")
    t_true = np.asarray(data[:, 1], dtype=np.float64)
    pos_ecef_true = np.asarray(data[:, 2:5], dtype=np.float64)
    vel_ecef_true = np.asarray(data[:, 5:8], dtype=np.float64)
    return t_true, pos_ecef_true, vel_ecef_true


def assemble_frames(est, imu_file, gnss_file, ref_lat=None, ref_lon=None, ref_r0=None):
    """Return aligned datasets in NED/ECEF/Body frames."""
    gnss = pd.read_csv(gnss_file)
    t_gnss = gnss["Posix_Time"].to_numpy()
    pos_ecef = gnss[["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]].to_numpy()
    vel_ecef = gnss[["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]].to_numpy()
    dt_g = np.diff(t_gnss, prepend=t_gnss[0])
    acc_ecef = np.zeros_like(vel_ecef)
    acc_ecef[1:] = np.diff(vel_ecef, axis=0) / dt_g[1:, None]

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
        raise ValueError("Reference location missing")
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    pos_gnss_ned = np.array([C @ (p - ref_r0) for p in pos_ecef])
    vel_gnss_ned = np.array([C @ v for v in vel_ecef])
    acc_gnss_ned = np.array([C @ a for a in acc_ecef])

    t_est = np.asarray(est["time"]).squeeze()
    fused_pos = np.asarray(est["pos"])[: len(t_est)]
    fused_vel = np.asarray(est["vel"])[: len(t_est)]
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
    return frames


def prepare_truth_frames(truth, ref_lat, ref_lon, ref_r0, t_ref):
    """Convert truth data to ECEF/NED and interpolate to *t_ref*."""
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    t = truth[:, 1]
    pos_ecef = truth[:, 2:5]
    vel_ecef = truth[:, 5:8]
    acc_ecef = np.zeros_like(vel_ecef)
    if len(t) > 1:
        dt = np.diff(t, prepend=t[0])
        acc_ecef[1:] = np.diff(vel_ecef, axis=0) / dt[1:, None]

    pos_ned = np.array([C @ (p - ref_r0) for p in pos_ecef])
    vel_ned = np.array([C @ v for v in vel_ecef])
    acc_ned = np.array([C @ a for a in acc_ecef])

    def interp(data):
        return np.vstack([np.interp(t_ref, t, data[:, i]) for i in range(3)]).T

    return {
        "ECEF": (t_ref, interp(pos_ecef), interp(vel_ecef), interp(acc_ecef)),
        "NED": (t_ref, interp(pos_ned), interp(vel_ned), interp(acc_ned)),
    }


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
    ap.add_argument(
        "--index-align",
        action="store_true",
        help="Match states by sample index instead of time",
    )
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)

    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        raise ImportError("matplotlib is required for plotting") from e

    est = load_estimate(args.est_file)
    truth = np.loadtxt(args.truth_file)

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
        if truth.size > 0:
            if ref_r0 is None:
                ref_r0 = truth[0, 2:5]
            lat_deg, lon_deg, _ = ecef_to_geodetic(*ref_r0)
            if ref_lat is None:
                ref_lat = np.deg2rad(lat_deg)
            if ref_lon is None:
                ref_lon = np.deg2rad(lon_deg)

    if ref_lat is None or ref_lon is None or ref_r0 is None:
        ap.error(
            "ref_lat, ref_lon and ref_r0 must be provided via command line or contained in the estimate file"
        )

    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)

    t_truth = truth[:, 1]
    truth_pos_ned = np.array([C @ (p - ref_r0) for p in truth[:, 2:5]])

    t_est = np.asarray(est["time"]).squeeze()
    pos_est = np.asarray(est["pos"])
    err_pos = None
    err_vel = None
    err_quat = None

    if args.index_align:
        n = min(len(pos_est), truth_pos_ned.shape[0])
        err_pos = pos_est[:n] - truth_pos_ned[:n]
        if est.get("vel") is not None:
            truth_vel_ned = np.array([C @ v for v in truth[:n, 5:8]])
            vel_est = np.asarray(est["vel"])
            err_vel = vel_est[:n] - truth_vel_ned
    else:
        n_pos = min(len(t_est), len(pos_est))
        t_pos = t_est[:n_pos]
        pos_est = pos_est[:n_pos]
        est_pos_interp = np.vstack(
            [np.interp(t_truth, t_pos, pos_est[:, i]) for i in range(3)]
        ).T
        err_pos = est_pos_interp - truth_pos_ned

        if est.get("vel") is not None:
            truth_vel_ned = np.array([C @ v for v in truth[:, 5:8]])
            vel_est = np.asarray(est["vel"])
            n_vel = min(len(t_est), len(vel_est))
            t_vel = t_est[:n_vel]
            vel_est = vel_est[:n_vel]
            est_vel_interp = np.vstack(
                [np.interp(t_truth, t_vel, vel_est[:, i]) for i in range(3)]
            ).T
            err_vel = est_vel_interp - truth_vel_ned

    if est.get("quat") is not None:
        q_true = truth[:, 8:12]
        quat_est = np.asarray(est["quat"])
        if args.index_align:
            n_q = min(len(quat_est), q_true.shape[0])
            r_true = R.from_quat(q_true[:n_q, [1, 2, 3, 0]])
            r_est = R.from_quat(quat_est[:n_q, [1, 2, 3, 0]])
            r_err = r_est * r_true.inv()
        else:
            n_q = min(len(t_est), len(quat_est))
            t_q = t_est[:n_q]
            r_est = R.from_quat(quat_est[:n_q][:, [1, 2, 3, 0]])
            r_true = R.from_quat(q_true[:, [1, 2, 3, 0]])
            slerp = Slerp(t_q, r_est)
            r_interp = slerp(np.clip(t_truth, t_q[0], t_q[-1]))
            r_err = r_interp * r_true.inv()
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
        diag = np.diagonal(est["P"], axis1=1, axis2=2)
        if args.index_align:
            n_sigma = min(err_pos.shape[0], diag.shape[0])
            diag = diag[:n_sigma]
        else:
            n_sigma = min(len(t_est), diag.shape[0])
            t_sigma = t_est[:n_sigma]
            diag = diag[:n_sigma]
        if diag.shape[1] >= 3:
            tmp = 3 * np.sqrt(diag[:, :3])
            if args.index_align:
                sigma_pos = tmp[: n_sigma, :]
            else:
                sigma_pos = np.vstack(
                    [np.interp(t_truth, t_sigma, tmp[:, i]) for i in range(3)]
                ).T
        if diag.shape[1] >= 6:
            tmp = 3 * np.sqrt(diag[:, 3:6])
            if args.index_align:
                sigma_vel = tmp[: n_sigma, :]
            else:
                sigma_vel = np.vstack(
                    [np.interp(t_truth, t_sigma, tmp[:, i]) for i in range(3)]
                ).T
        if diag.shape[1] >= 10:
            tmp = 3 * np.sqrt(diag[:, 6:10])
            if args.index_align:
                sigma_quat = tmp[: n_sigma, :]
            else:
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

    t_plot = t_truth if not args.index_align else np.arange(err_pos.shape[0])
    plot_err(t_plot, err_pos, sigma_pos, ["X", "Y", "Z"], "pos_err")
    if err_vel is not None:
        plot_err(t_plot, err_vel, sigma_vel, ["Vx", "Vy", "Vz"], "vel_err")
    if err_quat is not None:
        plot_err(t_plot, err_quat, sigma_quat, ["q0", "q1", "q2", "q3"], "att_err")

    m = re.match(
        r"(IMU_\w+)_GNSS_(\w+)_([A-Za-z]+)_kf_output", os.path.basename(args.est_file)
    )
    if m:
        imu_file = f"{m.group(1)}.dat"
        gnss_file = f"GNSS_{m.group(2)}.csv"
        method = m.group(3)
        try:
            frames = assemble_frames(est, imu_file, gnss_file, ref_lat, ref_lon, ref_r0)
            truth_frames = prepare_truth_frames(
                truth, ref_lat, ref_lon, ref_r0, frames["NED"]["fused"][0]
            )
        except Exception as e:
            print(f"Overlay plot failed: {e}")
            return
        for frame_name, data in frames.items():
            t_i, p_i, v_i, a_i = data["imu"]
            t_g, p_g, v_g, a_g = data["gnss"]
            t_f, p_f, v_f, a_f = data["fused"]
            try:
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
                )
            except Exception as e:
                print(f"Overlay plot failed: {e}")
            try:
                plot_frame(
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
                    truth_frames.get(frame_name),
                )
            except Exception as e:
                print(f"Frame plot failed: {e}")

        # --- Additional comparison with reference state -------------------
        try:
            truth = np.loadtxt(args.truth_file)
            t_true = truth[:, 1]
            pos_ecef_true = truth[:, 2:5]
            vel_ecef_true = truth[:, 5:8]
            dt_t = np.diff(t_true, prepend=t_true[0])
            acc_ecef_true = np.zeros_like(vel_ecef_true)
            acc_ecef_true[1:] = np.diff(vel_ecef_true, axis=0) / dt_t[1:, None]

            C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
            pos_ned_true = np.array([C @ (p - ref_r0) for p in pos_ecef_true])
            vel_ned_true = np.array([C @ v for v in vel_ecef_true])
            acc_ned_true = np.array([C @ a for a in acc_ecef_true])

            q_true = truth[:, 8:12]
            rot_true = R.from_quat(q_true[:, [1, 2, 3, 0]])
            pos_body_true = rot_true.apply(pos_ned_true)
            vel_body_true = rot_true.apply(vel_ned_true)
            acc_body_true = rot_true.apply(acc_ned_true)

            t_est = np.asarray(est["time"]).squeeze()
            pos_est = np.asarray(est["pos"])[: len(t_est)]
            vel_est = np.asarray(est["vel"])[: len(t_est)]
            acc_est = np.zeros_like(vel_est)
            pos_est_i = np.vstack([
                np.interp(t_true, t_est, pos_est[:, i]) for i in range(3)
            ]).T
            vel_est_i = np.vstack([
                np.interp(t_true, t_est, vel_est[:, i]) for i in range(3)
            ]).T
            acc_est_i = np.zeros_like(vel_est_i)
            if len(t_true) > 1:
                dt_i = np.diff(t_true, prepend=t_true[0])
                acc_est_i[1:] = np.diff(vel_est_i, axis=0) / dt_i[1:, None]

            C_N2E = C.T
            pos_ecef_est = (C_N2E @ pos_est_i.T).T + ref_r0
            vel_ecef_est = (C_N2E @ vel_est_i.T).T
            acc_ecef_est = (C_N2E @ acc_est_i.T).T

            q_est = est.get("quat")
            if q_est is not None:
                q_est = np.asarray(q_est)[: len(t_est)]
                r_est = R.from_quat(q_est[:, [1, 2, 3, 0]])
                slerp = Slerp(t_est[: len(q_est)], r_est)
                r_interp = slerp(np.clip(t_true, t_est[0], t_est[len(q_est) - 1]))
                pos_body_est = r_interp.apply(pos_est_i)
                vel_body_est = r_interp.apply(vel_est_i)
                acc_body_est = r_interp.apply(acc_est_i)
            else:
                pos_body_est = pos_est_i
                vel_body_est = vel_est_i
                acc_body_est = acc_est_i

            plot_frame(
                "NED",
                t_true,
                pos_ned_true,
                vel_ned_true,
                acc_ned_true,
                t_true,
                pos_est_i,
                vel_est_i,
                acc_est_i,
                args.output,
            )
            plot_frame(
                "ECEF",
                t_true,
                pos_ecef_true,
                vel_ecef_true,
                acc_ecef_true,
                t_true,
                pos_ecef_est,
                vel_ecef_est,
                acc_ecef_est,
                args.output,
            )
            plot_frame(
                "BODY",
                t_true,
                pos_body_true,
                vel_body_true,
                acc_body_true,
                t_true,
                pos_body_est,
                vel_body_est,
                acc_body_est,
                args.output,
            )
        except Exception as e:
            print(f"Frame plot failed: {e}")


if __name__ == "__main__":
    main()
