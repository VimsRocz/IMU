#!/usr/bin/env python3
"""Task 6 – Overlay fused results with the raw state trajectory.

This helper script loads the Kalman filter output from Task 5 together with
the corresponding ground truth file and saves only ``*_overlay_state.pdf``
figures showing the fused estimate versus the raw ``STATE_X`` data. Any
legacy ``*_overlay_truth.pdf`` output is skipped. Plots and logs will be
saved in ``results/``.
"""

import argparse
import re
from pathlib import Path
import time
import glob
import os

from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames
from utils import compute_C_ECEF_to_NED, ecef_to_geodetic
from scipy.spatial.transform import Rotation as R
from tabulate import tabulate
import numpy as np


def main() -> None:
    start_time = time.time()
    parser = argparse.ArgumentParser(
        description=(
            "Plot fused IMU/GNSS output with the reference trajectory."
        )
    )
    parser.add_argument(
        "--est-file",
        required=True,
        help="Path to <IMU>_<GNSS>_<METHOD>_kf_output.mat or .npz",
    )
    parser.add_argument(
        "--truth-file",
        required=True,
        help="STATE_X001.txt or similar ground truth file",
    )
    parser.add_argument(
        "--output",
        default="results",
        help="Directory for the generated PDFs",
    )
    parser.add_argument(
        "--tag",
        help="Dataset tag used as filename prefix. Defaults to the prefix of --est-file",
    )
    parser.add_argument(
        "--show-measurements",
        action="store_true",
        help="Include IMU and GNSS measurements in the overlay plots",
    )
    args = parser.parse_args()

    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Remove any existing Task 6 truth overlay PDFs to avoid confusion
    for f in glob.glob(str(out_dir / "*task6_*_truth.pdf")):
        try:
            os.remove(f)
        except OSError:
            pass

    est_path = Path(args.est_file)
    m = re.match(r"(IMU_\w+)_(GNSS_\w+)_([A-Za-z]+)_kf_output", est_path.stem)
    if not m:
        raise ValueError("Estimator filename must follow <IMU>_<GNSS>_<METHOD>_kf_output.*")
    imu_file = Path(f"{m.group(1)}.dat")
    gnss_file = Path(f"{m.group(2)}.csv")
    method = m.group(3)
    tag = args.tag or f"{m.group(1)}_{m.group(2)}_{method}"

    est = load_estimate(str(est_path))
    frames = assemble_frames(est, imu_file, gnss_file, args.truth_file)

    # Load raw truth data without interpolation
    truth_raw = np.loadtxt(args.truth_file, comments="#")
    t_truth = truth_raw[:, 1]
    pos_truth_ecef = truth_raw[:, 2:5]
    vel_truth_ecef = truth_raw[:, 5:8]
    acc_truth_ecef = np.zeros_like(vel_truth_ecef)
    if len(t_truth) > 1:
        dt = np.diff(t_truth, prepend=t_truth[0])
        acc_truth_ecef[1:] = np.diff(vel_truth_ecef, axis=0) / dt[1:, None]

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
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_truth_ecef[0])
        ref_lat = np.deg2rad(lat_deg)
        ref_lon = np.deg2rad(lon_deg)
        ref_r0 = pos_truth_ecef[0]
    else:
        ref_lat = float(np.asarray(ref_lat).squeeze())
        ref_lon = float(np.asarray(ref_lon).squeeze())
        # Convert to radians when values appear to be in degrees
        if abs(ref_lat) > np.pi or abs(ref_lon) > np.pi:
            ref_lat = np.deg2rad(ref_lat)
            ref_lon = np.deg2rad(ref_lon)
        ref_r0 = np.asarray(ref_r0).squeeze()

    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    pos_truth_ned = np.array([C @ (p - ref_r0) for p in pos_truth_ecef])
    vel_truth_ned = np.array([C @ v for v in vel_truth_ecef])
    acc_truth_ned = np.array([C @ a for a in acc_truth_ecef])
    sign = np.array([1.0, 1.0, -1.0])
    pos_truth_ned *= sign
    vel_truth_ned *= sign
    acc_truth_ned *= sign

    q = est.get("quat")
    if q is not None:
        rot = R.from_quat(np.asarray(q)[: len(t_truth)][:, [1, 2, 3, 0]])
        pos_truth_body = rot.apply(pos_truth_ned)
        vel_truth_body = rot.apply(vel_truth_ned)
        acc_truth_body = rot.apply(acc_truth_ned)
    else:
        pos_truth_body = pos_truth_ned
        vel_truth_body = vel_truth_ned
        acc_truth_body = acc_truth_ned

    truth_frames_raw = {
        "NED": (t_truth, pos_truth_ned, vel_truth_ned, acc_truth_ned),
        "ECEF": (t_truth, pos_truth_ecef, vel_truth_ecef, acc_truth_ecef),
        "Body": (t_truth, pos_truth_body, vel_truth_body, acc_truth_body),
    }

    summary_rows = []

    def centre(arr: np.ndarray) -> np.ndarray:
        """Return *arr* translated so its first sample is zero."""
        return arr - arr[0]

    def ensure_relative_time(t: np.ndarray) -> np.ndarray:
        """Return time vector shifted so t[0] == 0."""
        t = np.asarray(t).squeeze()
        return t - t[0]

    for frame_name, data in frames.items():
        t_i, p_i, v_i, a_i = data["imu"]
        t_g, p_g, v_g, a_g = data["gnss"]
        t_f, p_f, v_f, a_f = data["fused"]
        truth = data.get("truth")

        # ------------------------------------------------------------------
        # Normalise time vectors so that the first sample is at t=0. GNSS data
        # in the ECEF frame typically uses absolute POSIX timestamps, while
        # the filter output and truth data start at zero.  Subtracting the
        # initial timestamp ensures all curves share the same time axis.
        # ------------------------------------------------------------------
        if frame_name == "ECEF":
            t_g = t_g - t_g[0]
            t_i = t_i - t_i[0]
            t_f = t_f - t_f[0]
            if truth is not None:
                t_t, p_t, v_t, a_t = truth
                t_t = ensure_relative_time(t_t)
                truth = (t_t, p_t, v_t, a_t)
        else:
            offset = t_i[0]
            t_g = t_g - offset
            t_i = t_i - offset
            t_f = t_f - offset
            if truth is not None:
                t_t, p_t, v_t, a_t = truth
                t_t = ensure_relative_time(t_t)
                truth = (t_t, p_t, v_t, a_t)
        if frame_name == "NED":
            p_i = centre(p_i * sign)
            v_i = v_i * sign
            a_i = a_i * sign
            p_g = centre(p_g * sign)
            v_g = v_g * sign
            a_g = a_g * sign
            p_f = centre(p_f * sign)
            v_f = v_f * sign
            a_f = a_f * sign
            if truth is not None:
                t_t, p_t, v_t, a_t = truth
                p_t = centre(p_t * sign)
                v_t = v_t * sign
                a_t = a_t * sign
                truth = (t_t, p_t, v_t, a_t)
        # Skip generation of legacy *overlay_truth.pdf figures

        if truth is not None:
            t_t, p_t, v_t, a_t = truth
            err_p = p_f - p_t
            err_v = v_f - v_t
            err_a = a_f - a_t
            rmse_p = float(np.sqrt(np.mean(np.sum(err_p ** 2, axis=1))))
            rmse_v = float(np.sqrt(np.mean(np.sum(err_v ** 2, axis=1))))
            rmse_a = float(np.sqrt(np.mean(np.sum(err_a ** 2, axis=1))))
            final_p = float(np.linalg.norm(err_p[-1]))
            final_v = float(np.linalg.norm(err_v[-1]))
            final_a = float(np.linalg.norm(err_a[-1]))
            summary_rows.append(
                [frame_name, rmse_p, final_p, rmse_v, final_v, rmse_a, final_a]
            )

        # Additional plot using raw STATE data without interpolation
        raw = truth_frames_raw.get(frame_name)
        if raw is not None:
            t_t, p_t, v_t, a_t = raw
            t_t = ensure_relative_time(t_t)
            if frame_name == "NED":
                p_t = centre(p_t)
        name_state = f"{tag}_task6_{frame_name}_overlay_state.pdf"
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
            out_dir,
            t_truth=t_t,
            pos_truth=p_t,
            vel_truth=v_t,
            acc_truth=a_t,
            filename=name_state,
            include_measurements=args.show_measurements,
        )

    if summary_rows:
        headers = [
            "Frame",
            "RMSEpos",
            "FinalPos",
            "RMSEvel",
            "FinalVel",
            "RMSEacc",
            "FinalAcc",
        ]
        print(tabulate(summary_rows, headers=headers, floatfmt=".3f"))
    runtime = time.time() - start_time
    print(f"Task 6 runtime: {runtime:.2f} s")


if __name__ == "__main__":
    main()
