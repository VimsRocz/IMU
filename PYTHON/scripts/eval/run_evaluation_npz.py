"""Evaluate fused estimator against truth from an ``*_kf_output.npz`` file."""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
from scipy.signal import correlate


def _trim_to_overlap(t_truth: np.ndarray, x_truth: np.ndarray,
                     t_est: np.ndarray, x_est: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Trim ``truth`` and ``estimate`` arrays to their common time span."""
    start = max(t_truth[0], t_est[0])
    end = min(t_truth[-1], t_est[-1])
    truth_mask = (t_truth >= start) & (t_truth <= end)
    est_mask = (t_est >= start) & (t_est <= end)
    t_common = t_est[est_mask]
    x_est_trim = x_est[est_mask]
    truth_trim = np.vstack([
        np.interp(t_common, t_truth[truth_mask], x_truth[truth_mask][:, i])
        for i in range(x_truth.shape[1])
    ]).T
    return t_common, truth_trim, x_est_trim


def estimate_time_offset(sig_a: np.ndarray, sig_b: np.ndarray, fs: float) -> float:
    """Coarse-to-fine grid search for time offset between signals."""
    t = np.arange(len(sig_a)) / fs
    def _mse(dt: float) -> float:
        interp = np.interp(t, t + dt, sig_b, left=np.nan, right=np.nan)
        mask = ~np.isnan(interp)
        if not np.any(mask):
            return np.inf
        return np.mean((sig_a[mask] - interp[mask]) ** 2)

    coarse = np.arange(-5.0, 5.0, 0.1)
    errs = [ _mse(dt) for dt in coarse ]
    best = coarse[int(np.argmin(errs))]
    fine = np.arange(best - 0.1, best + 0.1, 0.001)
    errs = [ _mse(dt) for dt in fine ]
    return float(fine[int(np.argmin(errs))])


def run_evaluation_npz(npz_file: str) -> dict[str, np.ndarray | float]:
    """Return residual metrics for ``npz_file``."""
    data = np.load(npz_file)
    t_est = data["time"]
    vel_est = data.get("vel_ned")
    if vel_est is None:
        vel_est = data.get("fused_vel")
    t_truth = data.get("truth_time")
    vel_truth = data.get("truth_vel_ned")
    if t_truth is None or vel_truth is None or len(vel_truth) == 0:
        raise KeyError("truth_time/truth_vel_ned missing from npz")
    t_shift = float(data.get("t_shift", 0.0))
    t_truth = t_truth - t_shift
    t_common, truth_vel, est_vel = _trim_to_overlap(t_truth, vel_truth, t_est, vel_est)
    resid = truth_vel - est_vel
    rmse = np.sqrt(np.mean(resid**2, axis=0))
    final = resid[-1]
    max_resid_vel = float(np.max(np.linalg.norm(resid, axis=1)))
    print("Final velocity error [m/s]:", final)
    print("Velocity RMSE [m/s]:", rmse)
    print(f"max_resid_vel = {max_resid_vel:.3f} m/s")
    return {"rmse_vel": rmse, "final_vel": final, "max_resid_vel": max_resid_vel}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--npz", required=True, help="Path to *_kf_output.npz file")
    args = ap.parse_args()
    run_evaluation_npz(args.npz)


if __name__ == "__main__":
    main()
