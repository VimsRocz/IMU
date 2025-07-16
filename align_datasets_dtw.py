import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from fastdtw import fastdtw
from scipy.spatial.distance import euclidean
from pyproj import Transformer

from src.utils import compute_C_ECEF_to_NED


# ---------------------------------------------------------------------------
# Coordinate conversion helpers
# ---------------------------------------------------------------------------
_transformer_geodetic_to_ecef = Transformer.from_crs("epsg:4979", "epsg:4978", always_xy=True)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> np.ndarray:
    """Return ECEF coordinates for geodetic input using WGS84."""
    x, y, z = _transformer_geodetic_to_ecef.transform(lon_deg, lat_deg, alt_m)
    return np.array([x, y, z])


def ned_to_ecef(pos_ned: np.ndarray, ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float = 0.0) -> np.ndarray:
    """Convert NED positions to ECEF given a reference origin."""
    ref_ecef = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    C = compute_C_ECEF_to_NED(np.radians(ref_lat_deg), np.radians(ref_lon_deg))
    return (C.T @ pos_ned.T).T + ref_ecef


def enu_to_ecef(pos_enu: np.ndarray, ref_lat_deg: float, ref_lon_deg: float, ref_alt_m: float = 0.0) -> np.ndarray:
    """Convert ENU positions to ECEF using NED as intermediate."""
    ned = np.column_stack([pos_enu[:, 1], pos_enu[:, 0], -pos_enu[:, 2]])
    return ned_to_ecef(ned, ref_lat_deg, ref_lon_deg, ref_alt_m)


# ---------------------------------------------------------------------------
# Data loading helpers
# ---------------------------------------------------------------------------

_possible_est_keys = [
    "pos_ecef_m",
    "pos_ecef",
    "pos_ned_m",
    "pos_ned",
    "pos_enu_m",
    "pos_enu",
    "fused_pos",
    "position",
    "pos",
]

_possible_time_keys = ["time", "time_s", "time_residuals"]


def load_estimator_positions(path: str):
    data = np.load(path, allow_pickle=True)
    est_pos = None
    est_frame = None
    for key in _possible_est_keys:
        if key in data:
            est_pos = np.asarray(data[key])
            print(f"Using '{key}' for estimator position")
            if "ecef" in key:
                est_frame = "ecef"
            elif "ned" in key:
                est_frame = "ned"
            elif "enu" in key:
                est_frame = "enu"
            else:
                est_frame = None
            break
    if est_pos is None:
        raise KeyError(f"None of {_possible_est_keys} found in {path}")

    t = None
    for key in _possible_time_keys:
        if key in data:
            t = np.asarray(data[key]).squeeze()
            break
    return est_pos, est_frame, t


def load_truth_positions(path: str):
    truth = np.loadtxt(path, comments="#")
    time = truth[:, 1]
    pos = truth[:, 2:5]
    return pos, time


# ---------------------------------------------------------------------------
# Main alignment logic
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Align estimator with truth using DTW in ECEF frame")
    parser.add_argument("--est-file", required=True, help="Estimator NPZ file")
    parser.add_argument("--truth-file", required=True, help="Truth trajectory file")
    parser.add_argument("--output-dir", default="results", help="Directory for outputs")
    parser.add_argument("--est-frame", choices=["ecef", "ned", "enu"], help="Frame of estimator positions if auto-detect fails")
    parser.add_argument("--truth-frame", choices=["ecef", "ned", "enu"], help="Frame of truth positions; default 'ecef'")
    parser.add_argument("--ref-lat", type=float, help="Reference latitude (deg) for frame conversion")
    parser.add_argument("--ref-lon", type=float, help="Reference longitude (deg) for frame conversion")
    parser.add_argument("--ref-alt", type=float, default=0.0, help="Reference altitude (m)")
    parser.add_argument("--save-aligned", action="store_true", help="Save aligned ECEF positions to NPZ")

    args = parser.parse_args()
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    est_pos, est_frame_auto, est_time = load_estimator_positions(args.est_file)
    est_frame = args.est_frame or est_frame_auto or "ned"
    truth_pos, truth_time = load_truth_positions(args.truth_file)
    truth_frame = args.truth_frame or "ecef"

    ref_lat = args.ref_lat
    ref_lon = args.ref_lon
    ref_alt = args.ref_alt

    if (est_frame != "ecef" or truth_frame != "ecef") and (ref_lat is None or ref_lon is None):
        parser.error("--ref-lat and --ref-lon are required for NED/ENU conversion")

    if est_frame == "ned":
        est_pos_ecef = ned_to_ecef(est_pos, ref_lat, ref_lon, ref_alt)
    elif est_frame == "enu":
        est_pos_ecef = enu_to_ecef(est_pos, ref_lat, ref_lon, ref_alt)
    else:
        est_pos_ecef = est_pos

    if truth_frame == "ned":
        truth_pos_ecef = ned_to_ecef(truth_pos, ref_lat, ref_lon, ref_alt)
    elif truth_frame == "enu":
        truth_pos_ecef = enu_to_ecef(truth_pos, ref_lat, ref_lon, ref_alt)
    else:
        truth_pos_ecef = truth_pos

    # Normalise for DTW
    est_norm = (est_pos_ecef - est_pos_ecef.min(axis=0)) / (est_pos_ecef.max(axis=0) - est_pos_ecef.min(axis=0))
    truth_norm = (truth_pos_ecef - truth_pos_ecef.min(axis=0)) / (truth_pos_ecef.max(axis=0) - truth_pos_ecef.min(axis=0))

    _, path = fastdtw(est_norm, truth_norm, dist=euclidean)
    est_idx, truth_idx = zip(*path)

    aligned_est = est_pos_ecef[list(est_idx)]
    aligned_truth = truth_pos_ecef[list(truth_idx)]

    errors = np.linalg.norm(aligned_est - aligned_truth, axis=1)

    print(f"Aligned time range: 0 to {len(errors) - 1} samples")
    print(f"Position errors: mean={errors.mean():.4f}, std={errors.std():.4f}, final={errors[-1]:.4f}")

    np.savetxt(out_dir / "aligned_position_errors.txt", errors)

    if args.save_aligned:
        np.savez(out_dir / "aligned_positions.npz", est=aligned_est, truth=aligned_truth)

    plt.figure()
    plt.plot(est_pos_ecef[:, 0], est_pos_ecef[:, 1], label="Estimator")
    plt.plot(truth_pos_ecef[:, 0], truth_pos_ecef[:, 1], label="Truth")
    plt.legend()
    plt.title("XY Trajectory in ECEF before alignment")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.tight_layout()
    plt.savefig(out_dir / "trajectories_ecef.png")
    plt.close()

    plt.figure()
    plt.plot(aligned_est[:, 0], aligned_est[:, 1], label="Estimator")
    plt.plot(aligned_truth[:, 0], aligned_truth[:, 1], label="Truth")
    plt.legend()
    plt.title("XY Trajectory in ECEF after DTW alignment")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.tight_layout()
    plt.savefig(out_dir / "trajectories_aligned_ecef.png")
    plt.close()


if __name__ == "__main__":
    main()
