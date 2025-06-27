import argparse
import subprocess
import shutil
from pathlib import Path
import numpy as np
import pandas as pd
import scipy.io


def run_python_pipeline(imu_file: str, gnss_file: str, method: str) -> Path:
    """Run the Python fusion pipeline and return the .mat result path."""
    cmd = [
        "python",
        "GNSS_IMU_Fusion.py",
        "--imu-file", imu_file,
        "--gnss-file", gnss_file,
        "--method", method,
        "--no-plots",
    ]
    subprocess.run(cmd, check=True)
    imu_stem = Path(imu_file).stem
    gnss_stem = Path(gnss_file).stem
    return Path("results") / f"{imu_stem}_{gnss_stem}_{method}_kf_output.mat"


def run_matlab_pipeline(imu_file: str, gnss_file: str, method: str) -> Path:
    """Run the MATLAB pipeline via matlab/octave and return the result file."""
    matlab = shutil.which("matlab") or shutil.which("octave")
    if not matlab:
        raise RuntimeError("MATLAB/Octave not found")
    cmd = (
        f"imu_path=get_data_file('{imu_file}');"
        f"gnss_path=get_data_file('{gnss_file}');"
        f"main(imu_path, gnss_path, '{method}');"
    )
    if "octave" in Path(matlab).name:
        subprocess.run([matlab, "--eval", cmd], check=True)
    else:
        subprocess.run([matlab, "-batch", cmd], check=True)
    imu_stem = Path(imu_file).stem
    gnss_stem = Path(gnss_file).stem
    return Path("results") / f"{imu_stem}_{gnss_stem}_{method}_task5_results.mat"


def load_python_metrics(mat_path: Path) -> tuple[float, float]:
    data = scipy.io.loadmat(mat_path)
    rmse = float(np.squeeze(data["rmse_pos"]))
    final = float(np.squeeze(data["final_pos"]))
    return rmse, final


def load_matlab_metrics(mat_path: Path, imu_file: str, gnss_file: str) -> tuple[float, float]:
    data = scipy.io.loadmat(mat_path)
    x_log = data["x_log"]
    gnss_pos = data["gnss_pos_ned"]

    gnss_df = pd.read_csv(gnss_file)
    gnss_time = gnss_df["Posix_Time"].to_numpy()
    dt_imu = np.loadtxt(imu_file, usecols=1)
    dt = dt_imu[1] - dt_imu[0]
    imu_time = np.arange(x_log.shape[1]) * dt + gnss_time[0]

    pos_interp = np.vstack([
        np.interp(gnss_time, imu_time, x_log[i])
        for i in range(3)
    ]).T
    res = pos_interp - gnss_pos
    rmse = np.sqrt(np.mean(np.sum(res ** 2, axis=1)))
    final = np.linalg.norm(x_log[0:3, -1] - gnss_pos[-1])
    return float(rmse), float(final)


def main() -> None:
    ap = argparse.ArgumentParser(description="Compare Python and MATLAB results")
    ap.add_argument("--imu", default="IMU_X001.dat")
    ap.add_argument("--gnss", default="GNSS_X001.csv")
    ap.add_argument("--method", default="TRIAD", choices=["TRIAD","Davenport","SVD"])
    ap.add_argument("--csv", action="store_true", help="Write results to results/compare_summary.csv")
    args = ap.parse_args()

    py_mat = run_python_pipeline(args.imu, args.gnss, args.method)
    m_mat  = run_matlab_pipeline(args.imu, args.gnss, args.method)

    py_rmse, py_final = load_python_metrics(py_mat)
    m_rmse, m_final = load_matlab_metrics(m_mat, args.imu, args.gnss)

    print(f"Python RMSEpos:  {py_rmse:.3f} m")
    print(f"MATLAB RMSEpos:  {m_rmse:.3f} m")
    print(f"Difference:      {abs(py_rmse - m_rmse):.3f} m")
    print()
    print(f"Python Final Position Error: {py_final:.3f} m")
    print(f"MATLAB Final Position Error: {m_final:.3f} m")
    print(f"Difference:                  {abs(py_final - m_final):.3f} m")

    if args.csv:
        out = Path("results") / "compare_summary.csv"
        out.parent.mkdir(exist_ok=True)
        header = not out.exists()
        with open(out, "a") as fh:
            if header:
                fh.write("imu,gnss,method,rmse_python,rmse_matlab,final_python,final_matlab\n")
            fh.write(f"{args.imu},{args.gnss},{args.method},{py_rmse},{m_rmse},{py_final},{m_final}\n")
        print(f"Summary appended to {out}")


if __name__ == "__main__":
    main()
