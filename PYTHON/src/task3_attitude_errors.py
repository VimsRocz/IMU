from __future__ import annotations

import logging
import math
import re
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import pandas as pd
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation, Slerp

from utils.quaternion import assert_quaternion_convention
from tabulate import tabulate

# Required project import style
from utils.run_id import run_id  # noqa: F401  # imported for style compliance

logger = logging.getLogger(__name__)

RESULTS_ROOT = Path(__file__).resolve().parents[1] / "results"


def _load_quat_csv(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Load quaternion CSV with columns time,qx,qy,qz,qw (case-insensitive)."""
    df = pd.read_csv(path)
    cols = {c.lower(): c for c in df.columns}
    required = ["time", "qx", "qy", "qz", "qw"]
    if not all(c in cols for c in required):
        missing = [c for c in required if c not in cols]
        raise ValueError(f"{path} missing columns: {missing}")
    df = df.rename(columns={cols[c]: c for c in required})
    t = df["time"].to_numpy(dtype=float)
    q = df[["qx", "qy", "qz", "qw"]].to_numpy(dtype=float)
    nrm = np.linalg.norm(q, axis=1, keepdims=True)
    nrm[nrm == 0] = 1.0
    q = q / nrm
    assert_quaternion_convention(q)
    return t, q


def load_state_quats(run_id: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load state/reference quaternions for ``run_id``."""
    candidates = [
        RESULTS_ROOT / run_id / "state" / "quaternions.csv",
        RESULTS_ROOT / run_id / "task3" / "state_quaternions.csv",
        RESULTS_ROOT / run_id / "state_quaternions.csv",
    ]
    for path in candidates:
        if path.exists():
            return _load_quat_csv(path)
    raise FileNotFoundError(f"No state quaternions found for {run_id}")


def load_method_quats(run_id: str, method: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load estimated quaternions for ``method`` in ``run_id``."""
    method = method.lower()
    candidates = [
        RESULTS_ROOT / run_id / "task3" / f"{method}_quaternions.csv",
        RESULTS_ROOT / run_id / f"{method}_quaternions.csv",
    ]
    for path in candidates:
        if path.exists():
            return _load_quat_csv(path)
    raise FileNotFoundError(f"No {method} quaternions for {run_id}")


def _continuous(q: np.ndarray) -> np.ndarray:
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] *= -1
    return q


def interp_quats(t_src: np.ndarray, q_src: np.ndarray, t_dst: np.ndarray) -> np.ndarray:
    """Interpolate quaternions from ``t_src`` to ``t_dst`` using SLERP."""
    t_src = np.asarray(t_src, dtype=float)
    q_src = np.asarray(q_src, dtype=float)
    t_dst = np.asarray(t_dst, dtype=float)
    assert_quaternion_convention(q_src)
    if t_src.size < 2:
        raise ValueError("Need at least two quaternions for interpolation")
    order = np.argsort(t_src)
    t_src = t_src[order]
    q_src = _continuous(q_src[order])
    try:
        rot = Rotation.from_quat(q_src)
        slerp = Slerp(t_src, rot)
        out = slerp(t_dst).as_quat()
    except Exception:  # pragma: no cover - fallback
        arr = np.vstack([np.interp(t_dst, t_src, q_src[:, i]) for i in range(4)]).T
        n = np.linalg.norm(arr, axis=1, keepdims=True)
        n[n == 0] = 1.0
        out = arr / n
    assert_quaternion_convention(out)
    return out


def quat_delta(q_ref: np.ndarray, q_est: np.ndarray) -> np.ndarray:
    """Return quaternion difference ``q_ref * conj(q_est)``."""
    assert_quaternion_convention(q_ref)
    assert_quaternion_convention(q_est)
    q_est_conj = q_est.copy()
    q_est_conj[:, :3] *= -1
    x1, y1, z1, w1 = q_ref.T
    x2, y2, z2, w2 = q_est_conj.T
    q_err = np.column_stack(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]
    )
    n = np.linalg.norm(q_err, axis=1, keepdims=True)
    n[n == 0] = 1.0
    q_err = q_err / n
    q_err[q_err[:, 3] < 0] *= -1
    assert_quaternion_convention(q_err)
    return q_err


def quat_angle_deg(q: np.ndarray) -> np.ndarray:
    """Return rotation angle of quaternion(s) in degrees."""
    assert_quaternion_convention(q)
    return 2 * np.degrees(np.arccos(np.clip(q[:, 3], -1.0, 1.0)))


def summarize_errors(angle_deg: np.ndarray) -> Dict[str, float]:
    angle_deg = np.asarray(angle_deg, dtype=float)
    return {
        "mean": float(np.mean(angle_deg)),
        "median": float(np.median(angle_deg)),
        "rms": float(math.sqrt(np.mean(angle_deg**2))),
        "p95": float(np.percentile(angle_deg, 95)),
    }


def plot_task3_errors(
    methods: List[str],
    grav_err_deg: List[float] | np.ndarray,
    earth_err_deg: List[float] | np.ndarray,
    out_png: str | Path,
) -> None:
    """Plot Task 3 gravity/earth-rate errors and save figure.

    Parameters
    ----------
    methods : list of str
        Names of the attitude-determination methods in display order.
    grav_err_deg : array-like
        Gravity vector errors in degrees corresponding to ``methods``.
    earth_err_deg : array-like
        Earth-rate vector errors in degrees corresponding to ``methods``.
    out_png : path-like
        Output path for the saved ``.png`` file.
    """

    methods = list(methods)
    ge = np.asarray(grav_err_deg, float)
    ee = np.asarray(earth_err_deg, float)

    fig, axs = plt.subplots(1, 2, figsize=(12, 4), constrained_layout=True)
    fig.suptitle("Task 3: Attitude Error Comparison")

    # Gravity error -------------------------------------------------------
    axs[0].bar(methods, ge)
    axs[0].set_title("Gravity Error")
    axs[0].set_ylabel("Error (degrees)")
    gmax = np.nanmax(ge) if ge.size else 1e-6
    gmax = max(gmax, 1e-6)
    axs[0].set_ylim(0, gmax * 1.1)
    for x, y in zip(methods, ge):
        axs[0].text(
            x,
            y + gmax * 0.05,
            f"{y:.6f}",
            ha="center",
            va="bottom",
            fontsize=8,
            rotation=90,
        )

    # Earth-rate error ----------------------------------------------------
    axs[1].bar(methods, ee)
    axs[1].set_title("Earth Rate Error")
    axs[1].set_ylabel("Error (degrees)")
    emax = np.nanmax(ee) if ee.size else 1e-6
    emax = max(emax, 1e-6)
    axs[1].set_ylim(0, emax * 1.1)
    for x, y in zip(methods, ee):
        axs[1].text(
            x,
            y + emax * 0.05,
            f"{y:.6f}",
            ha="center",
            va="bottom",
            fontsize=8,
            rotation=90,
        )

    axs[0].grid(alpha=0.3)
    axs[1].grid(alpha=0.3)

    out_png = Path(out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(out_png.with_suffix('')))
    plt.close(fig)
    print(f"[Task3] Saved error comparison -> {out_png}")


def run_task3(run_id: str, runs: List[str] | None = None) -> None:
    """Generate quaternion overlay and error plots for given runs."""
    if runs is None:
        runs = [run_id]

    for run in runs:
        stats_rows: List[Dict[str, float]] = []
        est_means: Dict[str, np.ndarray] = {}
        ref_mean: np.ndarray | None = None
        try:
            t_ref, q_ref = load_state_quats(run)
            ref_mean = q_ref.mean(axis=0)
        except FileNotFoundError:
            print(f"[Task3] State quaternions not found for {run}. Skipping error plot; showing only estimated components.")
            t_ref = q_ref = None

        for method in ["triad", "davenport", "svd"]:
            try:
                t_est, q_est = load_method_quats(run, method)
            except FileNotFoundError:
                print(f"[Task3] {method} quaternions not found for {run}. Skipping {method}.")
                continue

            if t_ref is not None and q_ref is not None:
                start = max(t_ref[0], t_est[0])
                end = min(t_ref[-1], t_est[-1])
                m_ref = (t_ref >= start) & (t_ref <= end)
                m_est = (t_est >= start) & (t_est <= end)
                t_ref_clip = t_ref[m_ref]
                q_ref_clip = q_ref[m_ref]
                t_est_clip = t_est[m_est]
                q_est_clip = q_est[m_est]
                if t_ref_clip.size == 0 or t_est_clip.size == 0:
                    raise ValueError(f"No overlap between reference and {method} for run {run}")
                q_est_i = interp_quats(t_est_clip, q_est_clip, t_ref_clip)
                q_ref_a = q_ref_clip
                q_est_a = q_est_i
                dots = np.sum(q_ref_a * q_est_a, axis=1)
                q_est_a[dots < 0] *= -1
                mask = (~np.isnan(q_ref_a).any(axis=1)) & (~np.isnan(q_est_a).any(axis=1))
                q_ref_a = q_ref_a[mask]
                q_est_a = q_est_a[mask]
                if q_ref_a.size == 0:
                    raise ValueError(f"Empty aligned arrays for {run} {method}")
                q_err = quat_delta(q_ref_a, q_est_a)
                ang = quat_angle_deg(q_err)
                stats = summarize_errors(ang)
                stats_rows.append(
                    {
                        "run": run,
                        "method": method,
                        "n_samples": int(len(ang)),
                        "rms_deg": stats["rms"],
                        "mean_deg": stats["mean"],
                        "median_deg": stats["median"],
                        "p95_deg": stats["p95"],
                    }
                )
                est_mean = q_est_a.mean(axis=0)
            else:
                est_mean = q_est.mean(axis=0)
            est_means[method] = est_mean

        outdir = RESULTS_ROOT / run / "task3"
        outdir.mkdir(parents=True, exist_ok=True)

        # Quaternion overlay
        comp_labels = ["qx", "qy", "qz", "qw"]
        x = np.arange(len(comp_labels))
        series = []
        labels = []
        for method in ["triad", "davenport", "svd"]:
            if method in est_means:
                series.append(est_means[method])
                labels.append(f"{method.capitalize()} (est)")
        if ref_mean is not None:
            series.append(ref_mean)
            labels.append("State (ref)")
        fig, ax = plt.subplots(figsize=(16, 8), dpi=100)
        if series:
            n_s = len(series)
            width = 0.8 / n_s
            for i, (lab, data) in enumerate(zip(labels, series)):
                ax.bar(x + (i - n_s / 2) * width + width / 2, data, width, label=lab)
            ax.set_xticks(x)
            ax.set_xticklabels(comp_labels)
            ax.set_ylabel("Quaternion Component Value")
            ax.set_title(f"Task 3: Quaternion Components by Method (RUN={run})")
            ax.legend()
        fig.tight_layout()
        from utils.matlab_fig_export import save_matlab_fig
        save_matlab_fig(fig, str((outdir / f"{run}_task3_quaternions_comparison").with_suffix('')))
        plt.close(fig)

        if not stats_rows:
            continue

        df = pd.DataFrame(stats_rows)
        print(
            tabulate(
                df[["run", "method", "n_samples", "rms_deg", "mean_deg", "median_deg", "p95_deg"]],
                headers="keys",
                tablefmt="github",
                floatfmt=".3f",
            )
        )
        df.to_csv(outdir / f"{run}_task3_error_stats.csv", index=False)

        # Error comparison plot
        fig, ax = plt.subplots(figsize=(14, 5), dpi=100)
        x = np.arange(len(df))
        ax.bar(x, df["rms_deg"].to_numpy())
        run_label = re.search(r"X\d{3}", run)
        run_label = run_label.group(0) if run_label else run
        ax.set_xticks(x)
        ax.set_xticklabels([f"{m.upper()} ({run_label})" for m in df["method"]])
        ax.set_ylabel("Error (degrees)")
        ax.set_title(f"Task 3: Attitude Error Comparison (RUN={run})")
        for i, row in df.iterrows():
            ax.text(
                i,
                row["rms_deg"],
                f"mean={row['mean_deg']:.1f}\nmedian={row['median_deg']:.1f}\np95={row['p95_deg']:.1f}",
                ha="center",
                va="bottom",
                fontsize=8,
            )
        fig.tight_layout()
        from utils.matlab_fig_export import save_matlab_fig
        save_matlab_fig(fig, str((outdir / f"{run}_task3_errors_comparison").with_suffix('')))
        plt.close(fig)
