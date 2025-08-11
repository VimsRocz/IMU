from pathlib import Path
from typing import Optional, Tuple


import numpy as np
import matplotlib.pyplot as plt
from python.utils.save_plot_all import save_plot_all


def _norm(v: np.ndarray) -> np.ndarray:
    return np.linalg.norm(v, axis=1)


def plot_overlay(
    frame: str,
    method: str,
    t_imu: np.ndarray,
    pos_imu: np.ndarray,
    vel_imu: np.ndarray,
    acc_imu: np.ndarray,
    t_gnss: np.ndarray,
    pos_gnss: np.ndarray,
    vel_gnss: np.ndarray,
    acc_gnss: np.ndarray,
    t_fused: np.ndarray,
    pos_fused: np.ndarray,
    vel_fused: np.ndarray,
    acc_fused: np.ndarray,
    out_dir: str,
    truth: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = None,
    *,
    t_truth: Optional[np.ndarray] = None,
    pos_truth: Optional[np.ndarray] = None,
    vel_truth: Optional[np.ndarray] = None,
    acc_truth: Optional[np.ndarray] = None,
    suffix: Optional[str] = None,
    filename: Optional[str] = None,
    include_measurements: bool = True,
) -> None:
    """Save a 3x3 overlay plot comparing measured IMU, measured GNSS and
    fused GNSS+IMU tracks.

    Parameters
    ----------
    t_truth, pos_truth, vel_truth, acc_truth : np.ndarray or None, optional
        Ground-truth time, position, velocity and acceleration samples. When
        provided, a black line labelled ``"Truth"`` is drawn in each subplot.
    mode : str, default "state"
        Only "state" is supported.  The "truth" option is obsolete.
    suffix : str or None, optional
        Filename suffix appended to ``"{method}_{frame}"`` when saving the
        figure. Defaults to ``"_overlay_state"`` when truth data is
        supplied and ``"_overlay"`` otherwise. Extensions are added based on
        the ``formats`` argument to :func:`save_plot_all`.
    filename : str or None, optional
        Full filename (relative to ``out_dir``) for the saved figure. When
        provided, overrides the ``method``/``frame`` naming scheme and the
        ``suffix`` parameter.
    include_measurements : bool, optional
        Plot measured IMU and GNSS series when ``True`` (default). When ``False``
        only the fused estimate and optional truth data are shown.
    """
    if truth is not None:
        t_truth, pos_truth, vel_truth, acc_truth = truth

    if suffix is None:
        suffix = "_overlay_state" if t_truth is not None else "_overlay"

    axis_labels = {
        "NED": ["\u0394N [m]", "\u0394E [m]", "\u0394D [m]"],
        "ECEF": ["X", "Y", "Z"],
        "Body": ["X", "Y", "Z"],
    }
    cols = axis_labels.get(frame, ["X", "Y", "Z"])

    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)

    datasets = [
        (pos_imu, pos_gnss, pos_fused, pos_truth, "Position [m]"),
        (vel_imu, vel_gnss, vel_fused, vel_truth, "Velocity [m/s]"),
        (acc_imu, acc_gnss, acc_fused, acc_truth, "Acceleration [m/s$^2$]"),
    ]

    color_map = {
        "Truth": "black",
        "Fused": "tab:blue",
        "GNSS": "tab:green",
        "IMU": "tab:orange",
    }

    has_acc_truth = acc_truth is not None and len(acc_truth) > 0

    for row, (imu, gnss, fused, truth, ylab) in enumerate(datasets):
        for col, axis in enumerate(cols):
            ax = axes[row, col]
            values = [fused[:, col]]
            if include_measurements:
                ax.plot(
                    t_gnss,
                    gnss[:, col],
                    color=color_map["GNSS"],
                    label="Measured GNSS",
                )
                ax.plot(
                    t_imu,
                    imu[:, col],
                    linestyle="--",
                    color=color_map["IMU"],
                    label="Measured IMU",
                )
                values.append(gnss[:, col])
                values.append(imu[:, col])
            if (
                t_truth is not None
                and truth is not None
                and not (row == 2 and not has_acc_truth)
            ):
                ax.plot(
                    t_truth,
                    truth[:, col],
                    color=color_map["Truth"],
                    label="Truth",
                )
                values.append(truth[:, col])
            ax.plot(
                t_fused,
                fused[:, col],
                color=color_map["Fused"],
                linestyle=":",
                label=f"Fused GNSS+IMU ({method})",
            )
            values = np.concatenate(values)
            lim = np.max(np.abs(values)) * 1.1
            ax.set_ylim(-lim, lim)
            handles, labels_ = ax.get_legend_handles_labels()
            keep = [("Fused" in l) or ("Truth" in l) for l in labels_]
            ax.legend(
                [h for h, k in zip(handles, keep) if k],
                [l for l, k in zip(labels_, keep) if k],
                loc="upper right",
                fontsize=8,
            )
            if row == 0:
                ax.set_title(axis)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel("Time [s]")

    if t_truth is not None:
        title = f"Task 6 – {method} – {frame} Frame (Fused vs. Truth)"
    elif include_measurements:
        title = f"Task 6 – {method} – {frame} Frame (Fused vs. Measured GNSS)"
    else:
        title = f"Task 6 – {method} – {frame} Frame (Fused)"

    if not has_acc_truth and t_truth is not None:
        axes[2, 0].text(
            0.01,
            0.05,
            "No acceleration for Truth",
            transform=axes[2, 0].transAxes,
            fontsize=8,
            color="gray",
        )

    fig.suptitle(title)
    fig.tight_layout(rect=[0, 0, 1, 0.9])
    if filename is not None:
        out_path = Path(out_dir) / Path(filename).with_suffix("")
    else:
        out_path = Path(out_dir) / f"{method}_{frame}{suffix}"

    save_plot_all(fig, str(out_path), formats=(".pickle", ".fig"))
    print(f"Saved overlay figure {out_path}.pickle")
    plt.close(fig)
