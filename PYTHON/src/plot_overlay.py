from pathlib import Path
from typing import Optional, Tuple


import numpy as np
import matplotlib.pyplot as plt


def _plot_overlay_interactive_safe(*args, **kwargs):
    """Safely import and call :func:`plot_overlay_interactive`.

    Plotly and its optional dependencies (such as Kaleido for static export)
    are not required for the rest of this project, so we avoid importing them at
    module load time.  Instead, this helper lazily imports the interactive
    plotting backend when needed and provides a clear error message if the
    dependencies are missing.
    """

    try:  # pragma: no cover - import-time failure is environment specific
        from plot_overlay_interactive import (
            PLOTLY_AVAILABLE,
            plot_overlay_interactive,
        )
    except Exception as e:  # pragma: no cover - graceful degradation
        raise RuntimeError(
            "Interactive plotting requires Plotly and its extras. "
            "Install them with `pip install plotly kaleido`."
        ) from e

    if not PLOTLY_AVAILABLE:  # pragma: no cover - runtime check
        raise RuntimeError(
            "Plotly is not available. Install it with `pip install plotly` to "
            "enable interactive plotting."
        )

    return plot_overlay_interactive(*args, **kwargs)


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
    interactive: bool = False,
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
        figure. Defaults to ``"_overlay_state.pdf"`` when truth data is
        supplied and ``"_overlay.pdf"`` otherwise.
    filename : str or None, optional
        Full filename (relative to ``out_dir``) for the saved figure. When
        provided, overrides the ``method``/``frame`` naming scheme and the
        ``suffix`` parameter.
    include_measurements : bool, optional
        Plot measured IMU and GNSS series when ``True`` (default). When ``False``
        only the fused estimate and optional truth data are shown.
    interactive : bool, optional
        Create interactive Plotly plots when ``True`` (default). Falls back to
        static matplotlib plots if Plotly is not available.
    """
    # Try interactive plotting first if requested
    if interactive:
        try:
            _plot_overlay_interactive_safe(
                frame=frame,
                method=method,
                t_imu=t_imu,
                pos_imu=pos_imu,
                vel_imu=vel_imu,
                acc_imu=acc_imu,
                t_gnss=t_gnss,
                pos_gnss=pos_gnss,
                vel_gnss=vel_gnss,
                acc_gnss=acc_gnss,
                t_fused=t_fused,
                pos_fused=pos_fused,
                vel_fused=vel_fused,
                acc_fused=acc_fused,
                out_dir=out_dir,
                truth=truth,
                t_truth=t_truth,
                pos_truth=pos_truth,
                vel_truth=vel_truth,
                acc_truth=acc_truth,
                filename=filename,
                include_measurements=include_measurements,
                save_static=True,
            )
            print(f"✓ Created interactive plot for {method} {frame} frame")
            return
        except Exception as e:  # pragma: no cover - graceful fallback
            print(f"Interactive plotting failed: {e}")
            print("Falling back to static matplotlib plots...")
    
    # Original matplotlib plotting code (static plots)
    _plot_overlay_static(
        frame=frame,
        method=method,
        t_imu=t_imu,
        pos_imu=pos_imu,
        vel_imu=vel_imu,
        acc_imu=acc_imu,
        t_gnss=t_gnss,
        pos_gnss=pos_gnss,
        vel_gnss=vel_gnss,
        acc_gnss=acc_gnss,
        t_fused=t_fused,
        pos_fused=pos_fused,
        vel_fused=vel_fused,
        acc_fused=acc_fused,
        out_dir=out_dir,
        truth=truth,
        t_truth=t_truth,
        pos_truth=pos_truth,
        vel_truth=vel_truth,
        acc_truth=acc_truth,
        suffix=suffix,
        filename=filename,
        include_measurements=include_measurements,
    )


def _plot_overlay_static(
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
    """Create static matplotlib overlay plots (original implementation)."""
    if truth is not None:
        t_truth, pos_truth, vel_truth, acc_truth = truth

    if suffix is None:
        suffix = "_overlay_state.pdf" if t_truth is not None else "_overlay.pdf"

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
        out_path = Path(out_dir) / filename
    else:
        out_path = Path(out_dir) / f"{method}_{frame}{suffix}"

    fname_pdf = out_path.with_suffix(".pdf")
    fname_png = out_path.with_suffix(".png")
    # Save only PNG and MAT using the PDF stem as base
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(Path(fname_pdf).with_suffix("")))
    try:
        from utils import save_plot_mat
        save_plot_mat(fig, str(out_path.with_suffix(".mat")))
    except Exception:
        pass
    print(f"Saved overlay figure {fname_pdf}")
    plt.close(fig)
