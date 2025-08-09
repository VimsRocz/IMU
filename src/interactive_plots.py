"""Interactive plotting helpers for Task 5 and Task 6.

These functions try to use Plotly for rich, browser-based interactivity.
If Plotly is not installed, they fall back to Matplotlib and show the
plots in a window.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


def _try_import_plotly():
    try:
        import plotly.graph_objects as go  # type: ignore
        from plotly.subplots import make_subplots  # type: ignore
        import plotly.io as pio  # type: ignore

        return go, make_subplots, pio
    except Exception:
        return None, None, None


def show_task5_all_frames(
    *,
    tag: str,
    method: str,
    # time axes
    t_gnss: np.ndarray,
    t_imu: np.ndarray,
    # NED data
    gnss_pos_ned: np.ndarray,
    gnss_vel_ned: np.ndarray,
    gnss_acc_ned: np.ndarray,
    fused_pos_ned: np.ndarray,
    fused_vel_ned: np.ndarray,
    fused_acc_ned: np.ndarray,
    truth_pos_ned: Optional[np.ndarray] = None,
    truth_vel_ned: Optional[np.ndarray] = None,
    truth_acc_ned: Optional[np.ndarray] = None,
    # ECEF data
    gnss_pos_ecef: np.ndarray,
    gnss_vel_ecef: np.ndarray,
    gnss_acc_ecef: np.ndarray,
    fused_pos_ecef: np.ndarray,
    fused_vel_ecef: np.ndarray,
    fused_acc_ecef: np.ndarray,
    truth_pos_ecef: Optional[np.ndarray] = None,
    truth_vel_ecef: Optional[np.ndarray] = None,
    truth_acc_ecef: Optional[np.ndarray] = None,
    # Body data
    gnss_pos_body: np.ndarray,
    gnss_vel_body: np.ndarray,
    gnss_acc_body: np.ndarray,
    fused_pos_body: np.ndarray,
    fused_vel_body: np.ndarray,
    fused_acc_body: np.ndarray,
    truth_pos_body: Optional[np.ndarray] = None,
    truth_vel_body: Optional[np.ndarray] = None,
    truth_acc_body: Optional[np.ndarray] = None,
) -> None:
    """Show interactive 3x3 subplots for NED, ECEF, Body frames.

    Mirrors Task 4/5 "all_*" figure layout with Position, Velocity,
    Acceleration across the 3 axes.
    """

    go, make_subplots, pio = _try_import_plotly()
    prefer_truth = (truth_pos_ned is not None) or (truth_pos_ecef is not None) or (truth_pos_body is not None)
    titles = {
        "ned": f"Task 5 – {method} – NED Frame (Fused GNSS+IMU only)",
        "ecef": f"Task 5 – {method} – ECEF Frame (Fused GNSS+IMU only)",
        "body": f"Task 5 – {method} – Body Frame (Fused GNSS+IMU only)",
    }
    dirs_ned = ["N", "E", "D"]
    dirs_ecef = ["X", "Y", "Z"]
    dirs_body = ["X", "Y", "Z"]

    if go is not None:
        # Prefer browser for rich interactivity
        try:
            pio.renderers.default = "browser"  # open in default browser
        except Exception:
            pass

        # NED
        fig_ned = make_subplots(rows=3, cols=3, subplot_titles=
                                [f"Position {a}" for a in dirs_ned]
                                + [f"Velocity V{a}" for a in dirs_ned]
                                + [f"Acceleration {a}" for a in dirs_ned])
        for j in range(3):
            # Position (fused only)
            fig_ned.add_trace(go.Scatter(x=t_imu, y=fused_pos_ned[:, j], name="Fused pos", line=dict(color="#1f77b4")), row=1, col=j+1)
            # Velocity
            fig_ned.add_trace(go.Scatter(x=t_imu, y=fused_vel_ned[:, j], name="Fused vel", line=dict(color="#ff7f0e")), row=2, col=j+1)
            # Acceleration
            fig_ned.add_trace(go.Scatter(x=t_imu, y=fused_acc_ned[:, j], name="Fused acc", line=dict(color="#2ca02c")), row=3, col=j+1)
        fig_ned.update_layout(height=900, width=1200, title_text=titles["ned"], legend_title_text="Signals")
        fig_ned.show()

        # ECEF
        fig_ecef = make_subplots(rows=3, cols=3, subplot_titles=
                                 [f"Position {a}_ECEF" for a in dirs_ecef]
                                 + [f"Velocity V{a}_ECEF" for a in dirs_ecef]
                                 + [f"Acceleration {a}_ECEF" for a in dirs_ecef])
        for j in range(3):
            fig_ecef.add_trace(go.Scatter(x=t_imu, y=fused_pos_ecef[:, j], name="Fused pos", line=dict(color="#1f77b4")), row=1, col=j+1)
            fig_ecef.add_trace(go.Scatter(x=t_imu, y=fused_vel_ecef[:, j], name="Fused vel", line=dict(color="#ff7f0e")), row=2, col=j+1)
            fig_ecef.add_trace(go.Scatter(x=t_imu, y=fused_acc_ecef[:, j], name="Fused acc", line=dict(color="#2ca02c")), row=3, col=j+1)
        fig_ecef.update_layout(height=900, width=1200, title_text=titles["ecef"], legend_title_text="Signals")
        fig_ecef.show()

        # Body
        fig_body = make_subplots(rows=3, cols=3, subplot_titles=
                                 [f"Position r{a}_body" for a in dirs_body]
                                 + [f"Velocity v{a}_body" for a in dirs_body]
                                 + [f"Acceleration A{a}_body" for a in dirs_body])
        for j in range(3):
            fig_body.add_trace(go.Scatter(x=t_imu, y=fused_pos_body[:, j], name="Fused pos", line=dict(color="#1f77b4")), row=1, col=j+1)
            fig_body.add_trace(go.Scatter(x=t_imu, y=fused_vel_body[:, j], name="Fused vel", line=dict(color="#ff7f0e")), row=2, col=j+1)
            fig_body.add_trace(go.Scatter(x=t_imu, y=fused_acc_body[:, j], name="Fused acc", line=dict(color="#2ca02c")), row=3, col=j+1)
        fig_body.update_layout(height=900, width=1200, title_text=titles["body"], legend_title_text="Signals")
        fig_body.show()
        return

    # Matplotlib fallback (simple pop-up windows)
    import matplotlib.pyplot as plt

    def _mat_show(title: str, dirs: list[str], gnss_tuple, fused_tuple):
        fig, axes = plt.subplots(3, 3, figsize=(15, 10))
        (g_pos, g_vel, g_acc) = gnss_tuple
        (f_pos, f_vel, f_acc) = fused_tuple
        for i in range(3):
            # Position
            ax = axes[0, i]
            ax.plot(t_gnss, g_pos[:, i], "k-", label="GNSS")
            ax.plot(t_imu, f_pos[:, i], label="Fused")
            ax.set_title(f"Position {dirs[i]}")
            ax.legend(loc="best")
            # Velocity
            ax = axes[1, i]
            ax.plot(t_gnss, g_vel[:, i], "k-", label="GNSS")
            ax.plot(t_imu, f_vel[:, i], label="Fused")
            ax.set_title(f"Velocity {dirs[i]}")
            ax.legend(loc="best")
            # Acceleration
            ax = axes[2, i]
            ax.plot(t_gnss, g_acc[:, i], "k-", label="GNSS")
            ax.plot(t_imu, f_acc[:, i], label="Fused")
            ax.set_title(f"Acceleration {dirs[i]}")
            ax.legend(loc="best")
        fig.suptitle(title)
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        plt.show(block=False)

    _mat_show(titles["ned"], dirs_ned,
              (gnss_pos_ned, gnss_vel_ned, gnss_acc_ned),
              (fused_pos_ned, fused_vel_ned, fused_acc_ned))
    _mat_show(titles["ecef"], dirs_ecef,
              (gnss_pos_ecef, gnss_vel_ecef, gnss_acc_ecef),
              (fused_pos_ecef, fused_vel_ecef, fused_acc_ecef))
    _mat_show(titles["body"], dirs_body,
              (gnss_pos_body, gnss_vel_body, gnss_acc_body),
              (fused_pos_body, fused_vel_body, fused_acc_body))
    plt.show(block=True)


def show_task6_interactive(
    *,
    tag: str,
    method: str,
    t: np.ndarray,
    pos_ned: np.ndarray,
    vel_ned: np.ndarray,
    pos_ecef: Optional[np.ndarray] = None,
    vel_ecef: Optional[np.ndarray] = None,
    pos_body: Optional[np.ndarray] = None,
    vel_body: Optional[np.ndarray] = None,
    euler_deg: Optional[np.ndarray] = None,
) -> None:
    """Interactive Task 6 summary plots.

    Displays NED position/velocity and, if provided, ECEF/body position/velocity
    plus roll/pitch/yaw over time.
    """

    go, make_subplots, pio = _try_import_plotly()

    if go is not None:
        try:
            pio.renderers.default = "browser"
        except Exception:
            pass

        # NED pos/vel
        fig = make_subplots(rows=2, cols=3, subplot_titles=[
            "Position N", "Position E", "Position D",
            "Velocity N", "Velocity E", "Velocity D",
        ])
        labels = ["N", "E", "D"]
        for j in range(3):
            fig.add_trace(go.Scatter(x=t, y=pos_ned[:, j], name=f"pos {labels[j]}"), row=1, col=j+1)
            fig.add_trace(go.Scatter(x=t, y=vel_ned[:, j], name=f"vel {labels[j]}"), row=2, col=j+1)
        fig.update_layout(height=700, width=1100,
                          title_text=f"Task 6 – {method} – NED Position/Velocity")
        fig.show()

        # ECEF or Body if present
        if pos_ecef is not None and vel_ecef is not None:
            labels = ["X", "Y", "Z"]
            fig2 = make_subplots(rows=2, cols=3, subplot_titles=[
                "Position X_ECEF", "Position Y_ECEF", "Position Z_ECEF",
                "Velocity VX_ECEF", "Velocity VY_ECEF", "Velocity VZ_ECEF",
            ])
            for j in range(3):
                fig2.add_trace(go.Scatter(x=t, y=pos_ecef[:, j], name=f"pos {labels[j]}"), row=1, col=j+1)
                fig2.add_trace(go.Scatter(x=t, y=vel_ecef[:, j], name=f"vel {labels[j]}"), row=2, col=j+1)
            fig2.update_layout(height=700, width=1100,
                               title_text=f"Task 6 – {method} – ECEF Position/Velocity")
            fig2.show()
        if pos_body is not None and vel_body is not None:
            labels = ["X", "Y", "Z"]
            fig3 = make_subplots(rows=2, cols=3, subplot_titles=[
                "Position rX_body", "Position rY_body", "Position rZ_body",
                "Velocity vX_body", "Velocity vY_body", "Velocity vZ_body",
            ])
            for j in range(3):
                fig3.add_trace(go.Scatter(x=t, y=pos_body[:, j], name=f"pos {labels[j]}"), row=1, col=j+1)
                fig3.add_trace(go.Scatter(x=t, y=vel_body[:, j], name=f"vel {labels[j]}"), row=2, col=j+1)
            fig3.update_layout(height=700, width=1100,
                               title_text=f"Task 6 – {method} – Body Position/Velocity")
            fig3.show()
        if euler_deg is not None:
            fig4 = make_subplots(rows=1, cols=1, subplot_titles=["Attitude (deg)"])
            fig4.add_trace(go.Scatter(x=t, y=euler_deg[:, 0], name="Roll"))
            fig4.add_trace(go.Scatter(x=t, y=euler_deg[:, 1], name="Pitch"))
            fig4.add_trace(go.Scatter(x=t, y=euler_deg[:, 2], name="Yaw"))
            fig4.update_layout(height=450, width=1100,
                               title_text=f"Task 6 – {method} – Attitude Angles")
            fig4.show()
        return

    # Matplotlib fallback
    import matplotlib.pyplot as plt
    plt.figure(figsize=(11, 7))
    labels = ["N", "E", "D"]
    for i in range(3):
        ax = plt.subplot(2, 3, i + 1)
        ax.plot(t, pos_ned[:, i], label=f"pos {labels[i]}")
        ax.set_title(f"Position {labels[i]}")
        ax = plt.subplot(2, 3, 3 + i + 1)
        ax.plot(t, vel_ned[:, i], label=f"vel {labels[i]}")
        ax.set_title(f"Velocity {labels[i]}")
    plt.suptitle(f"Task 6 – {method} – NED Position/Velocity")
    plt.tight_layout()
    plt.show(block=False)

    if pos_ecef is not None and vel_ecef is not None:
        plt.figure(figsize=(11, 7))
        labels = ["X", "Y", "Z"]
        for i in range(3):
            ax = plt.subplot(2, 3, i + 1)
            ax.plot(t, pos_ecef[:, i], label=f"pos {labels[i]}")
            ax.set_title(f"Position {labels[i]} ECEF")
            ax = plt.subplot(2, 3, 3 + i + 1)
            ax.plot(t, vel_ecef[:, i], label=f"vel {labels[i]}")
            ax.set_title(f"Velocity V{labels[i]} ECEF")
        plt.suptitle(f"Task 6 – {method} – ECEF Position/Velocity")
        plt.tight_layout()
        plt.show(block=False)

    if pos_body is not None and vel_body is not None:
        plt.figure(figsize=(11, 7))
        labels = ["X", "Y", "Z"]
        for i in range(3):
            ax = plt.subplot(2, 3, i + 1)
            ax.plot(t, pos_body[:, i], label=f"pos {labels[i]}")
            ax.set_title(f"Position r{labels[i]} body")
            ax = plt.subplot(2, 3, 3 + i + 1)
            ax.plot(t, vel_body[:, i], label=f"vel {labels[i]}")
            ax.set_title(f"Velocity v{labels[i]} body")
        plt.suptitle(f"Task 6 – {method} – Body Position/Velocity")
        plt.tight_layout()
        plt.show(block=False)

    if euler_deg is not None:
        plt.figure(figsize=(11, 4))
        plt.plot(t, euler_deg[:, 0], label="Roll")
        plt.plot(t, euler_deg[:, 1], label="Pitch")
        plt.plot(t, euler_deg[:, 2], label="Yaw")
        plt.title(f"Task 6 – {method} – Attitude Angles")
        plt.xlabel("Time (s)")
        plt.ylabel("Angle (deg)")
        plt.legend(loc="best")
        plt.tight_layout()
        plt.show(block=True)
