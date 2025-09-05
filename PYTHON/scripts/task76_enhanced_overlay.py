#!/usr/bin/env python3
"""Task 7.6 Enhanced Overlays and Innovation Gating for noisy runs (e.g., X002).

Inputs a rich NPZ produced by the pipeline (e.g., *_kf_output.npz) and generates:

- Truth vs Fused overlays in NED/ECEF/Body (reusing saved series), plus an
  automatic zoom on the divergence segment based on error magnitude.
- Innovation plots with ±N·sigma gating bands using S ≈ H P H' + R, where P is
  taken from the closest IMU step to each GNSS update. R can be provided or
  defaults to conservative diagonals.
- Normalized innovation test ratios (per-axis), with outliers highlighted.
- A concise summary CSV of peak errors and gating exceedances.

This script is read-only and does not modify the pipeline. Use it to diagnose
noisy runs like X002 quickly and consistently.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt


def _load_npz(path: Path) -> dict[str, np.ndarray]:
    d = np.load(path, allow_pickle=True)
    # Convert to plain dict with ndarray/scalars for convenience
    out: dict[str, np.ndarray] = {}
    for k in d.files:
        out[k] = d[k]
    return out


def _infer_run_id(npz_path: Path, data: dict) -> str:
    # Prefer saved tag, otherwise derive from filename stem
    tag = data.get("tag")
    if tag is not None and np.asarray(tag).size:
        try:
            return str(np.asarray(tag).squeeze())
        except Exception:
            pass
    return npz_path.stem.replace("_kf_output", "")


def _ensure_out_dir(p: Path) -> Path:
    p.mkdir(parents=True, exist_ok=True)
    return p


def _overlay_plot(time: np.ndarray,
                  pos_est: np.ndarray, pos_truth: np.ndarray,
                  vel_est: np.ndarray, vel_truth: np.ndarray,
                  labels: list[str], frame_name: str,
                  out_base: Path) -> list[Path]:
    # Decimate lightly if huge
    n = len(time)
    stride = int(np.ceil(n / 200000)) if n > 200000 else 1
    t = time[::stride]
    p_e, p_t = pos_est[::stride], pos_truth[::stride]
    v_e, v_t = vel_est[::stride], vel_truth[::stride]

    fig, axes = plt.subplots(2, 3, figsize=(11, 5), sharex=True)
    for i in range(3):
        axes[0, i].plot(t, p_e[:, i], label="Fused")
        axes[0, i].plot(t, p_t[:, i], "--", label="Truth")
        axes[0, i].set_ylabel("Position [m]")
        axes[0, i].set_title(labels[i])
        axes[0, i].grid(True, alpha=0.3)
        axes[1, i].plot(t, v_e[:, i], label="Fused")
        axes[1, i].plot(t, v_t[:, i], "--", label="Truth")
        axes[1, i].set_ylabel("Velocity [m/s]")
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].grid(True, alpha=0.3)
    handles, labels_h = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels_h, ncol=2, loc="upper center")
    fig.suptitle(f"Task 7.6 – Truth vs Fused ({frame_name} Frame)")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    png = out_base.with_suffix(".png")
    pdf = out_base.with_suffix(".pdf")
    fig.savefig(png, dpi=200, bbox_inches="tight")
    fig.savefig(pdf, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return [png, pdf]


def _zoom_on_divergence(time: np.ndarray,
                        pos_est: np.ndarray, pos_truth: np.ndarray,
                        vel_est: np.ndarray, vel_truth: np.ndarray,
                        labels: list[str], frame_name: str,
                        out_base: Path,
                        err_thresh: float | None = None,
                        tail_seconds: float = 60.0) -> list[Path]:
    # Compute magnitude errors
    e_pos = np.linalg.norm(pos_est - pos_truth, axis=1)
    e_vel = np.linalg.norm(vel_est - vel_truth, axis=1)
    # Pick a window: either first time error exceeds thresh, or last tail_seconds
    t0 = time[0]
    t1 = time[-1]
    if err_thresh is None:
        # Default to last segment
        t_start = max(t0, t1 - tail_seconds)
    else:
        idx = np.where((e_vel > err_thresh) | (e_pos > 5 * err_thresh))[0]
        t_start = time[idx[0]] if idx.size else max(t0, t1 - tail_seconds)

    mask = time >= t_start
    t = time[mask]
    p_e, p_t = pos_est[mask], pos_truth[mask]
    v_e, v_t = vel_est[mask], vel_truth[mask]

    fig, axes = plt.subplots(2, 3, figsize=(11, 5), sharex=True)
    for i in range(3):
        axes[0, i].plot(t, p_e[:, i], label="Fused")
        axes[0, i].plot(t, p_t[:, i], "--", label="Truth")
        axes[0, i].set_ylabel("Position [m]")
        axes[0, i].set_title(labels[i])
        axes[0, i].grid(True, alpha=0.3)
        axes[1, i].plot(t, v_e[:, i], label="Fused")
        axes[1, i].plot(t, v_t[:, i], "--", label="Truth")
        axes[1, i].set_ylabel("Velocity [m/s]")
        axes[1, i].set_xlabel("Time [s]")
        axes[1, i].grid(True, alpha=0.3)
    # Add error magnitude subplot overlay for context
    ax_inset = fig.add_axes([0.75, 0.6, 0.22, 0.3])
    ax_inset.plot(time - time[0], e_vel, label="|vel err|")
    ax_inset.plot(time - time[0], e_pos, label="|pos err|", alpha=0.6)
    if err_thresh is not None:
        ax_inset.axhline(err_thresh, color="r", linestyle=":", label="vel thresh")
    ax_inset.set_title("Error magnitudes")
    ax_inset.grid(True, alpha=0.3)
    # Legend only if few entries
    handles, labels_h = axes[0, 0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels_h, ncol=2, loc="upper center")
    fig.suptitle(f"Task 7.6 – Zoom on Divergence ({frame_name})")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    png = out_base.with_name(out_base.name + "_zoom").with_suffix(".png")
    pdf = out_base.with_name(out_base.name + "_zoom").with_suffix(".pdf")
    fig.savefig(png, dpi=200, bbox_inches="tight")
    fig.savefig(pdf, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return [png, pdf]


def _make_R_blocks(r_pos_diag: float, r_vel_diag: float) -> tuple[np.ndarray, np.ndarray]:
    Rpos = np.eye(3) * float(r_pos_diag)
    Rvel = np.eye(3) * float(r_vel_diag)
    return Rpos, Rvel


def _plot_innovations_with_gates(innov_pos: np.ndarray, innov_vel: np.ndarray,
                                 t_updates: np.ndarray,
                                 P_hist: np.ndarray | None,
                                 r_pos_diag: float, r_vel_diag: float,
                                 out_base: Path, gate_sigma: float = 3.0) -> list[Path]:
    m = innov_pos.shape[0]
    idx = np.arange(m)
    labels = ["X/N", "Y/E", "Z/D"]

    # Compute ±N·sigma bands using S ≈ diag(HPH'+R). If P_hist is present,
    # pick the closest P for each update; else, use sample std as fallback.
    sig_pos = np.zeros_like(innov_pos)
    sig_vel = np.zeros_like(innov_vel)
    if P_hist is not None and P_hist.ndim == 3 and P_hist.shape[1] >= 6:
        # Map update time to nearest P index assuming t_updates aligns to estimator time
        # If update times are sample indices already, idx->P directly
        # Fallback: evenly spread over available P frames
        Pn = P_hist.shape[0]
        map_idx = np.minimum((idx * (Pn - 1) // max(m - 1, 1)).astype(int), Pn - 1)
        Rpos, Rvel = _make_R_blocks(r_pos_diag, r_vel_diag)
        for k in range(m):
            Pk = P_hist[map_idx[k]]
            # Position and velocity 3x3 blocks
            S_pos = Pk[0:3, 0:3] + Rpos
            S_vel = Pk[3:6, 3:6] + Rvel
            sig_pos[k] = np.sqrt(np.clip(np.diag(S_pos), 0.0, None))
            sig_vel[k] = np.sqrt(np.clip(np.diag(S_vel), 0.0, None))
    else:
        # Empirical fallback
        sig_pos[:] = np.std(innov_pos, axis=0, keepdims=True)
        sig_vel[:] = np.std(innov_vel, axis=0, keepdims=True)

    # Plot components with bands
    fig, axes = plt.subplots(3, 2, figsize=(12, 7), sharex=True)
    for i in range(3):
        axp = axes[i, 0]
        axv = axes[i, 1]
        axp.plot(idx, innov_pos[:, i], label="innovation")
        axp.plot(idx, gate_sigma * sig_pos[:, i], "r--", label=f"±{gate_sigma}σ")
        axp.plot(idx, -gate_sigma * sig_pos[:, i], "r--")
        axp.set_ylabel(f"Pos {labels[i]} [m]")
        axp.grid(True, alpha=0.3)
        axv.plot(idx, innov_vel[:, i], label="innovation")
        axv.plot(idx, gate_sigma * sig_vel[:, i], "r--", label=f"±{gate_sigma}σ")
        axv.plot(idx, -gate_sigma * sig_vel[:, i], "r--")
        axv.set_ylabel(f"Vel {labels[i]} [m/s]")
        axv.grid(True, alpha=0.3)
        if i == 0:
            axp.legend(loc="upper right")
            axv.legend(loc="upper right")
    axes[-1, 0].set_xlabel("GNSS update index")
    axes[-1, 1].set_xlabel("GNSS update index")
    fig.suptitle("Task 7.6 – Innovations with ±Nσ gates")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    png1 = out_base.with_name(out_base.name + "_innov_bands").with_suffix(".png")
    pdf1 = out_base.with_name(out_base.name + "_innov_bands").with_suffix(".pdf")
    fig.savefig(png1, dpi=200, bbox_inches="tight")
    fig.savefig(pdf1, dpi=200, bbox_inches="tight")
    plt.close(fig)

    # Normalized innovation squared (per-axis scalar proxy)
    # r_i^2 / S_ii; mark exceedances
    nis_pos = (innov_pos ** 2) / np.maximum(sig_pos ** 2, 1e-12)
    nis_vel = (innov_vel ** 2) / np.maximum(sig_vel ** 2, 1e-12)
    thresh = gate_sigma ** 2
    # Plot
    fig, axes = plt.subplots(3, 2, figsize=(12, 7), sharex=True)
    for i in range(3):
        axes[i, 0].plot(idx, nis_pos[:, i], label="NIS component")
        axes[i, 0].axhline(thresh, color="r", linestyle=":", label=f"{gate_sigma}σ gate")
        axes[i, 0].set_ylabel(f"Pos NIS[{i}] [-]")
        axes[i, 0].grid(True, alpha=0.3)
        axes[i, 1].plot(idx, nis_vel[:, i], label="NIS component")
        axes[i, 1].axhline(thresh, color="r", linestyle=":", label=f"{gate_sigma}σ gate")
        axes[i, 1].set_ylabel(f"Vel NIS[{i}] [-]")
        axes[i, 1].grid(True, alpha=0.3)
        if i == 0:
            axes[i, 0].legend(loc="upper right")
            axes[i, 1].legend(loc="upper right")
    axes[-1, 0].set_xlabel("GNSS update index")
    axes[-1, 1].set_xlabel("GNSS update index")
    fig.suptitle("Task 7.6 – Normalized Innovation (per-axis)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    png2 = out_base.with_name(out_base.name + "_nis").with_suffix(".png")
    pdf2 = out_base.with_name(out_base.name + "_nis").with_suffix(".pdf")
    fig.savefig(png2, dpi=200, bbox_inches="tight")
    fig.savefig(pdf2, dpi=200, bbox_inches="tight")
    plt.close(fig)

    # Save a concise CSV summary of exceedances
    try:
        import csv
        csv_path = out_base.with_name(out_base.name + "_innov_summary").with_suffix(".csv")
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["type", "axis", "exceed_count", "max_NIS", "max_abs_innov"])
            for name, nis, innov in (
                ("pos", nis_pos, innov_pos),
                ("vel", nis_vel, innov_vel),
            ):
                for i, axis in enumerate(["x/n", "y/e", "z/d"]):
                    w.writerow([
                        name,
                        axis,
                        int(np.sum((nis[:, i] > thresh) & np.isfinite(nis[:, i]))),
                        f"{np.nanmax(nis[:, i]):.3f}",
                        f"{np.nanmax(np.abs(innov[:, i])):.3f}",
                    ])
    except Exception:
        pass

    return [png1, pdf1, png2, pdf2]


def main() -> None:
    ap = argparse.ArgumentParser(description="Task 7.6 enhanced overlay and innovation gating plots")
    ap.add_argument("--npz", required=True, help="Path to *_kf_output.npz for a run")
    ap.add_argument("--output-dir", default="results", help="Directory for output plots")
    ap.add_argument("--gate-sigma", type=float, default=3.0, help="Gating sigma for bands and NIS threshold")
    ap.add_argument("--r-pos", type=float, default=25.0, help="Diagonal R for position [m^2]")
    ap.add_argument("--r-vel", type=float, default=1.0, help="Diagonal R for velocity [(m/s)^2]")
    ap.add_argument("--zoom-thresh", type=float, default=None, help="Velocity error threshold [m/s] to trigger zoom window")
    ap.add_argument("--zoom-tail", type=float, default=60.0, help="Fallback: last N seconds for zoom window")
    args = ap.parse_args()

    npz_path = Path(args.npz)
    data = _load_npz(npz_path)
    run_id = _infer_run_id(npz_path, data)
    out_dir = _ensure_out_dir(Path(args.output_dir))

    # Time base
    t = data.get("t_rel_imu") or data.get("time") or data.get("t")
    if t is None:
        raise ValueError("time/t/t_rel_imu not found in NPZ")
    t = np.asarray(t).squeeze()
    t = t - float(t[0])

    # --- Overlays -----------------------------------------------------------
    # NED
    p_fn = data.get("fused_pos") or data.get("pos_ned")
    v_fn = data.get("fused_vel") or data.get("vel_ned")
    pt_fn = data.get("truth_pos_ned_i") or data.get("truth_pos_ned")
    vt_fn = data.get("truth_vel_ned_i") or data.get("truth_vel_ned")
    if p_fn is not None and v_fn is not None and pt_fn is not None and vt_fn is not None:
        base = out_dir / f"{run_id}_task7_6_overlay_NED_enhanced"
        _overlay_plot(t, p_fn, pt_fn, v_fn, vt_fn, ["North", "East", "Down"], "NED", base)
        _zoom_on_divergence(t, p_fn, pt_fn, v_fn, vt_fn, ["North", "East", "Down"], "NED", base,
                            err_thresh=args.zoom_thresh, tail_seconds=args.zoom_tail)

    # ECEF (if present)
    p_e = data.get("pos_ecef")
    v_e = data.get("vel_ecef")
    pt_e = data.get("truth_pos_ecef")
    vt_e = data.get("truth_vel_ecef")
    if p_e is not None and v_e is not None and pt_e is not None and vt_e is not None and len(pt_e) and len(vt_e):
        base = out_dir / f"{run_id}_task7_6_overlay_ECEF_enhanced"
        _overlay_plot(t, p_e, pt_e, v_e, vt_e, ["X", "Y", "Z"], "ECEF", base)
        _zoom_on_divergence(t, p_e, pt_e, v_e, vt_e, ["X", "Y", "Z"], "ECEF", base,
                            err_thresh=args.zoom_thresh, tail_seconds=args.zoom_tail)

    # Body (if present)
    p_b = data.get("pos_body")
    v_b = data.get("vel_body")
    # For truth in body, prefer truth in NED rotated in pipeline and saved; if absent, skip
    pt_b = data.get("truth_pos_body")
    vt_b = data.get("truth_vel_body")
    if p_b is not None and v_b is not None and pt_b is not None and vt_b is not None:
        base = out_dir / f"{run_id}_task7_6_overlay_Body_enhanced"
        _overlay_plot(t, p_b, pt_b, v_b, vt_b, ["X", "Y", "Z"], "Body", base)
        _zoom_on_divergence(t, p_b, pt_b, v_b, vt_b, ["X", "Y", "Z"], "Body", base,
                            err_thresh=args.zoom_thresh, tail_seconds=args.zoom_tail)

    # --- Innovations & gating ---------------------------------------------
    innov_pos = data.get("innov_pos")
    innov_vel = data.get("innov_vel")
    t_upd = data.get("time_residuals")
    P_hist = data.get("P_hist")
    if innov_pos is not None and innov_vel is not None and t_upd is not None:
        base = out_dir / f"{run_id}_task7_6"
        _plot_innovations_with_gates(
            np.asarray(innov_pos),
            np.asarray(innov_vel),
            np.asarray(t_upd).squeeze(),
            np.asarray(P_hist) if P_hist is not None else None,
            float(args.r_pos),
            float(args.r_vel),
            base,
            gate_sigma=float(args.gate_sigma),
        )


if __name__ == "__main__":
    main()

