#!/usr/bin/env python
"""
Compare the three data sets (X001,X002,X003) on the same axes
for each attitude-initialisation method (TRIAD | Davenport | SVD).
"""

import pathlib
import gzip
import pickle
import matplotlib.pyplot as plt
import numpy as np
import os

os.makedirs('results', exist_ok=True)
print("Ensured 'results/' directory exists.")
RESULTS_DIR = pathlib.Path("results")
COLOURS     = {"X001": "tab:blue", "X002": "tab:orange", "X003": "tab:green"}
LABEL_MAP   = {"N": "North", "E": "East", "D": "Down"}

# ---------- 1. Load everything ------------------------------------------------
blobs = []
for f in RESULTS_DIR.glob("*_compare.pkl.gz"):
    with gzip.open(f, "rb") as fh:
        blobs.append(pickle.load(fh))

if not blobs:
    raise RuntimeError("No *_compare.pkl.gz files found – did you run step ①-②?")

# ---------- 2. Group by method ------------------------------------------------
by_method = {}
for b in blobs:
    by_method.setdefault(b["method"], []).append(b)

# ---------- 3. Helper to plot one figure -------------------------------------
def plot_one(method, packs):
    """Plot position, velocity and acceleration for all datasets."""
    fig, axes = plt.subplots(3, 3, figsize=(14, 12), sharex="col")
    axes = axes.reshape(3, 3)

    for pack in packs:
        ds  = pack["dataset"]
        col = COLOURS[ds]
        t   = pack["t"]

        # Position and velocity
        axes[0,0].plot(t, pack["pos_ned"][:,0], color=col)
        axes[0,1].plot(t, pack["pos_ned"][:,1], color=col)
        axes[0,2].plot(t, pack["pos_ned"][:,2], color=col, label=f"{ds}-Fused")
        axes[1,0].plot(t, pack["vel_ned"][:,0], color=col)
        axes[1,1].plot(t, pack["vel_ned"][:,1], color=col)
        axes[1,2].plot(t, pack["vel_ned"][:,2], color=col)

        # Derive acceleration from fused velocity
        acc_fused = np.gradient(pack["vel_ned"], t, axis=0)
        axes[2,0].plot(t, acc_fused[:,0], color=col)
        axes[2,1].plot(t, acc_fused[:,1], color=col)
        axes[2,2].plot(t, acc_fused[:,2], color=col)

        gnss_t = np.linspace(t[0], t[-1], pack["pos_gnss"].shape[0])
        axes[0,0].plot(gnss_t, pack["pos_gnss"][:,0], ls="--", color=col, alpha=.6)
        axes[0,1].plot(gnss_t, pack["pos_gnss"][:,1], ls="--", color=col, alpha=.6)
        axes[0,2].plot(gnss_t, pack["pos_gnss"][:,2], ls="--", color=col, alpha=.6,
                       label=f"{ds}-GNSS")
        axes[1,0].plot(gnss_t, pack["vel_gnss"][:,0], ls="--", color=col, alpha=.6)
        axes[1,1].plot(gnss_t, pack["vel_gnss"][:,1], ls="--", color=col, alpha=.6)
        axes[1,2].plot(gnss_t, pack["vel_gnss"][:,2], ls="--", color=col, alpha=.6)
        # GNSS acceleration from velocity
        vel_gnss = pack["vel_gnss"]
        acc_gnss = np.gradient(vel_gnss, gnss_t, axis=0)
        axes[2,0].plot(gnss_t, acc_gnss[:,0], ls="--", color=col, alpha=.6)
        axes[2,1].plot(gnss_t, acc_gnss[:,1], ls="--", color=col, alpha=.6)
        axes[2,2].plot(gnss_t, acc_gnss[:,2], ls="--", color=col, alpha=.6)

    for j, lab in enumerate(["N", "E", "D"]):
        axes[0, j].set_title(f"Position {LABEL_MAP[lab]}")
        axes[1, j].set_title(f"Velocity {LABEL_MAP[lab]}")
        axes[2, j].set_title(f"Acceleration {LABEL_MAP[lab]}")
        axes[2, j].set_xlabel("Time (s)")
        axes[0, j].set_ylabel("m")
        axes[1, j].set_ylabel("m/s")
        axes[2, j].set_ylabel("m/s$^2$")

    axes[0,2].legend(loc="upper center", bbox_to_anchor=(0.5, -0.15),
                     ncol=3, frameon=False)
    fig.suptitle(f"All datasets – Method: {method}", fontsize=16)
    fig.tight_layout()
    out = RESULTS_DIR / f"all_datasets_{method}_comparison.pdf"
    fig.savefig(out)
    print(f"  ➜ wrote {out}")

# ---------- 4. Generate the figures ------------------------------------------
for meth, packs in by_method.items():
    plot_one(meth, packs)

