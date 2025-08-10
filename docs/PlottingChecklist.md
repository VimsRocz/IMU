# Plotting Checklist

The following codex-style guidelines summarize how to present and document plots for this project. Use them as a reference when generating new figures or updating existing scripts.

## 1. Update Plot Scripts for Better Presentation
- Export all plots as **PNG** files and save MATLAB `.fig` versions when applicable, e.g. `plt.savefig("results/<meaningful_filename>.png")`.
- Filenames should include dataset, method, and content (e.g. `IMU_X001_GNSS_X001_TRIAD_task4_all_ned.png`).
- Each plot must have:
  - A descriptive title.
  - Axis labels with units (e.g. `Time [s]`, `Position North [m]`).
  - Subtract the first timestamp so the time axis starts at zero. This avoids
    huge offsets like `1e9` on the x‑axis.
  - A legend if multiple data series are shown.
  - Consistent colors across figures (North: blue, East: green, Down: red).
- Annotate key results directly on the plot (e.g. RMSE or max error).

## Standardized Legend Terms
Use the same wording across all plots so readers immediately understand what
each line represents.

- **Measured GNSS** – raw position or velocity from the GNSS log.
- **GNSS Derived** – values calculated from the GNSS data such as differentiated
  acceleration.
- **Derived IMU** – trajectory obtained by integrating the corrected IMU
  measurements.  When comparing different initialisation methods append the
  method name in parentheses, for example `Derived IMU (TRIAD)`.
- **Fused** – Kalman filter output that combines GNSS and IMU, labelled as
  `Fused <METHOD>`.

## 2. Create a Per-Plot Summary Table
Include a table describing each plot with columns:

| No. | Filename | Description & What’s Shown | Main Result/Insight | Improvement Needed |
|-----|----------|----------------------------|--------------------|--------------------|

Use one row per plot. See `plot_summary.md` for an example.

## 3. Optional Per-Plot Text Summaries
Store short summaries for each figure in `results/plot_summary.md` or similar:
```
# Plot: task5_results_IMU_X001_GNSS_X001_TRIAD.png

**Description:** Position, velocity, and acceleration after TRIAD initialization.

**Insight:** Fused trajectory remains stable.

**Improvement:** Add time markers and clearer labels.
```

## 4. Documentation & Reproducibility
- Document dataset-specific parameters and any script flags in the README.
- If `filterpy` fails to install on Ubuntu:
```bash
sudo apt-get install python3-pip
pip3 install filterpy
pip3 install numpy scipy matplotlib
```

## ✅ Checklist Before Submission
- All plots are PNGs with descriptive filenames (and `.fig` where appropriate).
- Titles, axis labels, units, and legends are present.
- Key stats (RMSE, max error) are annotated or noted in captions.
- Consistent color scheme across figures.
- `plot_summary.md` or per-plot summaries are updated.
- Installation notes for `filterpy` are included in the README.
- All analysis focuses on the TRIAD initialization method.
