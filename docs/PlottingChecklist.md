# Plotting Checklist

The following codex-style guidelines summarize how to present and document plots for this project. Use them as a reference when generating new figures or updating existing scripts.

## 1. Update Plot Scripts for Better Presentation
- Export all plots as **PDF** files using `plt.savefig("results/<meaningful_filename>.pdf")`.
- Filenames should include dataset, method, and content (e.g. `IMU_X001_GNSS_X001_TRIAD_task4_all_ned.pdf`).
- Each plot must have:
  - A descriptive title.
  - Axis labels with units (e.g. `Time [s]`, `Position North [m]`).
  - A legend if multiple data series are shown.
  - Consistent colors across figures (North: blue, East: green, Down: red).
- Annotate key results directly on the plot (e.g. RMSE or max error).

## 2. Create a Per-Plot Summary Table
Include a table describing each plot with columns:

| No. | Filename | Description & What’s Shown | Main Result/Insight | Improvement Needed |
|-----|----------|----------------------------|--------------------|--------------------|

Use one row per plot. See `plot_summary.md` for an example.

## 3. Optional Per-Plot Text Summaries
Store short summaries for each figure in `results/plot_summary.md` or similar:
```
# Plot: attitude_angles_IMU_X001_GNSS_X001_TRIAD_TRIAD.pdf

**Description:** Roll, pitch, and yaw over time after TRIAD initialization.

**Insight:** Attitude is stable; confirms initialization.

**Improvement:** Add time markers and clearer labels.
```

## 4. Filter Results – Residuals & Innovations
- Plot position and velocity residuals between the filter prediction and GNSS.
- Include innovations with ±3σ bounds in subplots.
- Provide RMSE and extrema either on the plot or in the summary table.

## 5. Documentation & Reproducibility
- Document dataset-specific parameters and any script flags in the README.
- All datasets live in `Data/` and the updated plotting scripts write PDFs to
  `results/`.
- If `filterpy` fails to install on Ubuntu:
```bash
sudo apt-get install python3-pip
pip3 install filterpy
pip3 install numpy scipy matplotlib
```

## ✅ Checklist Before Submission
- All plots are PDFs with descriptive filenames.
- Titles, axis labels, units, and legends are present.
- Key stats (RMSE, max error) are annotated or noted in captions.
- Consistent color scheme across figures.
- `plot_summary.md` or per-plot summaries are updated.
- Installation notes for `filterpy` are included in the README.
- All analysis focuses on the TRIAD initialization method.
