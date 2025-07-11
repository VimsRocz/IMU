# Results Overview

The table below summarises the statistics printed by `run_triad_only.py` for datasets **X001**, **X002** and **X003** using the TRIAD initialisation method.

| Dataset | RMSE Position [m] | Final Error [m] | Max Residual Pos [m] | Grav Err Mean [deg] | Earth Rate Max [deg] |
|---------|------------------|-----------------|----------------------|--------------------|---------------------|
| X001    | 0.28             | 0.01            | 0.29                 | 0.00               | 0.00                |
| X002    | 0.29             | 0.01            | 0.30                 | 0.04               | 0.23                |
| X003    | 0.29             | 0.01            | 0.30                 | 0.33               | 1.97                |

All datasets used the same number of ZUPT detections (479&nbsp;587). Processing
a single dataset took around three minutes on the provided hardware.

When a reference track (for example `STATE_X001.txt`) is available the
validation script writes additional `<method>_<frame>_overlay_truth.pdf`
figures to the `results/` folder. These plots overlay the fused trajectory with
the ground truth in the NED, ECEF and body frames so you can visually confirm
the filter performance.
