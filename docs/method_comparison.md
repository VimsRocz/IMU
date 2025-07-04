# Method Comparison

The table below summarises the fusion results produced by `run_all_datasets.py`.
Values are extracted from `results/summary.csv` via `summarise_runs.py`.
The best method per dataset is marked with a check mark.

| Dataset | Method | RMSE Position [m] | Final Error [m] | Best |
|---------|--------|------------------|-----------------|------|
| IMU_X001_small | Davenport | 0.00 | 0.00 | ✓ |
| IMU_X001_small | SVD | 0.00 | 0.00 |  |
| IMU_X001_small | TRIAD | 0.00 | 0.00 |  |
| IMU_X002_small | Davenport | 0.04 | 0.05 |  |
| IMU_X002_small | SVD | 0.04 | 0.05 |  |
| IMU_X002_small | TRIAD | 0.04 | 0.05 | ✓ |
| IMU_X003_small | Davenport | 0.04 | 0.05 |  |
| IMU_X003_small | SVD | 0.04 | 0.05 | ✓ |
| IMU_X003_small | TRIAD | 0.04 | 0.05 |  |
