# Method Comparison

The table below summarises the fusion results produced by `run_all_datasets.py`.
Values are extracted from `results/run_all_datasets/summary.csv` via `summarise_runs.py`.
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

## Full dataset results

The following table lists the metrics printed by `run_all_methods.py` when
processing the complete IMU and GNSS logs with all three initialisation
methods. Each run corresponds to the log excerpt shown above.

| Dataset | Method | RMSE Position [m] | Final Error [m] | RMS Residual Pos [m] | Max Residual Pos [m] | RMS Residual Vel [m/s] | Max Residual Vel [m/s] |
|---------|--------|------------------|-----------------|----------------------|----------------------|-------------------------|-------------------------|
| IMU_X001 | TRIAD | 0.28 | 0.00 | 0.02 | 0.29 | 0.01 | 0.13 |
| IMU_X001 | SVD | 0.28 | 0.00 | 0.02 | 0.29 | 0.01 | 0.13 |
| IMU_X001 | Davenport | 0.28 | 0.00 | 0.02 | 0.29 | 0.01 | 0.13 |
| IMU_X002 | TRIAD | 0.29 | 0.00 | 0.03 | 0.30 | 0.10 | 0.42 |
| IMU_X002 | SVD | 0.29 | 0.00 | 0.03 | 0.30 | 0.10 | 0.42 |
| IMU_X002 | Davenport | 0.29 | 0.00 | 0.03 | 0.30 | 0.10 | 0.42 |
| IMU_X003 | TRIAD | 0.29 | 0.00 | 0.03 | 0.30 | 0.10 | 0.42 |
| IMU_X003 | SVD | 0.29 | 0.00 | 0.03 | 0.30 | 0.10 | 0.42 |
| IMU_X003 | Davenport | 0.29 | 0.00 | 0.03 | 0.30 | 0.10 | 0.42 |
