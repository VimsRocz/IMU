## Changelog

### Recent Updates

- **Bias estimation fix** (`Task_2` function, `fusion_single.py`)
  - Detects a low-motion segment to compute accelerometer and gyroscope biases.
  - Scales the accelerometer magnitude to match 9.81 m/s² for more stable attitude initialisation.

- **Kalman filter with GNSS updates** (`kalman.py`, `src/GNSS_IMU_Fusion.py`)
  - Adds a bias-aware Kalman filter for fusing IMU data with GNSS measurements.
  - Provides helper functions to run the filter and to tune process noise.
- **Tunable noise parameters** (`kalman.py`, `fusion_single.py`)
  - Position/velocity process and measurement noise can now be set via
    arguments for easier experimentation.

- **Plotting helpers** (`auto_plots.py`, `summarise_runs.py`, `generate_summary.py`)
  - Automates generation of standard figures and summary tables.
  - Useful for batch processing of multiple datasets and visualising results.

These utilities were added to streamline the fusion pipeline and assist with
debugging and analysis.
