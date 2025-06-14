# Plot Summary

This project outputs a set of PDF figures illustrating the IMU/GNSS processing
workflow. The files described below are created by `full_project_workflow.py`
and help visualize the starting location and the final sensor fusion results.

- **1_Initial_Location_Map.pdf** – Displays a small world map centered on the
  first GNSS fix. It provides geographic context for the dataset by marking the
  initial latitude and longitude.
- **5_Fusion_Positions_Compare.pdf** – Shows the Kalman filter position
  solutions for clean and noisy datasets on the same axes. This helps judge how
  measurement noise affects the fused North, East, and Down positions over time.
- **5_Fusion_Residuals_Compare.pdf** – Plots the position residuals from the
  filter for both datasets. The residual curves reveal how well the fusion
  tracks the GNSS measurements and highlights periods of larger errors.

