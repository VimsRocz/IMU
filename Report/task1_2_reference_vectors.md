# Reference and Body Vectors

This page summarises **Task&nbsp;1** and **Task&nbsp;2** as executed by
`run_triad_only.py`. Task&nbsp;1 derives the reference vectors in the navigation
frame from the GNSS log. Task&nbsp;2 measures the same vectors in the body frame
using a static portion of the IMU data.

## Task 1 – Reference Vectors

### Data file format

`GNSS_Xnnn.csv` contains at least the columns

```
X_ECEF_m, Y_ECEF_m, Z_ECEF_m,
VX_ECEF_mps, VY_ECEF_mps, VZ_ECEF_mps
```

### Steps

1. Load the first valid ECEF position and velocity from the GNSS file.
2. Convert the position to geodetic latitude $\varphi$ and longitude
   $\lambda$ (see
   [ECEF_to_Geodetic.md](../docs/ECEF_to_Geodetic.md)).
3. Rotate the velocity to NED with `compute_C_ECEF_to_NED(\varphi, \lambda)`.
4. Define

   $$
   \mathbf{g}_{\text{NED}} = \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix},\qquad
   \boldsymbol{\omega}_{ie,\text{NED}} =
       \omega_E\begin{bmatrix}\cos\varphi \\ 0 \\ -\sin\varphi\end{bmatrix},
   $$
   with $g \approx 9.81\,\mathrm{m/s^2}$ and
   $\omega_E \approx 7.2921159\times10^{-5}\,\mathrm{rad/s}$.
5. Save a small location map as `results/<tag>_location_map.pdf`.

## Task 2 – Body‑Frame Measurements

### Data file format

`IMU_Xnnn.dat` is a whitespace separated table with a time column,
angular increments $\Delta\theta_x\,\Delta\theta_y\,\Delta\theta_z$ and
velocity increments $\Delta v_x\,\Delta v_y\,\Delta v_z$.

### Steps

1. Load the IMU file and extract the increment columns.
2. Estimate the sampling period from the time values.
3. Convert increments to rates and apply a low‑pass filter.
4. Detect a static interval with `detect_static_interval` and compute the mean
   accelerometer and gyroscope vectors.
5. Scale the accelerometer mean so its norm equals $9.81\,\mathrm{m/s^2}$.
6. Form

   $$
   \mathbf{g}_b = -\text{acc\_mean},\qquad
   \boldsymbol{\omega}_{ie,b} = \text{gyro\_mean}.
   $$
7. Record the window indices, sampling period and magnitudes in
   `triad_init_log.txt`.
8. Save the ZUPT diagnostic plot as `results/<tag>_ZUPT_variance.pdf`.

These body‑frame vectors correspond to the Task&nbsp;1 references expressed in
sensor axes and are used to determine the initial attitude in Task&nbsp;3.
