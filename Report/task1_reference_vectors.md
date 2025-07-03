# Reference Vectors

This page summarises **Task\u00a01** as executed by `run_triad_only.py`. It derives the navigation-frame reference vectors from the GNSS log.

## Data file format

`GNSS_Xnnn.csv` contains twenty comma-separated columns. The header lists the timestamp, geodetic position, ECEF coordinates and GNSS quality metrics:

```text
UTC_yyyy, UTC_MM, UTC_dd, UTC_HH, UTC_mm, UTC_ss,
Posix_Time,
Latitude_deg, Longitude_deg, Height_deg,
X_ECEF_m, Y_ECEF_m, Z_ECEF_m,
VX_ECEF_mps, VY_ECEF_mps, VZ_ECEF_mps,
HDOP, VDOP, PDOP, TDOP
```

* **UTC_yyyy \u2013 UTC_ss** \u2013 calendar time components.
* **Posix_Time** \u2013 continuous Unix time in seconds.
* **Latitude/Longitude** \u2013 geodetic coordinates in degrees.
* **Height** \u2013 altitude above the ellipsoid in metres (despite the `deg` suffix).
* **X/Y/Z_ECEF_m** \u2013 ECEF position in metres.
* **VX/VY/VZ_ECEF_mps** \u2013 ECEF velocity in metres per second.
* **HDOP/VDOP/PDOP/TDOP** \u2013 dimensionless GNSS DOP values.

The `UTC_ss` column resets every minute, whereas `Posix_Time` increases monotonically. Each sample log contains 1\u202f250 rows plus the header.

## Steps

1. Load the first valid ECEF position and velocity from the GNSS file.
2. Convert the position to geodetic latitude $\varphi$ and longitude $\lambda$ (see [ECEF_to_Geodetic.md](../docs/ECEF_to_Geodetic.md)).
3. Rotate the velocity to NED with `compute_C_ECEF_to_NED(\varphi, \lambda)`.
4. Define

   $$
   \mathbf{g}_{\text{NED}} = \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix},\qquad
   \boldsymbol{\omega}_{ie,\text{NED}} = \omega_E\begin{bmatrix}\cos\varphi \\ 0 \\ -\sin\varphi\end{bmatrix},
   $$
   with $g \approx 9.81\,\mathrm{m/s^2}$ and $\omega_E \approx 7.2921159\times10^{-5}\,\mathrm{rad/s}$.
5. Save a small location map as `results/<tag>_location_map.pdf`.
