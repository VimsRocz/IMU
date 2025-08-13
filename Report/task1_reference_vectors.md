# Reference Vectors

This page summarises **Task\u00a01** as executed by `run_triad_only.py`. It derives the navigation-frame reference vectors from the GNSS log.

## Data file format

`GNSS_Xnnn.csv` contains twenty comma-separated columns. The header lists the
timestamp, geodetic position, ECEF coordinates and GNSS quality metrics. The
table below spells out the column names and units for clarity.

| Column          | Unit  | Description                            |
|-----------------|------|----------------------------------------|
| `UTC_yyyy`      | year | Calendar year (UTC)                    |
| `UTC_MM`        | month| Calendar month (UTC)                   |
| `UTC_dd`        | day  | Day of month (UTC)                     |
| `UTC_HH`        | hour | Hour of day (UTC)                      |
| `UTC_mm`        | min  | Minute of hour (UTC)                   |
| `UTC_ss`        | s    | Second of minute (UTC)                 |
| `Posix_Time`    | s    | Continuous Unix time                   |
| `Latitude_deg`  | deg  | Geodetic latitude                      |
| `Longitude_deg` | deg  | Geodetic longitude                     |
| `Height_deg`    | m    | Altitude above the ellipsoid           |
| `X_ECEF_m`      | m    | ECEF X position                        |
| `Y_ECEF_m`      | m    | ECEF Y position                        |
| `Z_ECEF_m`      | m    | ECEF Z position                        |
| `VX_ECEF_mps`   | m/s  | ECEF X velocity                        |
| `VY_ECEF_mps`   | m/s  | ECEF Y velocity                        |
| `VZ_ECEF_mps`   | m/s  | ECEF Z velocity                        |
| `HDOP`          | –    | Horizontal dilution of precision       |
| `VDOP`          | –    | Vertical dilution of precision         |
| `PDOP`          | –    | Position dilution of precision         |
| `TDOP`          | –    | Time dilution of precision             |

The `UTC_ss` column resets every minute, whereas `Posix_Time` increases
monotonically. Each sample log contains 1\u202f250 rows plus the header.

## Steps

1. Load the GNSS CSV into memory and keep it available for later tasks. From
   this dataset read the first valid ECEF position and velocity.
2. Convert the position to geodetic latitude $\varphi$ and longitude $\lambda$ (see [ECEF_to_Geodetic.md](../docs/ECEF_to_Geodetic.md)).
3. Rotate the velocity to NED with `compute_C_ECEF_to_NED(\varphi, \lambda)`.
4. Define

   $$
   \mathbf{g}_{\text{NED}} = \begin{bmatrix}0 \\ 0 \\ g\end{bmatrix},\qquad
   \boldsymbol{\omega}_{ie,\text{NED}} = \omega_E\begin{bmatrix}\cos\varphi \\ 0 \\ -\sin\varphi\end{bmatrix},
   $$
   with $g \approx 9.81\,\mathrm{m/s^2}$ and $\omega_E \approx 7.2921\times10^{-5}\,\mathrm{rad/s}$.
5. Save a small location map as `results/run_triad_only/<tag>_task1_location_map.png`.
