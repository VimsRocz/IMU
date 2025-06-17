# TRIAD Method – Task 1 Reference Vectors

This page documents **Task&nbsp;1** of the `GNSS_IMU_Fusion.py` pipeline, which prepares the reference vectors required by the TRIAD attitude initialisation. The task consists of five subtasks executed before any IMU or GNSS data fusion.

## Overview

Task&nbsp;1 extracts the initial latitude and longitude from the GNSS data and defines the gravity and Earth‑rotation vectors in the local NED frame. These vectors serve as references for the TRIAD algorithm when estimating the initial orientation of the IMU.

```
GNSS ECEF → geodetic → latitude/longitude
↓
Reference vectors g_NED and ω_ie_NED
↓
Optional magnetic field if a magnetometer file is provided
```

## Subtasks

### 1.1 Set Initial Latitude and Longitude
- Load the GNSS CSV file and read the first valid ECEF coordinates (`X_ECEF_m`, `Y_ECEF_m`, `Z_ECEF_m`).
- Convert these coordinates to geodetic latitude and longitude using the helper function `ecef_to_geodetic`.
- Compute the initial GNSS velocity in NED by rotating the ECEF velocity with `compute_C_ECEF_to_NED(lat, lon)`.
- Print the latitude and longitude to confirm the location.

### 1.2 Define Gravity Vector
- In the NED frame gravity points **down**: `g_NED = [0, 0, g]` with `g ≈ 9.81 m/s²`.
- This vector is stored for later comparison with the body‑frame accelerometer data.

### 1.3 Define Earth Rotation Rate Vector
- The Earth rotation rate in NED is
  $$\omega_{ie,NED} = \omega_E \begin{bmatrix}\cos\varphi \\ 0 \\ -\sin\varphi\end{bmatrix},$$
  where `ω_E` is the Earth rate (`7.2921159e‑5 rad/s`) and `φ` is latitude.
- `omega_ie_NED` is used together with the gravity vector in the TRIAD solution.
- If a magnetometer file is supplied, the magnetic field in NED is also computed with `geomag`.

### 1.4 Validate Vectors
- Ensure all reference vectors are three‑component arrays.
- Check that gravity has no North/East component and Earth rate has no East component.
- Log the values and print them to the console for inspection.

### 1.5 Plot Initial Location
- Produce a small map centred on the derived latitude/longitude using the `PlateCarree` projection from `cartopy`.
- The position is marked with a red dot and labelled with latitude and longitude.
- The figure is saved as `results/<tag>_location_map.pdf` (unless `--no-plots` is used).

## Result

After completing Task 1 the script prints the reference vectors and saves the optional location map. Subsequent tasks (measurement in the body frame and the TRIAD solution) rely on these vectors to initialise the attitude.

