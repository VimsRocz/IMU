# MATLAB Pipeline – Task 1 Reference Vectors

This page describes **Task 1** of the MATLAB pipeline found in `MATLAB/Task_1.m`.  The task prepares the reference vectors required for TRIAD initialisation.

## Overview

The function reads the first GNSS fix, converts it to latitude and longitude and constructs gravity and Earth‑rotation vectors in the local NED frame.

```text
GNSS ECEF → geodetic → latitude/longitude
    ↓
Reference vectors g_NED and ω_ie_NED
    ↓
Optional magnetic field when a magnetometer log is present
```

## Subtasks

### 1.1 Read Initial GNSS Fix
- Use `readtable` to load the GNSS CSV and extract the first valid ECEF coordinates.
- Call `ecef2lla` (or `ecef2geodetic` depending on MATLAB version) to obtain latitude and longitude.
- Rotate the ECEF velocity to NED with `compute_C_ECEF_to_NED` and display the coordinates.

### 1.2 Initialise Gravity Vector
- Define `g_NED = [0; 0; 9.81]` representing gravity pointing down.
- Store the vector for use in Task 2.

### 1.3 Earth Rotation Rate
- Compute `ω_ie_NED` from the latitude using the constant `ω_E` defined in `constants.m`.
- If a magnetometer log is provided, compute the local magnetic field and store it for TRIAD.

### 1.4 Validate Reference Vectors
- Ensure the vectors are 3×1 numeric arrays.
- Verify that gravity has no horizontal component and print the values to the console.

### 1.5 Plot Initial Location
- Create a small map with `geoplot` or `geoscatter` showing the derived position.
- Annotate the latitude and longitude on the plot.
- Save the figure as `results/<tag>_location_map.pdf` unless disabled.

## Result

Task 1 returns the reference vectors and optionally a location plot.  Subsequent tasks use these values when measuring body vectors and solving the initial attitude.
