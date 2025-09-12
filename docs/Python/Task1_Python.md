# Python Pipeline – Task 1 Reference Vectors

This page mirrors **Task 1** of the original wiki but focuses on the Python implementation located in `src/GNSS_IMU_Fusion.py`.  The goal is to derive the reference vectors required by the TRIAD attitude initialisation.

## Overview

The script extracts the first valid latitude and longitude from the GNSS log and defines gravity and Earth‑rotation vectors in the local NED frame.  These vectors are later compared to the body‑frame measurements.

```text
GNSS ECEF → geodetic → latitude/longitude
    ↓
Reference vectors g_NED and ω_ie_NED
    ↓
Optional magnetic field if a magnetometer file is provided
```

## Subtasks

### 1.1 Read Initial GNSS Fix
- Load the GNSS CSV with `pandas.read_csv` and select the first row that contains all ECEF columns.
- Convert `X_ECEF_m`, `Y_ECEF_m`, `Z_ECEF_m` to latitude and longitude using `ecef_to_geodetic`.
- Rotate the ECEF velocity to NED with `compute_C_ECEF_to_NED(lat, lon)` and print the coordinates.

### 1.2 Initialise Gravity Vector
- Gravity in NED points down: `g_NED = [0, 0, g]` with `g ≈ 9.81 m/s²`.
- Store this vector for the comparison with the IMU measurements in Task 2.

### 1.3 Earth Rotation Rate
- Compute the NED Earth‑rate vector using the latitude from step 1.1.
- If a magnetometer file exists, use the `geomag` library to estimate the field and store it alongside `ω_ie_NED`.

### 1.4 Validate Reference Vectors
- Ensure each vector has shape `(3,)` and numeric values.
- Confirm gravity contains no North/East components and print the results to the console.

### 1.5 Plot Initial Location
- Create a small map centred on the derived position with `cartopy`.
- Mark the location with a red dot and label the latitude/longitude.
 - Save the plot as `results/<tag>_task1_location_map.[pdf|png]` unless the `--no-plots` flag is used.
- See the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) for consistent naming of plot entries.

## Running the Script


Earth‑rate vectors.  Use `--no-plots` to suppress map generation during batch
runs.

## Output Files

All artefacts are written to `PYTHON/results/`.  The location map provides a
visual check that the first GNSS fix was interpreted correctly and is stored as
`<tag>_task1_location_map.pdf` (and optionally `.png`).

## Result

Task 1 prints the reference vectors and optionally saves a location map.  The values feed directly into the body‑frame measurements of **Task 2** and ultimately the TRIAD attitude solution in **Task 3**.
