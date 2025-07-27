# Converting ECEF to Geodetic Coordinates

This note describes the algorithm used throughout the repository to convert
Earth‑Centred Earth‑Fixed (ECEF) coordinates to geodetic latitude,
longitude and altitude. The implementation is shared between the Python
code (`src/utils.py`) and the MATLAB helpers `ecef_to_geodetic.m` and
`ecef2geodetic.m`.

`ecef_to_geodetic.m` automatically delegates to MATLAB's Mapping
Toolbox function `ecef2geodetic` when available and otherwise executes
the lightweight `ecef2geodetic.m` implementation bundled with this
repository. Both variants yield identical results.

## Algorithm

The conversion assumes the WGS‑84 ellipsoid with
semi‑major axis `a = 6378137.0 m` and first eccentricity squared
`e² = 6.69437999014e‑3`.

1. Compute the distance from the Z‑axis
   `p = sqrt(x² + y²)`.
2. Compute an auxiliary angle
   `θ = atan2(z * a, p * (1 − e²))`.
3. Longitude in radians is simply `atan2(y, x)`.
4. Latitude in radians is
   ```
   atan2(z + e² * a * sin(θ)³ / (1 − e²),
         p − e² * a * cos(θ)³)
   ```
5. The prime vertical radius of curvature is
   `N = a / sqrt(1 − e² * sin(lat)²)`.
6. Altitude is `p / cos(lat) − N`.
7. Convert latitude and longitude to degrees for output.

Both implementations follow these steps exactly so the results match to
machine precision.

