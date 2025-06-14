# Tasks Overview

This project demonstrates a multi-step IMU/GNSS processing workflow.  The example scripts implement five main tasks, each with several subtasks.  The summaries below describe the intent of each step so that the overall flow is easy to follow.

## Task 1 – Define Reference Vectors in NED
1. **Set Initial Latitude and Longitude from GNSS ECEF Data** – Convert the first valid GNSS position from ECEF to geodetic coordinates to establish the reference location.
2. **Define Gravity Vector in NED** – Use `g = 9.81 m/s²` to set the gravity vector `[0, 0, g]` in NED coordinates.
3. **Define Earth Rotation Rate Vector in NED** – Compute the Earth rotation vector at the reference latitude.
4. **Validate and Print Reference Vectors** – Ensure the vectors are well-formed and log their values.
5. **Plot Location on Earth Map** – Create a PDF map showing the reference location.

## Task 2 – Measure the Vectors in the Body Frame
1. **Load and Parse IMU Data** – Read the raw IMU data file containing velocity and angular increments.
2. **Estimate Static Body‑Frame Vectors** – Average the first seconds of data to estimate static accelerometer and gyroscope outputs.
3. **Define Gravity and Earth Rate in Body Frame** – Derive the gravity and rotation vectors expressed in sensor axes.
4. **Validate and Print Body‑Frame Vectors** – Check magnitudes and print the measured vectors.

## Task 3 – Solve Wahba’s Problem
1. **Prepare Vector Pairs** – Normalize body and reference vectors for attitude determination.
2. **TRIAD Method** – Compute the body‑to‑NED rotation matrix using the TRIAD algorithm.
3. **Davenport’s Q‑Method** – Estimate the same rotation with Davenport’s approach.
4. **SVD Method** – Use the singular‑value decomposition technique for attitude determination.
5. **Convert to Quaternions** – Convert rotation matrices from the TRIAD and SVD solutions into quaternion form.
6. **Validate and Compare Methods** – Compare each method by checking how well they align the reference vectors.
7. **Plot Errors and Quaternions** – Export PDFs summarizing the orientation errors and quaternion components.
8. **Store Rotation Matrices** – Save the results for the following tasks.

## Task 4 – GNSS and IMU Data Integration and Comparison
1. **Access Rotation Matrices from Task 3** – Reuse the previously computed orientations for transforming IMU data.
2. **Compute ECEF↔NED Rotations** – Build utility matrices for converting between frames.
3. **Load GNSS Data** – Read positions and velocities from the GNSS CSV file.
4. **Extract Relevant Columns** – Pull out time, position, and velocity measurements in ECEF.
5. **Define Reference Point** – Select a fixed origin in ECEF/NED for differencing.
6. **Compute Rotation Matrix** – Calculate the ECEF‑to‑NED matrix at the reference location.
7. **Convert GNSS Data to NED** – Transform GNSS positions and velocities into the local frame.
8. **Estimate GNSS Acceleration** – Differentiate the velocity to approximate acceleration in NED.
9. **Load IMU Data and Correct for Bias** – Remove accelerometer and gyroscope bias for each attitude method.
10. **Set IMU Parameters and Gravity Vector** – Define constants used during integration.
11. **Initialize Output Arrays** – Allocate arrays for the integrated position, velocity, and acceleration.
12. **Integrate IMU Accelerations** – Propagate motion using the body‑to‑NED rotation for each method.
13. **Validate and Plot Data** – Produce several comparison plots in PDF format.

## Task 5 – Sensor Fusion with Kalman Filter
1. **Configure Logging** – Set up logging for consistency.
2. **Rotation Matrix – ECEF to NED** – Provide a helper function for frame conversion.
3. **Load GNSS and IMU Data** – Read the measurements used for filtering.
4. **Integrate IMU Data for Each Method** – Propagate position and velocity using the corrected IMU data.
5. **Apply Zero‑Velocity Updates (ZUPT)** – Zero the velocity when the platform is stationary.
6. **Kalman Filter for Sensor Fusion** – Fuse GNSS measurements with IMU integration results.
7. **Handle Event at 5000 s** – Placeholder for any event‑based handling (none used here).
8. **Plot Fusion Results** – Export PDFs of fused trajectories for each attitude method.
9. **Evaluate Filter Results – Residuals & Attitude** – Compute residual errors, derive attitude angles over time, and create additional plots.

---

These tasks correspond to the large demonstration script shown in the repository.  Each standalone script in the `methods/` folder focuses on a specific orientation method, while `methods/combined` runs all three methods for quick comparison.

