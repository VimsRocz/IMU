# IMU/GNSS Processing Workflow

This page provides a detailed overview of the full demonstration contained in this repository. It mirrors the steps implemented in `full_project_workflow.py` and references the scripts in `methods/` as well as the data files in the repository root.

The workflow is organized into five major tasks:

1. **Define Reference Vectors in NED Frame**
2. **Measure the Vectors in the Body Frame**
3. **Solve Wahba's Problem**
4. **GNSS and IMU Data Integration and Comparison**
5. **Sensor Fusion with Kalman Filter**

Each task is summarized below with its key subtasks so you can easily cross-reference the code.

## 1. Define Reference Vectors in NED Frame

This stage sets up the reference location and the fundamental vectors used throughout the workflow.

1. **Set Initial Latitude and Longitude from GNSS ECEF Data** – Load the first valid GNSS position from the provided CSV file and convert the ECEF coordinates to latitude, longitude, and altitude. This establishes the reference location for the entire dataset.
2. **Define Gravity Vector in NED** – Use `g = 9.81 m/s²` to define the gravity vector in North-East-Down (NED) coordinates: `[0, 0, g]`.
3. **Define Earth Rotation Rate Vector in NED** – Compute the Earth's rotation rate vector at the reference latitude using the constant `7.2921159e-5 rad/s`.
4. **Validate and Print Reference Vectors** – Ensure the vectors have the expected shape and print their values for verification.
5. **Plot Location on Earth Map** – Generate a PDF map with Cartopy showing the starting point. The plot is saved as `1_Initial_Location_Map.pdf`.

## 2. Measure the Vectors in the Body Frame

The second task extracts static measurements from the IMU in order to express the reference vectors in sensor coordinates.

1. **Load and Parse IMU Data** – Read the `IMU_X002.dat` file, parsing the columns for velocity and angular increments. Convert the increments to accelerations and angular rates using the sensor sampling interval.
2. **Estimate Static Body-Frame Vectors** – Average the first 4000 samples (10 s at 400 Hz) to estimate the static accelerometer and gyroscope outputs.
3. **Define Gravity and Earth Rate in Body Frame** – Express gravity (`g_body`) and the Earth rotation rate (`omega_ie_body`) in the body frame based on the static measurements.
4. **Validate and Print Body-Frame Vectors** – Check that the magnitudes are reasonable and print them. Expect roughly 9.81 m/s² for gravity and 7.29e-5 rad/s for Earth rotation.

## 3. Solve Wahba's Problem

This task determines the initial orientation of the body frame with respect to NED.

1. **Prepare Vector Pairs** – Normalize the body-frame and NED-frame vectors. Two cases are provided: the code's current implementation and the document equation version.
2. **TRIAD Method** – Compute the body-to-NED rotation matrix using the TRIAD algorithm for both cases.
3. **Davenport's Q-Method** – Apply Davenport's method to estimate the rotation, yielding both a quaternion and a rotation matrix.
4. **SVD Method** – Use the singular-value decomposition technique as an alternative attitude estimator.
5. **Convert DCMs to Quaternions** – Convert the rotation matrices from the TRIAD and SVD solutions into quaternion form.
6. **Validate and Compare Methods** – Compute angular errors between the estimated and reference vectors, logging the differences for each method and case.
7. **Plot Validation Errors and Quaternion Components** – Create PDF plots comparing errors and quaternion components for all methods. The files are `3_Wahba_Error_Comparison.pdf` and `3_Wahba_Quaternion_Comparison.pdf`.
8. **Store Rotation Matrices** – Save the rotation matrices for use in the following tasks.

## 4. GNSS and IMU Data Integration and Comparison

With the initial attitude solved, the next stage integrates the IMU data and compares it to GNSS measurements.

1. **Access Rotation Matrices from Task 3** – Load the matrices computed earlier for TRIAD, Davenport, and SVD.
2. **Compute ECEF↔NED Rotations** – Build helper functions for converting between ECEF and NED frames.
3. **Load GNSS Data** – Read GNSS positions and velocities from `GNSS_X001.csv`.
4. **Extract Relevant Columns** – Retrieve time, position, and velocity in ECEF coordinates.
5. **Define Reference Point** – Use a fixed origin in ECEF/NED for differencing, matching the values found in Task 1.
6. **Compute Rotation Matrix** – Determine the ECEF-to-NED matrix for the reference location.
7. **Convert GNSS Data to NED** – Transform GNSS positions and velocities into the local frame.
8. **Estimate GNSS Acceleration** – Differentiate velocity to approximate acceleration in NED.
9. **Load IMU Data and Correct for Bias** – Apply bias corrections to the accelerometer and gyroscope measurements for each method.
10. **Set IMU Parameters and Gravity Vector** – Specify constants used during integration, including the gravity vector.
11. **Initialize Output Arrays** – Allocate arrays for the integrated position, velocity, and acceleration for each method.
12. **Integrate IMU Accelerations** – Propagate motion using the body-to-NED rotation for each method.
13. **Validate and Plot Data** – Generate multiple PDF comparisons of GNSS vs IMU data in NED, ECEF, and body frames. Examples include `4_GNSS_IMU_Comparison_NED.pdf` and `4_All_Data_ECEF.pdf`.

## 5. Sensor Fusion with Kalman Filter

The final task fuses the GNSS measurements with the integrated IMU data using a Kalman filter.

1. **Configure Logging** – Set up consistent logging for the filter stage.
2. **Rotation Matrix – ECEF to NED** – Provide a helper function for frame conversion (same as in Task 4).
3. **Load GNSS and IMU Data** – Load the measurement files again for filtering.
4. **Integrate IMU Data for Each Method** – Propagate position and velocity using the corrected IMU data, starting from the GNSS-derived initial state.
5. **Apply Zero-Velocity Updates (ZUPT)** – Zero the velocity when the platform is stationary to limit drift.
6. **Kalman Filter for Sensor Fusion** – Fuse GNSS measurements with IMU integration results to produce smoothed trajectories.
7. **Handle Event at 5000 s** – The demo includes a placeholder for event handling, which is unused for this dataset.
8. **Plot Fusion Results** – Create PDF plots showing the fused position, velocity, and acceleration for each attitude method. Example filenames: `5_Fusion_Results_TRIAD.pdf` and `5_Fusion_Results_SVD.pdf`.
9. **Evaluate Filter Results – Residuals & Attitude** – Compute residual errors relative to GNSS, plot residuals, and derive roll, pitch, and yaw over time. Additional PDFs such as `5_Filter_Residuals_TRIAID.pdf` and `5_Attitude_TRIAD.pdf` are generated.

## Naming the Wiki Page

A descriptive name such as **`IMU_GNSS_Workflow`** or **`Project_Workflow`** is recommended. The file created in this repository is `wiki/IMU_GNSS_Workflow.md`.

