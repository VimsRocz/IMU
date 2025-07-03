# Body-Frame Measurements

This page summarises **Task\u00a02**, which measures the reference vectors in the sensor body frame using a static portion of the IMU data.

## Data file format

`IMU_Xnnn.dat` is a whitespace-separated table with ten columns.  The column
names and units are listed below.

| Column          | Unit | Description                                     |
|-----------------|-----|-------------------------------------------------|
| `index`         | –   | Sample counter starting at 0                    |
| `time_s`        | s   | Time stamp (increments of `0.0025` s)           |
| `dtheta_x`      | rad | Angular increment about X                       |
| `dtheta_y`      | rad | Angular increment about Y                       |
| `dtheta_z`      | rad | Angular increment about Z                       |
| `dv_x`          | m/s | Velocity increment along X                      |
| `dv_y`          | m/s | Velocity increment along Y                      |
| `dv_z`          | m/s | Velocity increment along Z                      |
| `temperature_C` | °C  | Sensor temperature (`22.5` in example files)    |
| `status`        | –   | Data validity flag (`1` for valid samples)      |

The time column increases monotonically and does not wrap. `IMU_X001.dat`
contains 500 000 rows, whereas the `_small` variants hold only the first
1 000 samples for quick tests.

## Steps

1. Load the IMU file into memory so the full dataset is available for later
   tasks, then extract the increment columns.
2. Estimate the sampling period from the time values.
3. Convert increments to rates and apply a low-pass filter.
4. Detect a static interval with `detect_static_interval` and compute the mean accelerometer and gyroscope vectors.
5. Scale the accelerometer mean so its norm equals $9.81\,\mathrm{m/s^2}$.
6. Form

   $$
   \mathbf{g}_b = -\text{acc\_mean},\qquad
   \boldsymbol{\omega}_{ie,b} = \text{gyro\_mean}.
   $$
7. Record the window indices, sampling period and magnitudes in `triad_init_log.txt`.
8. Save the ZUPT diagnostic plot as `results/<tag>_ZUPT_variance.pdf`.

These body-frame vectors correspond to the Task\u00a01 references expressed in sensor axes and are used to determine the initial attitude in Task\u00a03.
