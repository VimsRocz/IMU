# Plotting Examples and Reporting Template

This page provides ready-to-use snippets for visualising your IMU data and a template for documenting the results.

## Plot ZUPT-Detected Intervals and Accelerometer Variance

```python
import numpy as np
import matplotlib.pyplot as plt

# accel: ndarray (N,3)  - raw accelerometer data
# zupt_mask: ndarray (N,)  - True where ZUPT was detected
# dt: float                - IMU sampling interval in seconds
# threshold: float         - variance threshold for the static detector
# window_size: int         - length of the moving window

def plot_zupt_variance(accel, zupt_mask, dt, threshold, window_size=50):
    accel_norm = np.linalg.norm(accel, axis=1)
    accel_norm_mean = np.mean(accel_norm)
    accel_var = np.convolve((accel_norm - accel_norm_mean)**2,
                            np.ones(window_size)/window_size, mode='same')

    time = np.arange(accel.shape[0]) * dt

    plt.figure(figsize=(12, 5))
    plt.plot(time, accel_var, label='Accel Variance (Norm, moving window)')
    plt.axhline(threshold, color='r', linestyle='--', label='Static Threshold')
    plt.fill_between(time, 0, np.max(accel_var), where=zupt_mask,
                     color='lime', alpha=0.3, label='ZUPT Detected')
    plt.xlabel('Time [s]')
    plt.ylabel('Accelerometer Variance')
    plt.title('ZUPT Detection & Accelerometer Variance')
    plt.legend()
    plt.tight_layout()
    plt.show()
```

Call `plot_zupt_variance()` for each dataset after computing your ZUPT mask.

## Plot Euler Angles for Each Dataset

```python
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt

# rotmats: list of (3,3) rotation matrices from body to navigation frame
# labels: list of dataset names, e.g. ['X001', 'X002', 'X003']

def plot_initial_eulers(rotmats, labels):
    eulers_deg = []
    for mat in rotmats:
        r = R.from_matrix(mat)
        eulers_deg.append(r.as_euler('zyx', degrees=True))  # yaw, pitch, roll

    eulers_deg = np.array(eulers_deg)
    angles = ['Yaw (ψ)', 'Pitch (θ)', 'Roll (φ)']

    plt.figure(figsize=(8, 4))
    for i in range(3):
        plt.plot(labels, eulers_deg[:, i], 'o-', label=angles[i])
    plt.xlabel('Dataset')
    plt.ylabel('Angle [deg]')
    plt.title('TRIAD Initialization: Euler Angles per Dataset')
    plt.legend()
    plt.grid()
    plt.tight_layout()
    plt.show()
```

## Markdown Template for Results

```markdown
# IMU+GNSS Attitude Initialization and ZUPT-Aided Navigation: Documentation

## 1. Attitude Initialization (TRIAD)
- Datasets: X001, X002, X003
- Initial Euler angles (degrees):

| Dataset | Yaw (ψ) | Pitch (θ) | Roll (φ) |
|---------|---------|-----------|----------|
| X001    |   ...   |   ...     |   ...    |
| X002    |   ...   |   ...     |   ...    |
| X003    |   ...   |   ...     |   ...    |

---

## 2. ZUPT Detection
- Accelerometer variance threshold: `...`
- Static window size: `...`
- Number of ZUPT events: `...`
- Example plot:
  ![variance_plot](./variance_plot.png)

---

## 3. Pipeline Logging
- Initial static window: start = `...`, end = `...`
- ZUPT count: `...`
- Any warnings: `...`

---

## 4. Results
- RMSE Position: 0.28–0.29 m
- End Error: 0.01 m
- Velocity RMSE: 0.01–0.10 m/s
- Large static drift **eliminated** with ZUPT.

---

## 5. Comments / Next Steps
- For <0.5° heading, consider SVD or magnetometer.
- Adaptive threshold? (If needed in future)
```

Use this template to record the key parameters and metrics for each run.
