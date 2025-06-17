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

def plot_zupt_and_variance(accel, zupt_mask, dt, threshold, window_size=100):
    t = np.arange(accel.shape[0]) * dt
    accel_norm = np.linalg.norm(accel, axis=1)
    mean_conv = np.ones(window_size) / window_size
    var = np.convolve(accel_norm**2, mean_conv, mode='same') - \
          np.convolve(accel_norm, mean_conv, mode='same')**2

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, var, label='Accel Norm Variance', color='tab:blue')
    ax.axhline(threshold, color='gray', linestyle='--', label='ZUPT threshold')
    ax.fill_between(t, 0, np.max(var), where=zupt_mask,
                    color='tab:orange', alpha=0.3, label='ZUPT Detected')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Variance')
    ax.set_title('ZUPT Detected Intervals & Accelerometer Variance')
    ax.legend()
    plt.tight_layout()
    plt.show()
```

Call `plot_zupt_and_variance()` for each dataset after computing your ZUPT mask.

## Plot Euler Angles for Each Dataset

```python
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt

# rotmats: list of (3,3) rotation matrices from body to navigation frame
# labels: list of dataset names, e.g. ['X001', 'X002', 'X003']

def plot_triad_euler(rotmats, labels):
    eulers = []
    for mat in rotmats:
        r = R.from_matrix(mat)
        eulers.append(r.as_euler('zyx', degrees=True))

    eulers = np.array(eulers)
    plt.figure(figsize=(8, 4))
    plt.plot(labels, eulers[:, 2], 'o-', label='Roll [°]')
    plt.plot(labels, eulers[:, 1], 'o-', label='Pitch [°]')
    plt.plot(labels, eulers[:, 0], 'o-', label='Yaw [°]')
    plt.xlabel('Dataset')
    plt.ylabel('Angle [deg]')
    plt.title('TRIAD Attitude Initialization (Euler Angles)')
    plt.grid()
    plt.legend()
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
