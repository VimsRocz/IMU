# IMU Integration

In **Task&nbsp;4** the corrected accelerometer samples are integrated to obtain velocity and position. Each dataset uses the rotation matrix from Task&nbsp;3.

Biases are estimated during the static interval, e.g. for X001 the accelerometer bias was approximately $[0.56,-6.84,0.91]\,\mathrm{m/s^2}$. The gravity vector for the integration step is $[0,0,9.81]\,\mathrm{m/s^2}$.

Figures comparing the IMU trajectory against GNSS are saved as

```
results/run_triad_only/<tag>_task4_comparison_ned.png
results/run_triad_only/<tag>_task4_all_ned.png
results/run_triad_only/<tag>_task4_all_ecef.png
results/run_triad_only/<tag>_task4_all_body.png
```
