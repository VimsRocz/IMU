# Kalman Filter Fusion

**Task&nbsp;5** fuses the inertial solution with GNSS using a simple Kalman filter. Zero-velocity updates (ZUPT) are applied during stationary periods. The output includes position, velocity and acceleration along with residuals.

Example result files:

```
results/run_triad_only/IMU_X001_GNSS_X001_TRIAD_task5_results_TRIAD.pdf
```

Typical residual root-mean-square errors are below $0.3\,\mathrm{m}$ for position and $0.1\,\mathrm{m/s}$ for velocity.
