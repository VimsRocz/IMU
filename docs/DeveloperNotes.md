# Developer Notes

This page collects advanced tips and background information removed from the README for brevity.

## IMU Bias Detection
`GNSS_IMU_Fusion.py` automatically detects a static interval to estimate accelerometer and gyroscope biases. It also rescales the accelerometer vector so its magnitude matches 9.81 m/s^2. This improves initial attitude estimation when the sensor scale is slightly off.

## Verifying Earth Rotation Rate
Run any dataset with `--verbose` to print the measured Earth rotation magnitude. It should be close to `7.29e-5` rad/s. You can compute the nominal value in Python:

```bash
python - <<"EOF2"
import numpy as np; print(2*np.pi/86400)
EOF2
```

A significant difference indicates the sensor might not be still or needs calibration.

## Additional Scripts
The repository contains a few helper tools not covered by the core pipeline:

- `fusion_single.py` – early stand-alone fusion prototype showing the full bias estimation and Kalman logic.
- `validate_filter.py` – command-line tool to compare a saved filter state against GNSS measurements.

These utilities are optional and excluded from the unit tests.

## Next Steps
- **Logging:** Consider integrating the `rich` console handler for colourful status messages.
- **Documentation:** Generate API docs with Sphinx and publish guides with MkDocs.
- **CI:** A GitHub Actions workflow should install `requirements.txt`, run `flake8` and `pytest` on each pull request.
- **Debugging:** If the MATLAB pipeline drifts badly, see `docs/DebuggingDrift.md` for troubleshooting.

See [docs/CHANGELOG.md](CHANGELOG.md) for recent features.
