# IMU-
IMU data processing and initialization tools (Python)

## Installation

Install dependencies with:

```bash
pip install -r requirements.txt
```

## Notes

`GNSS_IMU_Fusion.py` now detects a low-motion interval in the IMU data to
estimate the initial accelerometer and gyroscope biases. The magnitude of the
static accelerometer vector is used to compute a simple scale factor so that the
measured gravity is close to 9.81 m/s². This improves the attitude
initialisation when the first samples are not perfectly static or the sensor
scale is slightly off.

## Running all methods

Use `run_all_methods.py` to execute the fusion script with TRIAD, Davenport and SVD sequentially:

```bash
python run_all_methods.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv
```

## Running all data sets

To process every IMU/GNSS pair defined in `run_all_datasets.py`, simply run:

```bash
python run_all_datasets.py
```

## Tests

Run the unit tests with `pytest`:

```bash
pytest -q
```
