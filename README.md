# IMU + GNSS Fusion (TRIAD / EKF) — Repository Layout

This repository contains both **Python** and **MATLAB** pipelines for IMU/GNSS fusion,
now organized with a clean data directory and separate results folders.

## Directory Structure

DATA/
IMU/      IMU_X001.dat, IMU_X002.dat, IMU_X003.dat, *_small.dat
GNSS/     GNSS_X001.csv, GNSS_X002.csv, *_small.csv
TRUTH/    STATE_X001.txt, STATE_X001_small.txt

MATLAB/
… (scripts and tasks)
results/  (MATLAB-generated outputs)

results/
python/   (Python-generated outputs)

src/
… (Python modules & CLI)

## Running (Python)

```bash
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt

# Example: run TRIAD-only with explicit file names (searches DATA/* automatically)
python -m src.run_triad_only_cli
  --imu IMU_X002.dat
  --gnss GNSS_X002.csv
  --truth STATE_X001.txt
  --results results/python
```

## Running (MATLAB)

Open MATLAB in repo root:

```matlab
% Example TRIAD run; file resolvers search DATA/*, so bare names work.
run_triad_only
```

Results appear under MATLAB/results/.

## Notes

- Time handling:
- IMU: 400 Hz synthetic timeline (t0=0, dt=0.0025s), raw clock can reset → we unwrap to monotonic time internally.
- GNSS: 1 Hz using Posix_Time (monotonic).
- TRUTH: read from STATE_X001.txt (ECEF position/velocity); aligned/interpolated as needed.
- The system now tolerates bare filenames and new DATA/* layout; old absolute paths still work.
