# IMU Repository — Final Layout

DATA/
IMU/      IMU_X001.dat, IMU_X002.dat, IMU_X003.dat, *_small.dat
GNSS/     GNSS_X001.csv, GNSS_X002.csv, *_small.csv
TRUTH/    STATE_X001.txt, STATE_X001_small.txt

python/
src/        (Python package & modules)
scripts/    (entry-point scripts & utilities)
tools/      (dev tools)
tests/      (python tests)
requirements.txt, pyproject.toml

MATLAB/
…         (MATLAB tasks & utils)
tests/      (MATLAB tests)
results/    (MATLAB outputs)

results/
python/     (Python outputs)
logs/

## Run (Python)
```bash
cd python
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python -m src.run_triad_only_cli   --imu IMU_X002.dat --gnss GNSS_X002.csv --truth STATE_X001.txt   --results ../results/python
```

## Run (MATLAB)

Open MATLAB in repo root and run:

```matlab
run_triad_only
```

MATLAB and Python both auto-search DATA/IMU, DATA/GNSS, DATA/TRUTH.
