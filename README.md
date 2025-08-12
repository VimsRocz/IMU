# IMU — GNSS + IMU Data Processing (Python & MATLAB)

This repository hosts parallel **Python** and **MATLAB** implementations for IMU/GNSS initialization and analysis. Each language is self-contained; **data are shared** via `DATA/`.

## Repository Layout

```
IMU/
├── DATA/                 # shared inputs (IMU, GNSS, Truth)
├── PYTHON/               # Python environment
│   ├── src/              # code
│   ├── results/          # generated artifacts
│   ├── config/           # YAML/JSON configs
│   └── docs/             # Python-specific docs
└── MATLAB/               # MATLAB environment
    ├── src/              # code
    ├── results/          # generated artifacts
    ├── config/           # scripts/MAT configs
    └── docs/             # MATLAB-specific docs
```

## Quick Start

### Data
Place datasets under `DATA/`:

```
DATA/
├── IMU/    IMU_X00{1,2,3}.dat
├── GNSS/   GNSS_X00{1,2}.csv
└── Truth/  STATE_X001.txt  (and _small)
```

### Python (TRIAD / Task-1)
```bash
cd PYTHON
python3 -m venv .venv
source .venv/bin/activate             # Windows: .venv\Scripts\activate
pip install -r requirements.txt

# Run TRIAD
PYTHONPATH=src python3 src/run_triad_only.py \
  --imu ../DATA/IMU/IMU_X002.dat \
  --gnss ../DATA/GNSS/GNSS_X002.csv \
  --truth ../DATA/Truth/STATE_X001.txt \
  --outdir results

Outputs: PYTHON/results/…
```

### MATLAB (Task-1)

```matlab
cd('MATLAB/src'); addpath(genpath(pwd));
Task_1   % or run_triad_only.m

Outputs: MATLAB/results/…
```

### Troubleshooting

Python import error `utils.utils`: see `PYTHON/README.md` for a one-file bridge that resolves the module/package name collision.

### Contributing

Work on a feature branch (e.g., `TASK_1`), push, and open a PR into main. Do not commit generated outputs outside the `results/` folders.

MIT License

