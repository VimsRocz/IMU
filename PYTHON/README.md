PYTHON — IMU/GNSS Processing

All Python code is in src/, outputs in results/.

Setup

```
cd PYTHON
python3 -m venv .venv
source .venv/bin/activate             # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Data

Shared in ../DATA/:
• ../DATA/IMU/IMU_X002.dat
• ../DATA/GNSS/GNSS_X002.csv
• ../DATA/Truth/STATE_X001.txt

New datasets should follow this layout (recommended):
- IMU logs: `DATA/IMU/IMU_<ID>.dat` (e.g., `IMU_X123.dat`)
- GNSS logs: `DATA/GNSS/GNSS_<ID>.csv` (e.g., `GNSS_X123.csv`)
- Optional truth: `DATA/Truth/STATE_<ID>.txt`

Run TRIAD (Task-1)

```
# From repo root
python PYTHON/src/run_triad_only.py --dataset X002

# Or from PYTHON directory
python src/run_triad_only.py --dataset X002

# Optional explicit overrides
#   --imu/--gnss/--truth/--outdir
```

Options
• --imu PATH, --gnss PATH, --truth PATH: dataset files
• --outdir DIR (default results)
• --config config/config_small.yml (if supported by the script)
• --verbose

Outputs

Artifacts (PNG/PDF/JSON/txt) are written to PYTHON/results/.

Notes
- No `PYTHONPATH` needed: entry scripts add `src/` to `sys.path` automatically.
- `utils` collision resolved: `src/utils/__init__.py` forwards symbols from `utils_legacy`.

Tests (optional)

```
pip install -r requirements-dev.txt
pytest -q tests
```

Using a YAML config (multiple datasets)

- Create a config file like `config/run_example.yml`:

```
methods: [TRIAD, Davenport, SVD]
datasets:
  - imu: DATA/IMU/IMU_X001.dat
    gnss: DATA/GNSS/GNSS_X001.csv
  - imu: DATA/IMU/IMU_X002.dat
    gnss: DATA/GNSS/GNSS_X002.csv
    init:        # optional per‑dataset initial position override
      lat_deg: 47.3977
      lon_deg: 8.5456
      alt_m:  488.0
init:            # optional global initial position override (used if per‑dataset missing)
  lat_deg: 47.3977
  lon_deg: 8.5456
  alt_m:  488.0
```

- Run all cases and collect logs/results:

```
python src/run_all_datasets.py --config config/run_example.yml --verbose
```

- Logs: `src/logs/<IMU>_<GNSS>_<METHOD>_<timestamp>.log`
- Outputs: `PYTHON/results/` (plots, NPZ/MAT, summary CSV)

Initial position override (single run)

If GNSS ECEF does not contain a good first fix, you can override the
initial latitude/longitude/altitude on the CLI:

```
python src/GNSS_IMU_Fusion.py \
  --imu-file DATA/IMU/IMU_X123.dat \
  --gnss-file DATA/GNSS/GNSS_X123.csv \
  --method TRIAD \
  --init-lat-deg 47.3977 --init-lon-deg 8.5456 --init-alt-m 488
```

MATLAB quick notes

- MATLAB helpers in `PYTHON/src/run_all_methods.m` are a simplified stub.
- Ensure your data lives under `DATA/IMU` and `DATA/GNSS`. If your copy of
  `run_all_methods.m` looks for files in the repo root (e.g. `IMU_X002.dat`),
  update it to prefix `DATA/IMU` and `DATA/GNSS`, or move the files accordingly.
- For running the full pipeline, prefer the Python entry points above.

User manual formatting

- If long commands wrap poorly in your PDF viewer, enable word wrap or reduce
  the zoom level. All commands here are provided in fenced code blocks which
  copy/paste cleanly.

Minimal workflow (single dataset)

- Prepare one IMU file and one GNSS file under `DATA/IMU` and `DATA/GNSS`.
- Edit `config/single_run.yml` to set:
  - `imu`: path to IMU file
  - `gnss`: path to GNSS file
  - `method`: one of `TRIAD|Davenport|SVD` (default TRIAD)
  - `init.lat_deg`, `init.lon_deg`, `init.alt_m`: launch site position
- Run the one-shot processor:

```
python src/process_one.py --config config/single_run.yml --verbose
```

Outputs
- Data: `PYTHON/results/<IMU>_<GNSS>_<METHOD>_kf_output.npz` and `.mat`
- Plots: several `*.png` and MATLAB-compatible `*.fig` under `PYTHON/results/`

Use results in MATLAB
- Load the `.mat` file and plot basic NED states:

```
% In MATLAB, from repo root or PYTHON/
addpath('PYTHON/MATLAB/utils');
plot_from_mat('PYTHON/results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat');
```

- To generate a fuller set of task plots (attitude, NED overlays, residuals, body-frame):

```
addpath('PYTHON/MATLAB/utils');
plot_all_tasks_from_mat('PYTHON/results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat');
```
