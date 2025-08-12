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
