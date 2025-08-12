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
PYTHONPATH=src python3 src/run_triad_only.py \
  --imu ../DATA/IMU/IMU_X002.dat \
  --gnss ../DATA/GNSS/GNSS_X002.csv \
  --truth ../DATA/Truth/STATE_X001.txt \
  --outdir results
```

Options
• --imu PATH, --gnss PATH, --truth PATH: dataset files
• --outdir DIR (default results)
• --config config/config_small.yml (if supported by the script)
• --verbose

Outputs

Artifacts (PNG/PDF/JSON/txt) are written to PYTHON/results/.

Troubleshooting: ModuleNotFoundError: utils.utils

If you have both src/utils.py and src/utils/:

Create this file:

PYTHON/src/utils/__init__.py

Contents:

```
from importlib import import_module as _imp
_utils = _imp('utils')  # loads PYTHON/src/utils.py
compute_C_ECEF_to_NED = _utils.compute_C_ECEF_to_NED
ecef_to_geodetic = _utils.ecef_to_geodetic
__all__ = ['compute_C_ECEF_to_NED', 'ecef_to_geodetic']
```

Run with:

```
PYTHONPATH=src python3 src/run_triad_only.py ...
```

Tests (optional)

```
pip install -r requirements-dev.txt
PYTHONPATH=src pytest -q tests
```

