# IMU

[![GitHub release](https://img.shields.io/github/v/release/VimsRocz/IMU?logo=github)](https://github.com/VimsRocz/IMU/releases) [![GitHub stars](https://img.shields.io/github/stars/VimsRocz/IMU?style=social)](https://github.com/VimsRocz/IMU/stargazers) [![GitHub forks](https://img.shields.io/github/forks/VimsRocz/IMU?style=social)](https://github.com/VimsRocz/IMU/network/members) [![Follow on GitHub](https://img.shields.io/github/followers/VimsRocz?style=social)](https://github.com/VimsRocz) [![License](https://img.shields.io/github/license/VimsRocz/IMU)](LICENSE) [![Python CI](https://github.com/VimsRocz/IMU/actions/workflows/python-ci.yml/badge.svg)](https://github.com/VimsRocz/IMU/actions/workflows/python-ci.yml) [![PyPI](https://img.shields.io/pypi/v/imu_gnss_fusion)](https://pypi.org/project/imu_gnss_fusion/) [![CodeQL](https://github.com/VimsRocz/IMU/actions/workflows/codeql.yml/badge.svg)](https://github.com/VimsRocz/IMU/actions/workflows/codeql.yml)


Simple GNSS/IMU fusion playground.  All data live in `DATA/` and are treated as
read‑only.

## Repository Structure
```
IMU/
├─ DATA/
│  ├─ IMU/    # IMU_X00*.dat
│  ├─ GNSS/   # GNSS_X00*.csv
│  └─ Truth/  # STATE_X001*.txt
├─ PYTHON/
│  ├─ src/      # Python modules and packages
│  ├─ tests/    # Python tests
│  ├─ scripts/  # helper scripts
│  └─ results/  # Python outputs
├─ MATLAB/
│  ├─ src/      # MATLAB functions
│  ├─ scripts/  # helper scripts
│  └─ results/  # MATLAB outputs
└─ docs/        # project docs
```

## How to Run
### Python
```bash
pip install -r requirements.txt  # if needed
python PYTHON/GNSS_IMU_Fusion_Single_script.py
```
This script reads from `DATA/` and writes plots to `PYTHON/results/`.

### MATLAB
```matlab
addpath('MATLAB');
run_triad_only('IMU_X002','GNSS_X002','TRIAD');
```
Outputs are written to `MATLAB/results/`.

Keep paths flat, keep data read‑only, and put generated files in the appropriate
`results/` folder.
