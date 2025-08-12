# IMU GNSS Fusion
GNSS-IMU sensor fusion system for navigation and attitude estimation with Python and MATLAB implementations.

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

## Working Effectively
- Bootstrap and install dependencies:
  - `pip install --upgrade pip setuptools wheel build` -- takes 10 seconds. NEVER CANCEL.
  - `pip install -r requirements.txt` -- takes 40 seconds. NEVER CANCEL. Set timeout to 300+ seconds.
  - If network timeouts occur, retry the pip install command. This is a known issue with PyPI connectivity.
  - `pip install pytest flake8` -- for testing and linting, takes 5 seconds. NEVER CANCEL.
- Run tests:
  - `pytest -q` -- takes 3 minutes. NEVER CANCEL. Set timeout to 300+ seconds.
  - 56 tests pass, 9 skip (MATLAB tests skip when MATLAB unavailable - this is normal)
- Process datasets (core functionality):
  - **Quick test**: `python src/run_all_datasets.py --method TRIAD --datasets X001 --config config_small.yml` -- takes 30 seconds. NEVER CANCEL.
  - **Full processing**: `python src/run_all_datasets.py` -- takes 5-15 minutes depending on datasets. NEVER CANCEL. Set timeout to 1200+ seconds.
  - **Single dataset**: `python src/run_all_datasets.py --method TRIAD --datasets X001` -- takes 2-5 minutes. NEVER CANCEL. Set timeout to 600+ seconds.

## Validation
- Always run the quick test with small dataset first to verify your changes work.
- Run full test suite with `pytest -q` after making changes.
- Always run `flake8 src tests` for linting before committing changes.
- Use `python validate_all_outputs.py` to validate processing results after running datasets.
- **Manual validation**: After processing, check that output files are created in `results/` directory with expected .png, .mat, and .npz files.

## Dependencies and Installation
- **CRITICAL**: Network timeouts are common when installing Python packages. Always retry failed pip installations.
- Required packages: numpy, scipy, matplotlib, pandas, filterpy, cartopy, tqdm, rich, tabulate
- Optional: MATLAB (9 tests skip without it, but core Python functionality works)
- Do NOT use `make test` or `pip install -e .[tests]` - these timeout due to network issues. Use direct pip installs instead.
- If filterpy installation fails: `pip install filterpy --no-binary :all:`

## Common Tasks
### Build and Test Commands
```bash
# Install dependencies (40 seconds, can timeout - retry if needed)
pip install --upgrade pip setuptools wheel build
pip install -r requirements.txt
pip install pytest flake8

# Run tests (3 minutes)
pytest -q

# Lint code (2 seconds)
flake8 src tests
```

### Dataset Processing Commands
```bash
# Quick test with small dataset (30 seconds)
python src/run_all_datasets.py --method TRIAD --datasets X001 --config config_small.yml

# Process single full dataset (2-5 minutes)
python src/run_all_datasets.py --method TRIAD --datasets X001

# Process all datasets with all methods (15-30 minutes)
python src/run_all_datasets.py

# Validate results
python validate_all_outputs.py
```

### Key Entry Points
- `src/run_all_datasets.py` - Main batch processing script
- `src/GNSS_IMU_Fusion.py` - Core fusion algorithm (do not run directly, has import issues)
- `validate_all_outputs.py` - Validation and plotting tool
- `align_datasets_dtw.py` - Dataset alignment utility

### Dataset Information
- Full datasets: IMU_X001.dat (500k samples), IMU_X002.dat, IMU_X003.dat 
- Small test datasets: IMU_X001_small.dat (1k samples), etc.
- GNSS data: GNSS_X001.csv (1250 epochs), GNSS_X002.csv
- Ground truth: STATE_X001.txt, STATE_X001_small.txt
- Processing times: Small datasets ~30s, full datasets 2-5 minutes each

### Output Structure
- Results saved to `results/` directory
- Plots: .png files for visualization
- Data: .mat files (MATLAB format), .npz files (NumPy format) 
- Typical outputs: task4_all_ned.png, task5_results_*.png, residuals_*.png

## Common Issues and Workarounds
- **Network timeouts during pip install**: Retry the command, this is normal due to PyPI connectivity
- **"No module named ruff"**: Install flake8 instead: `pip install flake8`
- **Import errors in FINAL.py**: Use `src/run_all_datasets.py` instead for batch processing
- **MATLAB tests skipping**: Normal behavior when MATLAB not available, Python functionality unaffected
- **Memory warnings about figures**: Normal during processing, does not affect results

## File Structure Reference
```
src/
├── run_all_datasets.py        # Main batch processor
├── GNSS_IMU_Fusion.py         # Core fusion algorithm
├── validate_with_truth.py     # Truth comparison
└── utils/                     # Helper modules

tests/                          # pytest test suite
requirements.txt               # Python dependencies
pyproject.toml                # Project configuration
Makefile                      # Build targets (avoid due to timeout issues)

Data files:
├── IMU_X001.dat              # Full IMU dataset (~64MB)
├── IMU_X001_small.dat        # Test dataset (~131KB)
├── GNSS_X001.csv             # GNSS measurements
├── STATE_X001.txt            # Ground truth trajectory
└── config_small.yml          # Test configuration
```

## Performance Expectations
- Dependency installation: 40-60 seconds (network dependent)
- Test suite: 3 minutes (56 pass, 9 skip)
- Small dataset processing: 30 seconds
- Full dataset processing: 2-5 minutes per dataset
- Linting: 2 seconds
- **Always use timeouts of 300+ seconds for pip installs and 600+ seconds for dataset processing**

## Advanced Usage
- Custom configurations: Edit YAML files in root directory
- Method comparison: Use `--method ALL` to run TRIAD, Davenport, and SVD methods
- Specific validation: `python src/validate_with_truth.py --est-file <result.mat> --truth-file STATE_X001.txt`
- DTW alignment: `python align_datasets_dtw.py --est-file <result> --truth-file <truth>`