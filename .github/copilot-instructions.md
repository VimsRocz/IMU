# IMU/GNSS Sensor Fusion Library
IMU/GNSS sensor fusion library in Python with MATLAB components for inertial navigation and attitude estimation.

Always reference these instructions first and fallback to search or bash commands only when you encounter unexpected information that does not match the info here.

## Working Effectively
- Bootstrap and set up the repository:
  - `pip install --upgrade pip setuptools wheel build`
  - `pip install -r requirements.txt` -- takes 40 seconds. NEVER CANCEL.
  - `pip install pytest ruff flake8` -- for testing and linting
- Run the full test suite:
  - `pytest -q tests/` -- takes 3 minutes. NEVER CANCEL. Set timeout to 300+ seconds.
  - 56+ tests pass, 8-9 skipped (due to no MATLAB), 11 warnings expected
- Build and run the main applications:
  - Small dataset test: `python src/GNSS_IMU_Fusion.py --imu-file IMU_X001_small.dat --gnss-file GNSS_X001_small.csv --method TRIAD --verbose` -- takes 18 seconds
  - Full dataset test: `python src/run_all_datasets.py --method TRIAD --datasets X001` -- takes 5 minutes. NEVER CANCEL. Set timeout to 600+ seconds.
  - Config-based run: `python src/run_all_methods.py --config config_small.yml` -- takes 2 minutes. NEVER CANCEL. Set timeout to 300+ seconds.

## Validation
- Always manually validate changes by running the fusion algorithms on test data.
- ALWAYS run through at least one complete end-to-end scenario after making changes:
  - Run: `python src/GNSS_IMU_Fusion.py --imu-file IMU_X001_small.dat --gnss-file GNSS_X001_small.csv --method TRIAD`
  - Verify: Check that `results/` directory contains generated plots and output files
  - Expected: Script completes successfully with position/velocity outputs and plots saved
- Validation scripts:
  - `python src/validate_with_truth.py --est-file results/[output].mat --truth-file STATE_X001.txt --output results` -- takes 14 seconds
- Always run linting before committing: `ruff check src tests` or `flake8 .` -- takes under 1 second
- The codebase has some expected linting issues (32 ruff errors, extensive flake8 warnings) - don't fix these unless specifically tasked

## Common Tasks
The following are outputs from frequently run commands. Reference them instead of viewing, searching, or running bash commands to save time.

### Repository Structure
```
.
├── .github/workflows/     # CI configuration
├── MATLAB/               # MATLAB implementation
├── src/                  # Python source code
├── tests/                # Test suite
├── docs/                 # Documentation
├── results/              # Output directory (created automatically)
├── requirements.txt      # Python dependencies
├── pyproject.toml        # Project configuration
├── Makefile             # Build automation
├── README.md            # Comprehensive documentation
├── IMU_X*.dat           # IMU data files (full and _small versions)
├── GNSS_X*.csv          # GNSS data files (full and _small versions)
├── STATE_X001.txt       # Ground truth reference data
└── config_small*.yml    # Configuration files for batch processing
```

### Key Entry Points
- `src/GNSS_IMU_Fusion.py` -- Main fusion script, processes single dataset
- `src/run_all_datasets.py` -- Batch processing of all datasets
- `src/run_all_methods.py` -- Run with config file, supports all methods
- `src/run_triad_only.py` -- TRIAD method only on all datasets
- `src/FINAL.py` -- Process first dataset with all methods
- `src/validate_with_truth.py` -- Compare results with ground truth

### Available Datasets
- Small datasets (for quick testing): `IMU_X001_small.dat`, `IMU_X002_small.dat`, `IMU_X003_small.dat`
- Full datasets: `IMU_X001.dat`, `IMU_X002.dat`, `IMU_X003.dat`
- GNSS data: `GNSS_X001.csv`, `GNSS_X002.csv` (X003 uses X002's GNSS)
- Ground truth: `STATE_X001.txt`

### Methods Available
- TRIAD -- Traditional three-axis attitude determination
- Davenport -- Davenport's Q-method for attitude estimation  
- SVD -- Singular Value Decomposition method

### Expected Timing and Warnings
- **CRITICAL**: Set appropriate timeouts (300+ seconds) for test commands and batch processing. DO NOT use default timeouts.
- Dependencies install: ~40 seconds
- Test suite: ~3 minutes (pytest -q tests/)
- Small dataset fusion: ~18 seconds per method
- Full dataset fusion: ~5 minutes per method
- Config-based batch run: ~2 minutes for small datasets
- Validation: ~14 seconds per output file
- **NEVER CANCEL** long-running operations. Builds and tests may take several minutes.

### Python Environment Notes
- Requires Python 3.9+, tested with Python 3.12
- Some packages (filterpy, fastdtw, fpdf) build from source - this is normal
- Editable installs (`pip install -e .`) may fail due to network timeouts - use direct requirements.txt install instead
- MATLAB components are optional - tests skip MATLAB-dependent functionality automatically

### CI and Quality Checks
- GitHub Actions workflow: `.github/workflows/python-ci.yml`
- Linting: `ruff check src tests` (expect 32 errors) or `flake8 .` (expect many warnings)
- Code analysis: GitHub CodeQL configured
- The codebase has known linting issues that are not critical to functionality

### File Output Patterns
Results are saved to `results/` directory with naming pattern:
- `IMU_X[dataset]_GNSS_X[dataset]_[method]_[task][N]_[description].[ext]`
- Example: `IMU_X001_GNSS_X001_TRIAD_task5_results_ned_TRIAD.png`
- Supported formats: `.png`, `.pickle`, `.mat`, `.npz`

### Configuration Files
- `config_small.yml` -- Small datasets with TRIAD method only
- `config_small_all.yml` -- Small datasets with all methods
- `example_config.yaml` -- Full datasets with all methods

### Troubleshooting
- If validation script fails with "No module named 'utils'", run from src/: `python src/validate_with_truth.py ...`
- Network timeouts during pip installs are common - retry or use alternative approaches
- Results directory is created automatically by scripts
- MATLAB tests are skipped automatically if MATLAB is not available
- Large output files (>100MB) in results/ are normal for full dataset processing

### Manual Testing Workflow
After making changes, always run this validation sequence:
1. `python src/GNSS_IMU_Fusion.py --imu-file IMU_X001_small.dat --gnss-file GNSS_X001_small.csv --method TRIAD`
2. Check that `results/` contains new `.png` and `.pickle` files
3. Verify console output shows successful completion with position/velocity statistics
4. Run `ruff check src tests` to check for new linting issues
5. Run subset of tests: `pytest tests/test_python_accuracy.py -v` (takes ~1 minute)