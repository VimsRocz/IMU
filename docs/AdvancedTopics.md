# Advanced Topics

## Developing & Debugging in GitHub Codespaces

1. **Open in Codespace**
   Click **Code → Open with Codespaces** on the repo.

2. **Install dependencies**
   Codespaces will automatically build the container (per `.devcontainer/`), install Python & your `requirements.txt`.

3. **Enable verbose diagnostics**
   We’ve added a `--verbose` flag to `src/run_all_datasets.py` that enables all the extra tables and timing you requested. `src/GNSS_IMU_Fusion.py` also accepts `--verbose` for the same detailed diagnostics.

4. **Run with diagnostics**
   In the integrated terminal:
   ```bash
   python src/run_all_datasets.py --verbose 2>&1 | tee debug_output.log
   ```

This will print:
        •       Static-Calibration Summary

=== IMU Calibration ===
Accel bias [m/s²]:    [ 0.5687, -6.9884,  0.9356 ]
Gyro bias  [rad/s]:   [ -1e-08, 6.6e-09, -3.1e-09 ]
Accel scale factor:   1.0000
Static interval:      samples 234–634 (Δt=1 s)

        •       Attitude-Determination Diagnostics for each method (gravity/Earth-rate errors, condition numbers, degeneracies)
        •       GNSS Quality Metrics (HDOP/PDOP statistics, satellite counts)
        •       ZUPT & KF events summary (counts, total ZUPT time, final covariance)
        •       Trajectory-Error Time Series (t, pos-error, vel-error)
        •       Timing & Performance per task + total
        •       Final Comparison Table across datasets & methods

        5.      Copy & paste
Once it finishes, open debug_output.log, copy the console dump and paste it back here so we can verify and iterate.

## MATLAB Usage

### Running the MATLAB pipeline
```matlab
imu_path  = get_data_file('IMU_X001.dat');
gnss_path = get_data_file('GNSS_X001.csv');
TRIAD(imu_path, gnss_path);
```

Scripts such as `TRIAD.m` take the full paths to the IMU and GNSS files as arguments. The helper function `get_data_file` searches both the repository root and `MATLAB/data`, letting `TRIAD` and related scripts locate the bundled sample logs automatically when you pass just the file names.

### MATLAB Scripts

All MATLAB code now lives in the single `MATLAB/` directory. It contains the pipeline tasks (`Task_1`–`Task_5`) alongside helper scripts such as `TRIAD.m`, `FINAL.m`, `plot_results.m` and `validate_3sigma.m`.

To run `TRIAD.m` with the new data-file detection logic simply resolve the file paths with `get_data_file` and pass them to the script:
```matlab
imu  = get_data_file('IMU_X001.dat');
gnss = get_data_file('GNSS_X001.csv');
TRIAD(imu, gnss);
```
`get_data_file` searches `MATLAB/data/` first and falls back to the repository root.

### Sequential Task Execution

To replicate the Python pipeline step by step, call each `Task_1`–`Task_5` function in order. Always resolve the data paths with `get_data_file` so the commands work from any folder. Every task writes a `.mat` file that the next one loads:
```matlab
imu  = get_data_file('IMU_X001.dat');
gnss = get_data_file('GNSS_X001.csv');

Task_1(imu, gnss, 'TRIAD');   % -> results/Task1_init_IMU_X001_GNSS_X001_TRIAD.mat
Task_2(imu, gnss, 'TRIAD');   % -> results/Task2_body_IMU_X001_GNSS_X001_TRIAD.mat
Task_3(imu, gnss, 'TRIAD');   % uses Task1/2 output, writes Task3_results_IMU_X001_GNSS_X001.mat
Task_4(imu, gnss, 'TRIAD');   % uses Task3 results, writes Task4_results_IMU_X001_GNSS_X001.mat
Task_5(imu, gnss, 'TRIAD');   % uses Task4 results, writes Task5_results_IMU_X001_GNSS_X001.mat
```

### MATLAB-only pipeline
```matlab
run_all_datasets_matlab
```
The script scans `Data/` (or the repository root if that folder is missing) for matching IMU and GNSS logs, executes `Task_1` through `Task_5` for each pair and saves the results as `IMU_<id>_GNSS_<id>_TRIAD_kf_output.mat` in `results/`. `plot_results` is invoked automatically to export the standard PDF figures.

Prerequisites: MATLAB R2023a or newer with the Signal Processing and Navigation Toolboxes.

### MATLAB+Python hybrid pipeline
```matlab
run_all_datasets_with_python
```
This wrapper simply invokes `src/run_all_datasets.py` using the active Python
interpreter. The Python script handles both the batch processing and PDF figure
generation, so MATLAB only launches the process. Install Python 3.x along with
`numpy`, `pandas`, `scipy` and `matplotlib` (or simply `pip install -r
requirements.txt`).

### Export to MATLAB (.mat) files
To convert the saved Python results into MATLAB `.mat` files, run from the repo root:
```bash
cd /path/to/IMU
pip install numpy scipy
python export_mat.py
```
Or from *any* directory by giving the full path:
```bash
pip install numpy scipy
python /path/to/IMU/export_mat.py
```

