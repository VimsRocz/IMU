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
imu_path  = '../IMU_X001.dat';
gnss_path = '../GNSS_X001.csv';
TRIAD(imu_path, gnss_path);
```

Scripts such as `TRIAD.m` take the full paths to the IMU and GNSS files as arguments. Use relative paths from the repository root (e.g. `../IMU_X001.dat`) when calling the MATLAB functions.

### MATLAB Scripts

All MATLAB code now lives in the single `MATLAB/` directory. It contains the pipeline tasks (`Task_1`–`Task_5`) alongside helper scripts such as `TRIAD.m`, `FINAL.m`, `plot_results.m` and `validate_3sigma.m`.

To run `TRIAD.m` simply pass the data file names relative to the repository root:
```matlab
TRIAD('../IMU_X001.dat', '../GNSS_X001.csv');
```
The MATLAB results are written to `MATLAB/results/` while the Python pipeline writes to `results/` in the project root.

### Sequential Task Execution

To replicate the Python pipeline step by step, call each `Task_1`–`Task_5` function in order using root-relative paths. Every task writes a `.mat` file that the next one loads:
```matlab
imu  = '../IMU_X001.dat';
gnss = '../GNSS_X001.csv';

Task_1(imu, gnss, 'TRIAD');   % -> results/Task1_init_IMU_X001_GNSS_X001_TRIAD.mat
Task_2(imu, gnss, 'TRIAD');   % -> results/Task2_body_IMU_X001_GNSS_X001_TRIAD.mat
Task_3(imu, gnss, 'TRIAD');   % uses Task1/2 output, writes results/Task3_results_IMU_X001_GNSS_X001.mat
Task_4(imu, gnss, 'TRIAD');   % uses Task3 results, writes results/Task4_results_IMU_X001_GNSS_X001.mat
Task_5(imu, gnss, 'TRIAD');   % uses Task4 results, writes results/IMU_X001_GNSS_X001_TRIAD_task5_results.mat
```

### MATLAB-only pipeline
```matlab
run_all_datasets_matlab('TRIAD')
```
The script `MATLAB/run_all_datasets_matlab.m` accepts the initialisation method as an optional argument. It scans `Data/` (or the repository root if that folder is missing) for matching IMU and GNSS logs, executes `Task_1` through `Task_5` for each pair and saves the results as `IMU_<id>_GNSS_<id>_<METHOD>_kf_output.mat` in `results/`. `plot_results` is invoked automatically to export the standard PNG figures.

Prerequisites: MATLAB R2023a or newer. The **Signal Processing Toolbox** is strongly
recommended for the MATLAB pipeline. If the toolbox is missing, Task&nbsp;2 falls back to
a moving-average filter and manual variance computation, which may reduce accuracy and
increase runtime. Install the toolbox via the MATLAB **Add-On Manager**
(Home &rarr; Add-Ons &rarr; Get Add-Ons) and verify the installation with
```matlab
ver('signal')
```
The Navigation Toolbox remains optional but is helpful for some helper scripts.

### MATLAB+Python hybrid pipeline
```matlab
run_all_datasets_with_python
```
This wrapper simply invokes `src/run_all_datasets.py` using the active Python
interpreter. The Python script handles both the batch processing and PNG figure
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

