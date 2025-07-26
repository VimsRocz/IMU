# IMU+GNSS Initialization Pipeline (MATLAB)

This folder contains a MATLAB translation of the Python pipeline.

## Usage

Run the batch script from the repository root so all data paths resolve
correctly:

```matlab
addpath('MATLAB');         % from repository root
run_all_datasets_matlab;   % all methods
% or
run_all_datasets_matlab('TRIAD');
```

## Structure

```
MATLAB/
    main.m
    Task_1.m   % function
    Task_2.m   % function
    Task_3.m
    Task_4.m
    Task_5.m
    data/
output_matlab/
```

### Installing FilterPy (Ubuntu)

Some helper scripts call the Python Kalman filter via `filterpy`. If the wheel fails
to build, install from source:

```bash
pip install filterpy --no-binary :all:
```

If build tools are missing run:

```bash
sudo apt install build-essential python3-dev
pip install filterpy --no-binary :all:
```

Place your `.dat` and `.csv` data files inside the `data/` folder. If the folder is
missing, the scripts will also look for the files in the repository root. The
scripts save outputs and plots in `output_matlab/`.

Run the entire pipeline from MATLAB by executing `main.m`. The script now
accepts optional file names **and** a list of methods so you can run:

```matlab
main                 % uses all bundled datasets and all methods
main('IMU_X002.dat','GNSS_X002.csv')
main('IMU_X001.dat','GNSS_X001.csv','TRIAD')
main({'IMU_X001.dat','IMU_X002.dat'}, {'GNSS_X001.csv','GNSS_X002.csv'}, {'SVD'})
Task_4('IMU_X001.dat','GNSS_X001.csv')
Task_5('IMU_X001.dat','GNSS_X001.csv', gnss_pos_ned)
```

`main` executes `Task_1`–`Task_5` for each selected attitude initialisation
method (TRIAD, Davenport and SVD by default). Output files include the
method name so results are preserved for every run.

`Task_4` expects the rotation matrices produced by `Task_3` to be saved as
`output_matlab/task3_results.mat`. Make sure `Task_3` completes before running
`Task_4` separately.

`Task_4` also saves the NED-converted GNSS position array `gnss_pos_ned` to
`output_matlab/task4_results.mat`.
`Task_5` looks for this file when started or you can pass `gnss_pos_ned`
directly as an argument.

`Task_1` and `Task_2` are now functions that accept an optional method name,
so you can call them directly as
```matlab
Task_1('IMU_X001.dat','GNSS_X001.csv','TRIAD')
Task_2('IMU_X001.dat','GNSS_X001.csv','TRIAD')
```


### Batch processing
The helper script `run_all_datasets.m` iterates over every `IMU_X*.dat` and `GNSS_X*.csv` pair and runs all three methods. After each run the Task 5 results are loaded into workspace variables such as `result_IMU_X001_GNSS_X001_TRIAD` and saved in `output_matlab/` as `.mat` files.

```matlab
run_all_datasets
```

To run only the TRIAD method use:

```matlab
% process all bundled datasets (paths resolved with get_data_file)
results = TRIAD_batch();

% or run a specific pair
% result = TRIAD_batch('IMU_X001.dat','GNSS_X001.csv');

% multiple pairs can be given as cell arrays
% results = TRIAD_batch({'IMU_X001.dat','IMU_X002.dat'}, {'GNSS_X001.csv','GNSS_X002.csv'});
```

`TRIAD_batch` resolves file names with `get_data_file`, so the bundled logs are
found even if you run the command from another folder.  When more than one
pair is processed the function returns a cell array of result structs, each
matching the corresponding `output_matlab/Result_<IMU>_<GNSS>_TRIAD.mat` file.

Dedicated wrappers `run_triad_only.m`, `run_svd_only.m` and
`run_davenport_only.m` behave like the Python helpers of the same name and
simply forward the chosen method to [`run_all_datasets_matlab.m`](run_all_datasets_matlab.m):

```matlab
run_triad_only      % TRIAD method
run_svd_only        % SVD method
run_davenport_only  % Davenport method
```

### GNSS_IMU_Fusion_single

Use `GNSS_IMU_Fusion_single` when you only need to process one IMU/GNSS pair.
The function mirrors `run_triad_only.py` and generates the same location map,
Task 3/4/5 results, residuals and attitude plots in the `output_matlab/` folder.
After saving the results, `plot_task5_results_all_methods` produces a
summary of the fused GNSS/IMU trajectory for Task 5.

```matlab
GNSS_IMU_Fusion_single('IMU_X001.dat','GNSS_X001.csv')
```

### Overlay comparison with ground truth

Run `python src/validate_with_truth.py` or call the MATLAB helper
`overlay_truth_task4` to overlay the fused trajectory with the
`STATE_X001.txt` reference. Before launching the script read the first row of
the state file to determine the reference ECEF coordinate and convert it to
latitude and longitude. Pass these values via `--ref-lat`, `--ref-lon` and
`--ref-r0` so that all transformations use the same origin:

```bash
python src/validate_with_truth.py --est-file <kf.mat> --truth-file STATE_X001.txt \
    --output results --ref-lat <lat> --ref-lon <lon> --ref-r0 <x> <y> <z>
```

The comparison figures `<method>_<frame>_overlay_truth.pdf` are stored in the
`output_matlab/` directory next to the Kalman filter output.

### Task 6 – Truth Overlay

`Task_6` automates the overlay step entirely in MATLAB. Call it with the
Task 5 result file and the associated data paths:

```matlab
task5 = fullfile('output_matlab','IMU_X001_GNSS_X001_TRIAD_task5_results.mat');
Task_6(task5,'IMU_X001.dat','GNSS_X001.csv','STATE_X001.txt');
```

The function loads `<IMU>_<GNSS>_<METHOD>_kf_output.mat`, reads the matching
`STATE_*.txt` file and interpolates all series to a common time vector. It
then calls `plot_overlay` for the NED, ECEF and body frames, saving the
figures `<METHOD>_NED_overlay_truth.pdf`, `<METHOD>_ECEF_overlay_truth.pdf`
and `<METHOD>_Body_overlay_truth.pdf` in `output_matlab/`.

### Compatibility notes

Some older MATLAB releases expect the ellipsoid name for `ecef2lla` as a
character vector rather than as an ellipsoid object. If you see an error like
```
World model type is not a character vector.
```
ensure you are using the `'WGS84'` string in `Task_1.m` when calling
`ecef2lla`.

## GitHub Usage

1. Clone or open this repository.
2. Add your data files to `MATLAB/data/` (or keep them in the repository
   root).
3. In MATLAB, navigate to `MATLAB/` and run `main`.
4. Commit new scripts or results with:
   ```bash
   git add MATLAB
   git commit -m "Update MATLAB pipeline"
   git push
   ```

## Codex Integration Notes: Gravity, Bias and Metrics

The MATLAB version mirrors the Python pipeline. Use the following steps when
modifying or extending the code so both implementations stay aligned:

1. **Scale Gravity in the Body Frame** – After detecting the static interval in
   `Task_2`, normalise the mean accelerometer vector so its magnitude is exactly
   `9.81` m/s². Save this vector (`g_body_scaled`) and print it.
2. **Accelerometer Bias** – Compute the bias from the static accelerometer data
   using `g_body_scaled`, subtract it from all IMU samples and store the values.
3. **Gyroscope Bias** – Estimate the mean of the static gyroscope samples,
   subtract it from the gyro data and print the bias.
4. **Performance Metrics** – After the filtering step in `Task_5` compute
   position/velocity RMSE and the final position error. Save these in a struct
   called `results` along with the biases.
5. **Result Logging** – Write the struct to
   `output_matlab/IMU_GNSS_bias_and_performance.mat` and optionally append the printed
   summary to `output_matlab/IMU_GNSS_summary.txt`.

Add comments in the code where each step occurs (e.g. `% Task 2.3: Gravity and
Bias`) to help future maintainers keep the MATLAB and Python versions
consistent.
