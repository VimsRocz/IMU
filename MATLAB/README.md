# IMU+GNSS Initialization Pipeline (MATLAB)

This folder contains a MATLAB translation of the Python pipeline.

## Structure

```
MATLAB/
    main.m
    Task_1.m   % function
    Task_2.m   % function
    Task_3.m
    Task_4.m
    Task_5.m
    results/
```

Place your `.dat` and `.csv` data files inside the top-level `Data/` folder.
The scripts look there by default and save outputs and plots in `MATLAB/results/`.
Note that `IMU_X003.dat` is bundled without a matching `GNSS_X003.csv`; use `GNSS_X002.csv` instead.

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
`MATLAB/results/task3_results.mat`. Make sure `Task_3` completes before running
`Task_4` separately.

`Task_4` also saves the NED-converted GNSS position array `gnss_pos_ned` to
`MATLAB/results/task4_results.mat`.
`Task_5` looks for this file when started or you can pass `gnss_pos_ned`
directly as an argument.
If a reference state log such as `STATE_X001.txt` is available,
`Task_5` reads it using
```matlab
readmatrix(path, 'CommentStyle','#')
```
to skip the header comments. Provide the path as the optional
`truthFile` argument when running the task.

`Task_1` and `Task_2` are now functions that accept an optional method name,
so you can call them directly as
```matlab
Task_1('IMU_X001.dat','GNSS_X001.csv','TRIAD')
Task_2('IMU_X001.dat','GNSS_X001.csv','TRIAD')
```


### Batch processing
The helper script `run_all_datasets.m` iterates over every `IMU_X*.dat` and `GNSS_X*.csv` pair and runs all three methods. Paths are resolved with `get_data_file` so the script can be executed from any folder. After each run the Task 5 results are loaded into workspace variables such as `result_IMU_X001_GNSS_X001_TRIAD` and saved in `MATLAB/results/` as `.mat` files.

```matlab
run_all_datasets
```

To reproduce the Python helper `run_triad_only.py`, use `run_triad_only.m`:

```matlab
run_triad_only
```

This MATLAB script calls the Python batch processor with the TRIAD method and
validates the generated MAT files when reference logs are present.

The `TRIAD` function described below is a simplified single-pair demonstration
and **not** the batch runner.

For a simplified single-pair demonstration call `TRIAD` directly:

```matlab
% process all bundled datasets (paths resolved with get_data_file)
results = TRIAD();

% or run a specific pair
% result = TRIAD('IMU_X001.dat','GNSS_X001.csv');

% enable verbose output for step-by-step messages
% result = TRIAD('IMU_X001.dat','GNSS_X001.csv', true);

% multiple pairs can be given as cell arrays
% results = TRIAD({'IMU_X001.dat','IMU_X002.dat'}, {'GNSS_X001.csv','GNSS_X002.csv'});
```

Passing a third `verbose` argument toggles step-by-step output:

```matlab
TRIAD('IMU_X001.dat','GNSS_X001.csv', true)
```

`TRIAD` resolves file names with `get_data_file`, so the bundled logs are
found even if you run the command from another folder.  When more than one
pair is processed the function returns a cell array of result structs, each
matching the corresponding `MATLAB/results/Result_<IMU>_<GNSS>_TRIAD.mat` file.

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
2. Add your data files to the `Data/` folder in the repository root.
3. In MATLAB, navigate to `MATLAB/` and run `main`.
4. Commit new scripts or results with:
   ```bash
   git add MATLAB
   git commit -m "Update MATLAB pipeline"
   git push
   ```

### Regenerating Plots from Saved Results

After running the Python pipeline you can recreate the figures in MATLAB by
loading the saved result file:

```matlab
plot_results('results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat')
```

If `load_npz.m` is on the MATLAB path, `plot_results` also accepts the
NumPy `.npz` archives produced by the Python code:

```matlab
plot_results('results_X001.npz')
```

You can also call `load_npz` directly to access the arrays in a struct.

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
   `MATLAB/results/IMU_GNSS_bias_and_performance.mat` and optionally append the printed
   summary to `MATLAB/results/IMU_GNSS_summary.txt`.

Add comments in the code where each step occurs (e.g. `% Task 2.3: Gravity and
Bias`) to help future maintainers keep the MATLAB and Python versions
consistent.
