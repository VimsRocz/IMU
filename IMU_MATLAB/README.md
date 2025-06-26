# IMU+GNSS Initialization Pipeline (MATLAB)

This folder contains a MATLAB translation of the Python pipeline.

## Structure

```
IMU_MATLAB/
    main.m
    Task_1.m
    Task_2.m
    Task_3.m
    Task_4.m
    Task_5.m
    data/
    results/
```

Place your `.dat` and `.csv` data files inside the `data/` folder. If the folder is
missing, the scripts will also look for the files in the repository root. The
scripts save outputs and plots in `results/`.

Run the entire pipeline from MATLAB by executing `main.m`.

## GitHub Usage

1. Clone or open this repository.
2. Add your data files to `IMU_MATLAB/data/` (or keep them in the repository
   root).
3. In MATLAB, navigate to `IMU_MATLAB/` and run `main`.
4. Commit new scripts or results with:
   ```bash
   git add IMU_MATLAB
   git commit -m "Update MATLAB pipeline"
   git push
   ```
