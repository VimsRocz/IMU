# Fix for run_triad_only.m Function Errors

## Problem Resolved

The `run_triad_only.m` script was failing with the error:
```
Unrecognized function or variable 'set_debug'.
Error in run_triad_only (line 9)
```

## Root Cause

The issue was in the order of operations in `run_triad_only.m`:
- Line 8: `addpath(genpath(fullfile(pwd,'MATLAB','src','utils')));`
- Line 9: `set_debug(strcmpi(getenv('DEBUG'),'1') || strcmpi(getenv('DEBUG'),'true'));`

The `set_debug` function was being called BEFORE the utility paths were added, so the function wasn't available.

## Fix Applied

1. **Reordered path setup**: Moved `addpath` calls before any function calls
2. **Added both utility directories**: Ensured both `MATLAB/src/utils` and `MATLAB/utils` are in the path
3. **Added compatibility functions**: Created `strings_compat.m` and `readmatrix_compat.m` for MATLAB/Octave compatibility

## Files Modified

- `MATLAB/run_triad_only.m` - Fixed path setup order
- `MATLAB/src/utils/strings_compat.m` - New compatibility function
- `MATLAB/src/utils/readmatrix_compat.m` - New compatibility function
- Updated timeline functions to use compatibility functions

## How to Use

The script can now be run successfully:

```matlab
% Basic usage with defaults
run_triad_only();

% With custom configuration
run_triad_only(struct('dataset_id','X002'));

% With specific dataset
cfg = struct();
cfg.dataset_id = 'X001';
cfg.imu_file = 'IMU_X001.dat';
cfg.gnss_file = 'GNSS_X001.csv';
run_triad_only(cfg);
```

## Verification

The fix has been verified to resolve the original error. All utility functions (`set_debug`, `log_msg`, `project_paths`, `run_id`, `ensure_input_file`) are now properly accessible.

Additional MATLAB-specific functions may need compatibility layers for full execution in Octave environments, but the core path setup issue has been completely resolved.