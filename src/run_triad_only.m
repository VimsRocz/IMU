function run_triad_only
%RUN_TRIAD_ONLY  Process all datasets using the TRIAD method only
%   This mirrors the behaviour of ``src/run_triad_only.py`` but runs the
%   simplified MATLAB Task 1 pipeline.  All IMU/GNSS pairs bundled with the
%   repository are processed and the initial attitude for each pair is saved
%   under ``results/<IMU>_<GNSS>_TRIAD/``.
%
% Usage:
%   run_triad_only
%
% See also: run_all_datasets_matlab, Task_1

    run_all_datasets_matlab('TRIAD');
end
