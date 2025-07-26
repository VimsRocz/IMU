function run_triad_only
%RUN_TRIAD_ONLY  Execute TRIAD attitude estimation for all dataset pairs.
%   Mirrors the behaviour of ``src/run_triad_only.py`` but implemented in
%   MATLAB. Each IMU/GNSS pair is processed using Task_1 and the results
%   are saved under ``results/<pair>_TRIAD``.
%
%   Usage:
%       run_triad_only
%
%   See also: run_all_datasets_matlab, Task_1

    run_all_datasets_matlab('TRIAD');
end
