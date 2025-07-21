function diagnose_velocity(est_file, truth_file, imu_file, gnss_file, frame, output_dir)
%DIAGNOSE_VELOCITY  Stub for velocity diagnostics in MATLAB.
%
%   diagnose_velocity(EST_FILE, TRUTH_FILE, IMU_FILE, GNSS_FILE, FRAME, OUTPUT_DIR)
%   is the MATLAB counterpart of the Python diagnose_velocity script.
%   It is currently a placeholder and not implemented.
%
%   The Python implementation trims quaternion and time vectors to the
%   shortest length when they differ, prints velocity residual
%   statistics, checks accelerometer data after gravity removal in the
%   navigation frame, and compares fused versus GNSS velocity at
%   several time points.  See diagnose_velocity.py for the full logic.
%
%See also: DIAGNOSE_VELOCITY.PY

if nargin < 6
    output_dir = 'results';
end

warning('diagnose_velocity:NotImplemented', ...
    'diagnose_velocity.m is a stub corresponding to the Python implementation.');
end
