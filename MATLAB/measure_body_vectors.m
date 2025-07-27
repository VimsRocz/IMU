function [dt, g_body, omega_ie_body, mag_body] = measure_body_vectors(imu_file, static_start, static_end, mag_file)
%MEASURE_BODY_VECTORS  Stub corresponding to the Python implementation.
%   [DT, G_BODY, OMEGA_IE_BODY, MAG_BODY] = MEASURE_BODY_VECTORS(IMU_FILE,
%   STATIC_START, STATIC_END, MAG_FILE) mirrors the interface of the Python
%   function of the same name. It will estimate the gravity and Earth
%   rotation vectors in the body frame from IMU data and optionally use
%   magnetometer measurements. The full MATLAB implementation is pending.
%
%   This stub is provided for API parity only. All outputs are empty.
%
%   See also: MEASURE_BODY_VECTORS in Python.

if nargin < 4
    mag_file = [];
end

warning('measure_body_vectors:NotImplemented', ...
    'measure_body_vectors.m is a stub. Full implementation pending.');

dt = [];
g_body = [];
omega_ie_body = [];
mag_body = [];
end
