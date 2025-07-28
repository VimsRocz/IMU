function C = compute_C_NED_to_ECEF(lat_rad, lon_rad)
%COMPUTE_C_NED_TO_ECEF Rotation matrix from NED to ECEF.
%   This mirrors the helper in ``src/run_all_methods.py``. It simply
%   returns the transpose of ``compute_C_ECEF_to_NED`` so that Python and
%   MATLAB use identical conventions.

C = compute_C_ECEF_to_NED(lat_rad, lon_rad)';
end
