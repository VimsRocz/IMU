function [lat_deg, lon_deg, alt, g_ned, omega_ie_ned, mag_ned, initial_vel_ned, pos_ecef] = compute_reference_vectors(gnss_file, mag_file)
%COMPUTE_REFERENCE_VECTORS  Stub corresponding to the Python implementation.
%   [LAT_DEG, LON_DEG, ALT, G_NED, OMEGA_IE_NED, MAG_NED, INITIAL_VEL_NED, POS_ECEF] =
%   COMPUTE_REFERENCE_VECTORS(GNSS_FILE, MAG_FILE) mirrors the interface of
%   the Python function of the same name. It will load GNSS data to
%   establish the reference latitude, longitude and altitude, compute the
%   gravity and Earth rotation vectors in the NED frame, and optionally use
%   magnetometer measurements. The full MATLAB implementation is pending.
%
%   This stub is provided for API parity only. All outputs are empty.
%
%   See also: COMPUTE_REFERENCE_VECTORS in Python.

if nargin < 2
    mag_file = [];
end

warning('compute_reference_vectors:NotImplemented', ...
    'compute_reference_vectors.m is a stub. Full implementation pending.');

lat_deg = [];
lon_deg = [];
alt = [];
g_ned = [];
omega_ie_ned = [];
mag_ned = [];
initial_vel_ned = [];
pos_ecef = [];
end
