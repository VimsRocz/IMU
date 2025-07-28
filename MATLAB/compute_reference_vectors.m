function [lat_deg, lon_deg, alt, g_ned, omega_ie_ned, mag_ned, initial_vel_ned, pos_ecef] = compute_reference_vectors(gnss_file, mag_file)
%COMPUTE_REFERENCE_VECTORS  Return NED reference vectors from GNSS data.
%   This function mirrors ``compute_reference_vectors`` used by the Python
%   pipeline.  It reads the first valid ECEF coordinates from GNSS_FILE,
%   converts them to geodetic latitude and longitude and computes the
%   gravity and Earth rotation vectors in the NED frame.  If MAG_FILE is
%   provided the mean magnetometer vector is returned as MAG_NED.
%
%   [LAT_DEG, LON_DEG, ALT, G_NED, OMEGA_IE_NED, MAG_NED, INITIAL_VEL_NED, POS_ECEF]
%   = COMPUTE_REFERENCE_VECTORS(GNSS_FILE, MAG_FILE)
%
%   Inputs
%   ------
%   GNSS_FILE : string
%       Path to the GNSS CSV file containing ECEF coordinates in the columns
%       ``X_ECEF_m``, ``Y_ECEF_m`` and ``Z_ECEF_m`` as well as ECEF
%       velocities ``VX_ECEF_mps``, ``VY_ECEF_mps`` and ``VZ_ECEF_mps``.
%   MAG_FILE  : string, optional
%       Optional CSV file with magnetometer measurements.  When supplied, the
%       mean of the first three columns is returned in MAG_NED.
%
%   Outputs mirror the Python implementation and use column vectors.

if nargin < 2 || isempty(mag_file)
    mag_file = '';
end

T = read_csv_table(gnss_file);

valid_idx = find(T.X_ECEF_m ~= 0 | T.Y_ECEF_m ~= 0 | T.Z_ECEF_m ~= 0, 1, 'first');
if isempty(valid_idx)
    error('compute_reference_vectors:NoValidRows', ...
          'No valid ECEF coordinates found in %s', gnss_file);
end

x_ecef = T.X_ECEF_m(valid_idx);
y_ecef = T.Y_ECEF_m(valid_idx);
z_ecef = T.Z_ECEF_m(valid_idx);

[lat_deg, lon_deg, alt] = ecef_to_geodetic(x_ecef, y_ecef, z_ecef);
lat_rad = deg2rad(lat_deg);
lon_rad = deg2rad(lon_deg);

% Gravity vector in NED using the same helper as Python
g_ned = [0; 0; constants.GRAVITY];

% Earth rotation rate vector in NED frame
omega_ie_ned = constants.EARTH_RATE * [cos(lat_rad); 0; -sin(lat_rad)];

% Initial velocity in NED from the first valid GNSS row
vel_ecef = [T.VX_ECEF_mps(valid_idx); T.VY_ECEF_mps(valid_idx); T.VZ_ECEF_mps(valid_idx)];
C_e2n = compute_C_ECEF_to_NED(lat_rad, lon_rad);
initial_vel_ned = C_e2n * vel_ecef;

% Position of the first sample in ECEF for later use
pos_ecef = [x_ecef; y_ecef; z_ecef];

mag_ned = [];
if ~isempty(mag_file) && isfile(mag_file)
    try
        mag_data = readmatrix(mag_file);
        if ismatrix(mag_data) && size(mag_data,2) >= 3
            mag_ned = mean(mag_data(:,1:3), 1)';
        end
    catch ME %#ok<NASGU>
        % Ignore magnetometer failures just like the Python code
        mag_ned = [];
    end
end

end
