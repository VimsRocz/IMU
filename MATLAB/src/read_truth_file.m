function truth = read_truth_file(filename)
%READ_TRUTH_FILE Load STATE_X truth file and convert to NED.
%   TRUTH = READ_TRUTH_FILE(FILENAME) reads the ground truth text file
%   ``STATE_X*.txt`` which contains ECEF position and velocity along with
%   quaternions. The output structure has fields:
%       time     - Nx1 time vector [s]
%       pos_ned  - Nx3 position in NED [m]
%       vel_ned  - Nx3 velocity in NED [m/s]
%
%   The NED frame uses the first sample as the origin.

    if nargin < 1
        error('read_truth_file:MissingFile','Filename required');
    end
    data = read_state_file(filename);
    t = data(:,2);
    pos_ecef = data(:,3:5);
    vel_ecef = data(:,6:8);
    r0 = pos_ecef(1,:);  % origin
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(r0(1), r0(2), r0(3));
    C = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
    pos_ned = (C * (pos_ecef' - r0')).';
    vel_ned = (C * vel_ecef').';
    truth.time = t;
    truth.pos_ned = pos_ned;
    truth.vel_ned = vel_ned;
end
