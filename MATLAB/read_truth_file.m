function truth = read_truth_file(filename)
%READ_TRUTH_FILE Load STATE_X truth file and provide multi-frame states.
%   TRUTH = READ_TRUTH_FILE(FILENAME) reads the ground truth text file
%   ``STATE_X*.txt`` which contains ECEF position, velocity and quaternions.
%   The returned struct contains fields for the time vector and the
%   position/velocity/acceleration expressed in the ECEF, NED and Body
%   reference frames.  Acceleration is obtained by differentiating the
%   velocity series with forward differences.

    if nargin < 1
        error('read_truth_file:MissingFile','Filename required');
    end

    data = read_state_file(filename);
    if isempty(data)
        error('read_truth_file:NoData','Truth file %s contained no data', filename);
    end

    t         = data(:,2);
    pos_ecef  = data(:,3:5);
    vel_ecef  = data(:,6:8);
    quat_e2b  = data(:,9:12); % quaternions (wxyz) ECEF->Body

    % Estimate acceleration in ECEF via numerical differentiation
    dt = diff(t);
    acc_ecef = [zeros(1,3); diff(vel_ecef)./dt];

    % Reference for NED transformation: first sample as origin
    r0 = pos_ecef(1,:);  % origin
    [lat_deg, lon_deg, ~] = ecef_to_geodetic(r0(1), r0(2), r0(3));
    lat = deg2rad(lat_deg); lon = deg2rad(lon_deg);
    C_e2n = compute_C_ECEF_to_NED(lat, lon);
    pos_ned = (C_e2n * (pos_ecef' - r0')).';
    vel_ned = (C_e2n * vel_ecef').';
    acc_ned = (C_e2n * acc_ecef').';

    % Body frame using provided quaternions (assumed ECEF->Body)
    R_e2b = attitude_tools('quat_to_dcm_batch', quat_e2b'); % 3x3xN
    R_n2e = C_e2n.';
    N = size(pos_ecef,1);
    pos_body = zeros(N,3); vel_body = zeros(N,3); acc_body = zeros(N,3);
    for k = 1:N
        R_b_n = R_e2b(:,:,k) * R_n2e;
        pos_body(k,:) = (R_b_n * pos_ned(k,:).').';
        vel_body(k,:) = (R_b_n * vel_ned(k,:).').';
        acc_body(k,:) = (R_b_n * acc_ned(k,:).').';
    end

    truth.time = t;
    truth.ecef.pos = pos_ecef;
    truth.ecef.vel = vel_ecef;
    truth.ecef.acc = acc_ecef;
    truth.ned.pos  = pos_ned;
    truth.ned.vel  = vel_ned;
    truth.ned.acc  = acc_ned;
    truth.body.pos = pos_body;
    truth.body.vel = vel_body;
    truth.body.acc = acc_body;
    truth.lat_ref  = lat;
    truth.lon_ref  = lon;
    truth.r0_ecef  = r0;
end
