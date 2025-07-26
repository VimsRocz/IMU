function save_attitude_output(out_dir, R_bn, q_bn)
%SAVE_ATTITUDE_OUTPUT Save rotation matrix and quaternion.
%   SAVE_ATTITUDE_OUTPUT(DIR, R_BN, Q_BN) writes initial_attitude.mat to
%   DIR containing R_bn and q_bn.

    out_file = fullfile(out_dir, 'initial_attitude.mat');
    save(out_file, 'R_bn', 'q_bn');
end
