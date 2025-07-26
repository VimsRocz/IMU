function save_attitude_output(out_dir, R_bn, q_bn)
%SAVE_ATTITUDE_OUTPUT  Save rotation matrix and quaternion.
%   SAVE_ATTITUDE_OUTPUT(OUT_DIR, R_BN, Q_BN) writes ``initial_attitude.mat``
%   containing the rotation matrix ``R_bn`` and quaternion ``q_bn``.

    if ~exist(out_dir,'dir'); mkdir(out_dir); end
    save(fullfile(out_dir, 'initial_attitude.mat'), 'R_bn', 'q_bn');
end
