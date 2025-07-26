function save_attitude_output(out_dir, R_bn, q_bn)
%SAVE_ATTITUDE_OUTPUT  Store rotation matrix and quaternion.
%   SAVE_ATTITUDE_OUTPUT(OUT_DIR, R_BN, Q_BN) writes ``initial_attitude.mat``
%   to OUT_DIR containing ``R_bn`` and ``q_bn``.

    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    file = fullfile(out_dir, 'initial_attitude.mat');
    R_bn = double(R_bn); %#ok<NASGU>
    q_bn = double(q_bn); %#ok<NASGU>
    save(file, 'R_bn', 'q_bn');
end
