function validate_all_methods()
%VALIDATE_ALL_METHODS Validate KF output of all init methods.
%   Loads each *_kf_output.mat result, compares against STATE_X001.txt and
%   checks that the estimation error stays within \pm3 sigma bounds.
%   Generates plots and prints a summary table.

methods = {'TRIAD','Davenport','SVD'};
truth = load('STATE_X001.txt');
ref_r0 = truth(1,3:5)';
[lat_deg, lon_deg, ~] = ecef2geodetic(ref_r0(1), ref_r0(2), ref_r0(3));
C_ECEF_to_NED = compute_C_ECEF_to_NED(deg2rad(lat_deg), deg2rad(lon_deg));
truth_ned_pos = (C_ECEF_to_NED*(truth(:,3:5)' - ref_r0))';
truth_ned_vel = (C_ECEF_to_NED*truth(:,6:8)')';
truth_ned_acc = [zeros(1,3); diff(truth_ned_vel)];
truth_ecef_pos = truth(:,3:5);
truth_ecef_vel = truth(:,6:8);
truth_ecef_acc = [zeros(1,3); diff(truth_ecef_vel)];
resultsDir = 'results';

summary = cell(numel(methods),4);

for i = 1:numel(methods)
    method = methods{i};
    kfFile = fullfile(resultsDir, sprintf('IMU_X001_GNSS_X001_%s_kf_output.mat', method));
    if ~exist(kfFile,'file')
        warning('File not found: %s', kfFile);
        summary(i,:) = {method,false,false,false};
        continue
    end
    S = load(kfFile);

    % extract states
    pos  = S.pos_ned;
    vel  = S.vel_ned;
    quat = S.quat_log;
    P    = S.P;

    % compute errors
    err_pos  = pos  - truth(:,1:3);
    err_vel  = vel  - truth(:,4:6);
    err_quat = quat - truth(:,7:10);

    % compute 3-sigma envelopes
    sigma_pos  = 3*sqrt(squeeze(P(:,1:3,1:3)));
    sigma_vel  = 3*sqrt(squeeze(P(:,4:6,4:6)));
    sigma_quat = 3*sqrt(squeeze(P(:,7:10,7:10)));

    % plot position errors
    labels = {'North','East','Down'};
    h = figure('Visible','off');
    for j = 1:3
        subplot(3,1,j); hold on
        plot(err_pos(:,j),'b','DisplayName','error');
        plot( sigma_pos(:,j),'r--','DisplayName','+3\sigma');
        plot(-sigma_pos(:,j),'r--','HandleVisibility','off');
        ylabel(labels{j});
        if j==1
            title([method ' Position Error']);
        end
        if j==3
            xlabel('Sample');
        end
        grid on
    end
    legend('show');
    saveas(h, fullfile(resultsDir, [method '_pos_3sigma.png']));

    % plot velocity errors
    h = figure('Visible','off');
    for j = 1:3
        subplot(3,1,j); hold on
        plot(err_vel(:,j),'b','DisplayName','error');
        plot( sigma_vel(:,j),'r--','DisplayName','+3\sigma');
        plot(-sigma_vel(:,j),'r--','HandleVisibility','off');
        ylabel(labels{j});
        if j==1
            title([method ' Velocity Error']);
        end
        if j==3
            xlabel('Sample');
        end
        grid on
    end
    legend('show');
    saveas(h, fullfile(resultsDir, [method '_vel_3sigma.png']));

    % plot quaternion errors (4 subplots)
    qlabels = {'q0','q1','q2','q3'};
    h = figure('Visible','off');
    for j = 1:4
        subplot(4,1,j); hold on
        plot(err_quat(:,j),'b','DisplayName','error');
        plot( sigma_quat(:,j),'r--','DisplayName','+3\sigma');
        plot(-sigma_quat(:,j),'r--','HandleVisibility','off');
        ylabel(qlabels{j});
        if j==1
            title([method ' Quaternion Error']);
        end
        if j==4
            xlabel('Sample');
        end
        grid on
    end
    legend('show');
    saveas(h, fullfile(resultsDir, [method '_quat_3sigma.png']));

    % save errors and sigma for later review
    save(fullfile(resultsDir, [method '_validate.mat']), 'err_pos','err_vel','err_quat', 'sigma_pos','sigma_vel','sigma_quat');

    % check bounds
    viol_pos  = sum(any(abs(err_pos)  > sigma_pos,  2));
    viol_vel  = sum(any(abs(err_vel)  > sigma_vel,  2));
    viol_quat = sum(any(abs(err_quat) > sigma_quat, 2));
    if viol_pos>0
        warning('%s pos error exceeded 3\sigma at %d samples', method, viol_pos);
    end
    if viol_vel>0
        warning('%s vel error exceeded 3\sigma at %d samples', method, viol_vel);
    end
    if viol_quat>0
        warning('%s quat error exceeded 3\sigma at %d samples', method, viol_quat);
    end

    pass_pos  = viol_pos  == 0;
    pass_vel  = viol_vel  == 0;
    pass_quat = viol_quat == 0;
    summary(i,:) = {method, pass_pos, pass_vel, pass_quat};

    if isfield(S,'gnss_pos_ned') && isfield(S,'vel_ned')
        t = 1:size(S.vel_ned,1);
        acc_est = diff([zeros(1,3); vel]);
        acc_gnss = diff([zeros(1,3); S.gnss_vel_ned]);
        plot_overlay(t, pos, vel, acc_est, t, S.gnss_pos_ned, S.gnss_vel_ned, acc_gnss, t, pos, vel, acc_est, 'NED', method, resultsDir);

        n = min(length(t), size(truth_ned_pos,1));
        plot_overlay_with_truth(t(1:n), pos(1:n,:), vel(1:n,:), acc_est(1:n,:), ...
            t(1:n), S.gnss_pos_ned(1:n,:), S.gnss_vel_ned(1:n,:), acc_gnss(1:n,:), ...
            t(1:n), truth_ned_pos(1:n,:), truth_ned_vel(1:n,:), truth_ned_acc(1:n,:), ...
            'NED', method, resultsDir);

        % ECEF frame overlay
        C_N2E = C_ECEF_to_NED';
        pos_ecef = (C_N2E*pos' + ref_r0)';
        vel_ecef = (C_N2E*vel')';
        acc_ecef = diff([zeros(1,3); vel_ecef]);
        pos_gnss_ecef = (C_N2E*S.gnss_pos_ned' + ref_r0)';
        vel_gnss_ecef = (C_N2E*S.gnss_vel_ned')';
        acc_gnss_ecef = diff([zeros(1,3); vel_gnss_ecef]);
        plot_overlay_with_truth(t(1:n), pos_ecef(1:n,:), vel_ecef(1:n,:), acc_ecef(1:n,:), ...
            t(1:n), pos_gnss_ecef(1:n,:), vel_gnss_ecef(1:n,:), acc_gnss_ecef(1:n,:), ...
            t(1:n), truth_ecef_pos(1:n,:), truth_ecef_vel(1:n,:), truth_ecef_acc(1:n,:), ...
            'ECEF', method, resultsDir);
    end
end

% display summary table
T = cell2table(summary, 'VariableNames',{'Method','PosPass','VelPass','QuatPass'});
disp(T);

end
