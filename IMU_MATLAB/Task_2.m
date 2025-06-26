function Task_2()
    % TASK 2: Measure the vectors in the body frame (static interval)
    fprintf('\nTASK 2: Measure the vectors in the body frame\n');

    data = load(get_data_file('IMU_X001.dat'));
    acc = data(:,6:8);  % velocity increments
    gyro = data(:,3:5); % angular increments
    dt = mean(diff(data(1:100,2)));
    if dt <= 0, dt = 1/400; end
    acc = acc / dt;      % m/s^2
    gyro = gyro / dt;    % rad/s

    % Low-pass filter to match Python implementation
    [b,a] = butter(4, 5/(0.5/dt));
    acc = filtfilt(b,a,acc);
    gyro = filtfilt(b,a,gyro);

    win = 80;  % sliding window length
    accel_var = movvar(acc, [win-1 0], 1);
    gyro_var  = movvar(gyro, [win-1 0], 1);
    max_accel = max(accel_var,[],2);
    max_gyro  = max(gyro_var,[],2);
    static_mask = (max_accel < 0.01) & (max_gyro < 1e-6);
    d = diff([0; static_mask; 0]);
    starts = find(d==1); ends = find(d==-1)-1;
    lens = ends - starts + 1;
    lens(lens < 80) = 0;
    [~,idx] = max(lens);
    start_idx = starts(idx); end_idx = ends(idx);

    static_acc = mean(acc(start_idx:end_idx,:),1);
    static_gyro = mean(gyro(start_idx:end_idx,:),1);

    g_norm = norm(static_acc);
    scale = 9.81 / g_norm;
    acc = acc * scale;
    static_acc = static_acc * scale;

    g_body = -static_acc';
    omega_ie_body = static_gyro';

    fprintf('Static window: %d-%d\n', start_idx, end_idx);
    fprintf('Static accelerometer mean: [% .8f % .8f % .8f]\n', static_acc);
    fprintf('Static gyroscope mean: [% .8f % .8f % .8f]\n', static_gyro);

    save(fullfile('results','Task2_body.mat'),'g_body','omega_ie_body');
end
