function Task_2()
    % TASK 2: Measure the vectors in the body frame (static interval)
    fprintf('\nTASK 2: Measure the vectors in the body frame\n');

    data = load(fullfile('data','IMU_X001.dat'));
    acc = data(:,6:8);  % velocity increments
    gyro = data(:,3:5); % angular increments
    dt = mean(diff(data(1:100,2)));
    if dt <= 0, dt = 1/400; end
    acc = acc / dt;      % m/s^2
    gyro = gyro / dt;    % rad/s

    Nstatic = min(4000, size(data,1));
    static_acc = mean(acc(1:Nstatic,:),1);
    static_gyro = mean(gyro(1:Nstatic,:),1);

    g_body = -static_acc';
    omega_ie_body = static_gyro';

    fprintf('Static accelerometer mean: [%g %g %g]\n', static_acc);
    fprintf('Static gyroscope mean: [%g %g %g]\n', static_gyro);

    save(fullfile('results','Task2_body.mat'),'g_body','omega_ie_body');
end
