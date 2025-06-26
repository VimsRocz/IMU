function init = Task_1(imuFile, gnssFile)
    % TASK 1: Define reference vectors in NED frame
    fprintf("\nTASK 1: Define reference vectors in NED frame\n");

    % ensure results directory exists relative to the current folder
    if ~exist('results','dir')
        mkdir('results');
    end
    [~, imu_name, ~] = fileparts(imuFile);
    [~, gnss_name, ~] = fileparts(gnssFile);
    tag = [imu_name '_' gnss_name];

    %% Subtask 1.1: Setting initial latitude and longitude from GNSS ECEF data
    opts = detectImportOptions(get_data_file(gnssFile), 'NumHeaderLines', 0);
    gnss = readtable(get_data_file(gnssFile), opts);
    X0 = gnss.X_ECEF_m(1); Y0 = gnss.Y_ECEF_m(1); Z0 = gnss.Z_ECEF_m(1);
    % use ecef2lla which returns a 3-element vector [lat lon alt]
    % older MATLAB versions may not support the ellipsoid argument with
    % multiple outputs, so extract the components manually
    % Some MATLAB releases expect the ellipsoid as a character vector rather
    % than an object. Use 'WGS84' to avoid errors from `worldparams`.
    lla = ecef2lla([X0 Y0 Z0], 'WGS84');
    lat = lla(1); lon = lla(2); h = lla(3);
    fprintf('Computed initial latitude: %.6f\u00B0, longitude: %.6f\u00B0\n', lat, lon);

    %% Subtask 1.2: Defining gravity vector in NED frame
    g_NED = [0;0;9.81];
    fprintf('Gravity vector in NED: [%0.2f %0.2f %0.2f] m/s^2\n', g_NED);

    %% Subtask 1.3: Defining Earth rotation rate vector in NED frame
    w_ie = 7.292115e-5; % rad/s
    lat_rad = deg2rad(lat);
    omega_NED = [w_ie * cos(lat_rad); 0; -w_ie * sin(lat_rad)];
    fprintf('Earth rotation rate in NED: [%e %e %e] rad/s\n', omega_NED);

    %% Subtask 1.4: Validating reference vectors
    if abs(norm(g_NED) - 9.81) < 1e-2
        disp('Gravity vector validated.');
    end
    if abs(norm(omega_NED) - w_ie) < 1e-7
        disp('Earth rotation vector validated.');
    end

    %% Subtask 1.5: Plotting location on Earth map
    figure;
    geoscatter(lat, lon, 'r', 'filled');
    title('Initial Location (from GNSS ECEF)');
    geobasemap('satellite');
    saveas(gcf, fullfile('results', ['Task1_location_map_' tag '.png']));
    close;

    init.lat = lat;
    init.lon = lon;
    init.g_NED = g_NED;      % renamed from g_ned
    init.omega_NED = omega_NED;  % renamed from omega_ned
    save(fullfile('results', ['Task1_init_' tag '.mat']), '-struct', 'init');
end
