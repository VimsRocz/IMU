function Task_1()
    % TASK 1: Define reference vectors in NED frame
    fprintf("\nTASK 1: Define reference vectors in NED frame\n");

    % ensure results directory exists relative to the current folder
    if ~exist('results','dir')
        mkdir('results');
    end

    %% Subtask 1.1: Setting initial latitude and longitude from GNSS ECEF data
    opts = detectImportOptions(get_data_file('GNSS_X001.csv'), 'NumHeaderLines', 0);
    gnss = readtable(get_data_file('GNSS_X001.csv'), opts);
    X0 = gnss.X_ECEF_m(1); Y0 = gnss.Y_ECEF_m(1); Z0 = gnss.Z_ECEF_m(1);
    % use ecef2lla which returns a 3-element vector [lat lon alt]
    % older MATLAB versions may not support the ellipsoid argument with
    % multiple outputs, so extract the components manually
    wgs84 = wgs84Ellipsoid('meter');
    lla = ecef2lla([X0 Y0 Z0], wgs84);
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
    saveas(gcf, fullfile('results','Task1_location_map.png'));
    close;

    save(fullfile('results','Task1_init.mat'), 'lat', 'lon', 'g_NED', 'omega_NED');
end
