function Task_1()
    % TASK 1: Define reference vectors in NED frame
    fprintf("\nTASK 1: Define reference vectors in NED frame\n");

    %% Subtask 1.1: Setting initial latitude and longitude from GNSS ECEF data
    gnss = readmatrix(get_data_file('GNSS_X001.csv'));
    col_x = 11; col_y = 12; col_z = 13; % ECEF columns
    x_ecef = gnss(:,col_x); y_ecef = gnss(:,col_y); z_ecef = gnss(:,col_z);
    X0 = x_ecef(1); Y0 = y_ecef(1); Z0 = z_ecef(1);
    [lat, lon, h] = ecef2geodetic_custom(X0, Y0, Z0);
    fprintf('Initial latitude: %.6f\u00B0, Initial longitude: %.6f\u00B0\n', lat, lon);

    %% Subtask 1.2: Defining gravity vector in NED frame
    g_NED = [0;0;9.81];
    fprintf('Gravity vector in NED: [%0.2f %0.2f %0.2f] m/s^2\n', g_NED);

    %% Subtask 1.3: Defining Earth rotation rate vector in NED frame
    w_ie = 7.292115e-5; % rad/s
    lat_rad = deg2rad(lat);
    omega_NED = [w_ie * cos(lat_rad); 0; w_ie * sin(lat_rad)];
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

function [lat, lon, h] = ecef2geodetic_custom(X, Y, Z)
    a = 6378137.0; e = 8.1819190842622e-2;
    b = sqrt(a^2 * (1 - e^2));
    ep = sqrt((a^2 - b^2) / b^2);
    p = sqrt(X^2 + Y^2);
    th = atan2(a*Z, b*p);
    lon = atan2(Y, X);
    lat = atan2(Z + ep^2*b*sin(th).^3, p - e^2*a*cos(th).^3);
    N = a ./ sqrt(1 - e^2 .* sin(lat).^2);
    h = p ./ cos(lat) - N;
    lat = rad2deg(lat); lon = rad2deg(lon);
end
