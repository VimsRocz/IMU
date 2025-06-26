function refVectors = Task_1(imuFile, gnssFile)
%TASK_1 Initialise reference vectors using GNSS data
%   refVectors = Task_1(imuFile, gnssFile) reads the first GNSS ECEF sample,
%   computes reference gravity and Earth rotation vectors in the NED frame,
%   validates them, and plots the initial location.

    %% Subtask 1.1: Read and display
    fprintf('Subtask 1.1: Reading GNSS ECEF data from %s\n', gnssFile);
    data = readmatrix(gnssFile);
    X = data(:,11); Y = data(:,12); Z = data(:,13);
    disp([X(1:5) Y(1:5) Z(1:5)]);
    lla = ecef2lla([X(1) Y(1) Z(1)]);
    lat = deg2rad(lla(1));
    lon = deg2rad(lla(2));
    fprintf('Computed initial latitude: %.6f\xB0, longitude: %.6f\xB0\n', ...
            rad2deg(lat), rad2deg(lon));

    %% Subtask 1.2: Gravity vector
    fprintf('Subtask 1.2: Defining gravity vector in NED frame.\n');
    g_ned = [0;0;9.81];
    disp(g_ned');

    %% Subtask 1.3: Earth rotation rate
    fprintf('Subtask 1.3: Defining Earth rotation rate vector in NED frame.\n');
    omega_e = 7.292115e-5; % rad/s
    omega_ned = [omega_e*cos(lat); 0; -omega_e*sin(lat)];
    disp(omega_ned');

    %% Subtask 1.4: Validation
    fprintf('Subtask 1.4: Validating reference vectors.\n');
    assert(abs(norm(g_ned) - 9.81) < 1e-3);
    assert(abs(norm(omega_ned) - omega_e) < 1e-7);
    fprintf('Reference vectors validated successfully.\n');

    %% Subtask 1.5: Plot GNSS location
    fprintf('Subtask 1.5: Plotting initial location on Earth map.\n');
    [~,imuBase,~]  = fileparts(imuFile);
    [~,gnssBase,~] = fileparts(gnssFile);
    fig = figure('Visible','off');
    geoscatter(rad2deg(lat), rad2deg(lon), 'r','filled');
    geobasemap('satellite');
    title(sprintf('%s / %s', imuBase, gnssBase));
    outdir = 'matlab_plot';
    if ~exist(outdir,'dir'), mkdir(outdir); end
    outname = sprintf('%s/%s_%s_task1_location_map.pdf', outdir, imuBase, gnssBase);
    saveas(fig, outname);
    close(fig);
    fprintf('Location map saved to %s\n', outname);

    %% Final summary
    fprintf('==== Reference Vectors Summary ====\n');
    fprintf('Latitude  : %.6f deg\n', rad2deg(lat));
    fprintf('Longitude : %.6f deg\n', rad2deg(lon));
    fprintf('g_ned     : [%g %g %g]\n', g_ned);
    fprintf('omega_ned : [%g %g %g]\n', omega_ned);

    refVectors = struct('lat', lat, 'lon', lon, 'g_ned', g_ned, 'omega_ned', omega_ned);
    if ~exist('results','dir'), mkdir('results'); end
    save(fullfile('results','Task1_init.mat'),'lat','lon','g_ned','omega_ned');
end
