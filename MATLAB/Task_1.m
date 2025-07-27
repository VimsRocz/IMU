function result = Task_1(imu_path, gnss_path, method)
% TASK 1: Define Reference Vectors in NED Frame
% This function translates Task 1 from the Python file GNSS_IMU_Fusion.py
% into MATLAB.
%
% Usage:
%   Task_1(imu_path, gnss_path, method)

if nargin < 3 || isempty(method)
    method = '';
end

if ~isfile(gnss_path)
    error('Task_1:GNSSFileNotFound', ...
          'Could not find GNSS data at:\n  %s\nCheck path or filename.', ...
          gnss_path);
end
if ~isfile(imu_path)
    error('Task_1:IMUFileNotFound', ...
          'Could not find IMU data at:\n  %s\nCheck path or filename.', ...
          imu_path);
end

% Remove command-line side effects to behave like a normal function

results_dir = get_results_dir();
if ~exist(results_dir,'dir')
    mkdir(results_dir);
end

% Print dataset and method like the Python script
[~, imu_name, ~] = fileparts(imu_path);
[~, gnss_name, ~] = fileparts(gnss_path);
if isempty(method)
    tag = [imu_name '_' gnss_name];
else
    tag = [imu_name '_' gnss_name '_' method];
end

fprintf('%s %s\n', char(hex2dec('25B6')), tag); % \u25B6 is the triangle symbol
fprintf('Ensured ''output_matlab/'' directory exists.\n');
if ~isempty(method)
    fprintf('Running attitude-estimation method: %s\n', method);
end

if isempty(method)
    log_tag = '';
else
    log_tag = [' (' method ')'];
end
fprintf('TASK 1%s: Define reference vectors in NED frame\n', log_tag);

% --- Configuration ---


% ================================
% Subtask 1.1: Set Initial Latitude and Longitude from GNSS ECEF Data
% ================================
fprintf('\nSubtask 1.1: Setting initial latitude and longitude from GNSS ECEF data.\n');

% Load GNSS data using readtable
try
    gnss_data = readtable(gnss_path);
catch e
    error('Failed to load GNSS data file: %s\n%s', gnss_path, e.message);
end

% Display column names and first few rows for debugging
disp('GNSS data columns:');
disp(gnss_data.Properties.VariableNames');
disp('First few rows of ECEF coordinates:');
disp(head(gnss_data(:, {'X_ECEF_m', 'Y_ECEF_m', 'Z_ECEF_m'})));

% Find the first row with non-zero ECEF coordinates
valid_idx = find((gnss_data.X_ECEF_m ~= 0) | ...
                 (gnss_data.Y_ECEF_m ~= 0) | ...
                 (gnss_data.Z_ECEF_m ~= 0), 1, 'first');

if ~isempty(valid_idx)
    initial_row = gnss_data(valid_idx, :);
    x_ecef = initial_row.X_ECEF_m;
    y_ecef = initial_row.Y_ECEF_m;
    z_ecef = initial_row.Z_ECEF_m;
    
    % Convert ECEF to geodetic coordinates using the shared helper
    [lat_deg, lon_deg, alt_m] = ecef2geodetic(x_ecef, y_ecef, z_ecef);
    
    % Convert degrees to radians for calculations
    lat = deg2rad(lat_deg);
    
    fprintf('Computed initial latitude: %.6f°, longitude: %.6f° from GNSS\n', lat_deg, lon_deg);
else
    error('No valid ECEF coordinates found in GNSS data.');
end


% ================================
% Subtask 1.2: Define Gravity Vector in NED
% ================================
fprintf('\nSubtask 1.2: Defining gravity vector in NED frame.\n');

% Compute gravity magnitude using WGS-84 model and print validation line
g_NED = validate_gravity_vector(lat_deg, alt_m);
% Override with the gravity magnitude used in the Python pipeline
g_NED = [0; 0; constants.GRAVITY];


% ================================
% Subtask 1.3: Define Earth Rotation Rate Vector in NED
% ================================
fprintf('\nSubtask 1.3: Defining Earth rotation rate vector in NED frame.\n');

% Earth rotation rate in rad/s
omega_E = constants.EARTH_RATE;

% Earth rotation rate vector in NED frame: ω_ie,NED = ω_E * [cos(φ); 0; -sin(φ)]
omega_ie_NED = omega_E * [cos(lat); 0; -sin(lat)];

fprintf('Earth rotation rate (NED):   [%.8e %.8e %.8e] rad/s\n', omega_ie_NED(1), omega_ie_NED(2), omega_ie_NED(3));


% ================================
% Subtask 1.4: Validate and Print Reference Vectors
% ================================
fprintf('\nSubtask 1.4: Validating reference vectors.\n');

% Validate vector shapes and components (MATLAB uses column vectors by convention)
assert(isequal(size(g_NED), [3, 1]), 'Gravity vector must be a 3x1 vector.');
assert(isequal(size(omega_ie_NED), [3, 1]), 'Earth rotation rate vector must be a 3x1 vector.');
assert(abs(g_NED(1)) < 1e-9 && abs(g_NED(2)) < 1e-9, 'Gravity should have no North/East component.');
assert(abs(omega_ie_NED(2)) < 1e-9, 'Earth rate should have no East component in NED.');

fprintf('Reference vectors validated successfully.\n');

% Print reference vectors
fprintf('\n==== Reference Vectors in NED Frame ====\n');
fprintf('Gravity vector (NED):        [%.8f %.8f %.8f] m/s^2\n', g_NED);
fprintf('Earth rotation rate (NED):   [%.8e %.8e %.8e] rad/s\n', omega_ie_NED);
fprintf('Latitude (deg):              %.6f\n', lat_deg);
fprintf('Longitude (deg):             %.6f\n', lon_deg);


% ================================
% Subtask 1.5: Plot Location on Earth Map
% ================================
fprintf('\nSubtask 1.5: Plotting location on Earth map.\n');

% Create a geographic plot if Mapping Toolbox is available
if exist('geoplot', 'file') == 2 && license('test', 'map_toolbox')
    figure('Name', 'Initial Location on Earth Map', 'Position', [100, 100, 1000, 500]);
    geobasemap satellite; % Use satellite imagery as the basemap

    % Set map limits to focus on the location
    geolimits([lat_deg - 2, lat_deg + 2], [lon_deg - 2, lon_deg + 2]);

    % Plot the initial location with a red marker
    hold on;
    geoplot(lat_deg, lon_deg, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Add a text label
    text_str = sprintf('Lat: %.4f°, Lon: %.4f°', lat_deg, lon_deg);
    text(lon_deg + 0.1, lat_deg, text_str, 'Color', 'white', 'FontSize', 12, 'FontWeight', 'bold');
    hold off;

    % Set plot title
    title('Initial Location on Earth Map');

    % Save the plot as both PDF and PNG using a reasonable page size
    set(gcf, 'PaperPositionMode', 'auto');
    base = fullfile(results_dir, sprintf('%s_location_map', tag));
    print(gcf, [base '.pdf'], '-dpdf', '-bestfit');
    print(gcf, [base '.png'], '-dpng');
    fprintf('Location map saved to %s.[pdf|png]\n', base);
else
    warning('Mapping Toolbox not found. Skipping geographic plot.');
end
% close(gcf); % Uncomment to close the figure after saving

% Save results for later tasks

lat = lat_deg; %#ok<NASGU>
lon = lon_deg; %#ok<NASGU>
omega_NED = omega_ie_NED; %#ok<NASGU>
save(fullfile(results_dir, ['Task1_init_' tag '.mat']), 'lat', 'lon', 'g_NED', 'omega_NED');
fprintf('Initial data saved to %s\n', fullfile(results_dir, ['Task1_init_' tag '.mat']));

% Return results and store in base workspace for interactive use
result = struct('lat', lat_deg, 'lon', lon_deg, ...
                'g_NED', g_NED, 'omega_NED', omega_ie_NED);
assignin('base', 'task1_results', result);

end
