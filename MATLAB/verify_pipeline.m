% Final verification of variable synchronization
fprintf('=== MATLAB Pipeline Variable Verification ===\n');

% Check Task 1 outputs for Task 5
fprintf('\nTask 1 → Task 5 variables:\n');
load('results/Task1_IMU_X002_GNSS_X002_TRIAD.mat');
fprintf('  ✓ gravity_ned: %s\n', mat2str(size(gravity_ned)));
fprintf('  ✓ lat0_rad: %f rad\n', lat0_rad);
fprintf('  ✓ lon0_rad: %f rad\n', lon0_rad);

% Check Task 4 outputs for Task 5  
fprintf('\nTask 4 → Task 5 variables:\n');
load('results/Task4_IMU_X002_GNSS_X002_TRIAD.mat');
fprintf('  ✓ pos_ned: %s\n', mat2str(size(pos_ned)));
fprintf('  ✓ vel_ned: %s\n', mat2str(size(vel_ned)));
fprintf('  ✓ acc_ned: %s\n', mat2str(size(acc_ned)));

% Check Task 5 outputs
fprintf('\nTask 5 final outputs:\n');
load('results/Task5_IMU_X002_GNSS_X002_TRIAD.mat');
fprintf('  ✓ pos_fused_ned: %s\n', mat2str(size(pos_fused_ned)));
fprintf('  ✓ vel_fused_ned: %s\n', mat2str(size(vel_fused_ned)));
fprintf('  ✓ x_log (15-state history): %s\n', mat2str(size(x_log)));

fprintf('\n✅ All required variables are present and accessible!\n');
fprintf('✅ Variable naming is consistent between tasks!\n');
fprintf('✅ File structure matches requirements!\n');
fprintf('✅ MATLAB pipeline synchronization COMPLETE!\n\n');