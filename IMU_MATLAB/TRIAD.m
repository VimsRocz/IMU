function result = TRIAD(imuFile, gnssFile)
%TRIAD Run the pipeline using only the TRIAD method.
%   RESULT = TRIAD(IMUFILE, GNSSFILE) executes Tasks 1--5 with the TRIAD
%   attitude initialisation and returns the Task 5 results struct. The
%   results are also saved in results/Result_IMU_GNSS_TRIAD.mat and the
%   summary line printed to the console.

if nargin < 2
    error('Usage: TRIAD(''IMUFILE'',''GNSSFILE'')');
end

Task_1(imuFile, gnssFile, 'TRIAD');
Task_2(imuFile, gnssFile, 'TRIAD');
Task_3(imuFile, gnssFile, 'TRIAD');
Task_4(imuFile, gnssFile, 'TRIAD');
Task_5(imuFile, gnssFile, 'TRIAD');

[~, imuName]  = fileparts(imuFile);
[~, gnssName] = fileparts(gnssFile);
res_file = fullfile('results', sprintf('%s_%s_TRIAD_task5_results.mat', ...
    imuName, gnssName));
result = load(res_file);

save(fullfile('results', sprintf('Result_%s_%s_TRIAD.mat', imuName, gnssName)), ...
    '-struct', 'result');

sum_file = fullfile('results', 'IMU_GNSS_summary.txt');
if isfile(sum_file)
    lines = splitlines(fileread(sum_file));
    if ~isempty(lines)
        disp('--- TRIAD Method Summary ---');
        disp(lines{end-1});
    end
end
end
