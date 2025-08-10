function validate_with_truth(est_file, truth_file, output_dir)
%VALIDATE_WITH_TRUTH Stub to mirror Python validate_with_truth behavior.
%   Checks that expected IMU and GNSS files exist before generating
%   overlays. If files are missing, a warning is issued and overlay
%   generation is skipped.
%
%   Usage:
%       validate_with_truth('path/to/IMU_X001_GNSS_X001_method_kf_output.mat', ...
%                          'path/to/truth.csv', 'results');
%
%   TODO: Implement full validation and plotting logic.

arguments
    est_file (1,:) char
    truth_file (1,:) char
    output_dir (1,:) char = 'results'
end

m = regexp(est_file, '(IMU_\w+)_(GNSS_\w+)_([A-Za-z]+)_kf_output', 'tokens', 'once');
if ~isempty(m)
    dataset_dir = fileparts(truth_file);
    imu_file = fullfile(dataset_dir, [m{1}, '.dat']);
    gnss_file = fullfile(dataset_dir, [m{2}, '.csv']);
    missing = {};
    if ~isfile(imu_file)
        missing{end+1} = imu_file; %#ok<AGROW>
    end
    if ~isfile(gnss_file)
        missing{end+1} = gnss_file; %#ok<AGROW>
    end
    if ~isempty(missing)
        for i = 1:numel(missing)
            warning('validate_with_truth:MissingFile', ...
                'Overlay skipped. Expected file not found: %s', missing{i});
        end
        return;
    end
end

% Placeholder for overlay generation to mirror Python implementation.
% plot_overlay(...); % TODO: implement plotting

end
