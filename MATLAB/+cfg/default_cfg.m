function s = default_cfg()
%DEFAULT_CFG  Central place for run-time policy (no hidden defaults in tasks).
p = project_paths();
s = struct();
s.dataset_id = '';        % must be set by caller
s.method     = '';        % must be set by caller
s.imu_file   = '';
s.gnss_file  = '';
s.truth_file = '';

s.paths = p;

s.plots = struct( ...
    'popup_figures', true, ...
    'save_pdf',      true, ...
    'save_png',      true);
end
