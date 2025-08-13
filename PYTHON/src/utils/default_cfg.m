function cfg = default_cfg()
%DEFAULT_CFG Minimal configuration struct for plotting helpers.
%   Provides plotting flags with safe defaults so code can assume the
%   presence of `cfg.plots.popup_figures`, mirroring Python's default
%   configuration.

cfg = struct();
cfg.plots = struct('popup_figures', false, 'save_png', true);
end
