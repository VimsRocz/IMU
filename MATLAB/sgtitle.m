function sgtitle(title_str)
%SGTITLE  Replacement for MATLAB's sgtitle function for Octave compatibility
%   This function creates a figure-level title using text annotation

    if nargin < 1
        error('sgtitle requires a title string');
    end
    
    % Create text annotation at the top of the figure
    annotation('textbox', [0 0.9 1 0.1], ...
               'String', title_str, ...
               'EdgeColor', 'none', ...
               'HorizontalAlignment', 'center', ...
               'FontSize', 14, 'FontWeight', 'bold');
end