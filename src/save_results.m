function save_results(result, tag)
%SAVE_RESULTS Persist result structure to the results directory.
%   SAVE_RESULTS(RESULT, TAG) writes RESULT to results/<TAG>_summary.mat.

if isempty(result)
    return;
end
outfile = fullfile('results', sprintf('%s_summary.mat', tag));
if ~exist('results','dir'); mkdir('results'); end
save(outfile, '-struct', 'result');
end
