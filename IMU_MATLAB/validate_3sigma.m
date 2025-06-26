function validate_3sigma(matFile, stateFile)
% Compare estimation error with 3-sigma bounds from covariance.
% STATE_X001.txt should contain the true state as [x y z ...].
% The script plots error and ±3σ envelopes for sanity check.

S = load(matFile);
truth = load(stateFile);
err = S.pos_ned - truth(:,1:3);
if isfield(S, 'P')
    Pdiag = squeeze(S.P(:,1:3,1:3));
    sigma = 3*sqrt(Pdiag);
else
    sigma = [];
end

labels = {'North','East','Down'};
for i=1:3
    figure;
    plot(err(:,i),'DisplayName','error'); hold on;
    if ~isempty(sigma)
        plot(sigma(:,i),'r--','DisplayName','+3\sigma');
        plot(-sigma(:,i),'r--','HandleVisibility','off');
    end
    xlabel('Sample'); ylabel([labels{i} ' error [m]']);
    legend; grid on;
    title([labels{i} ' Error vs 3\sigma']);
    saveas(gcf, sprintf('%s_err_%s.pdf', matFile(1:end-4), labels{i}));
end
end
