function plot_quaternion_from_mat(file_path)
% plot_quaternion_from_mat Render quaternion Truth vs Estimate from a .mat bundle.
%   Expects variables: t (Nx1), q_truth (Nx4, wxyz), q_est (Nx4, wxyz)

if nargin < 1 || isempty(file_path)
    [f,p] = uigetfile({'*.mat','MAT-files (*.mat)'}, 'Select quaternion .mat');
    if isequal(f,0), return; end
    file_path = fullfile(p,f);
end

S = load(file_path);
if ~isfield(S,'t') || ~isfield(S,'q_truth') || ~(isfield(S,'q_est') || isfield(S,'q_kf'))
    error('Expected variables t, q_truth and q_est/q_kf in %s', file_path);
end
t = S.t(:);
qt = S.q_truth; qe = []; if isfield(S,'q_est'), qe = S.q_est; else, qe = S.q_kf; end
qt = reshape(qt, [], 4); qe = reshape(qe, [], 4);
n = min([numel(t), size(qt,1), size(qe,1)]);
t = t(1:n); qt = qt(1:n,:); qe = qe(1:n,:);

[~, base] = fileparts(file_path);
figure('Name', base, 'Color','w');
labs = {'w','x','y','z'};
for i=1:4
    ax = subplot(4,1,i); hold(ax,'on'); grid(ax,'on');
    plot(ax, t, qt(:,i), '-', 'DisplayName','Truth');
    plot(ax, t, qe(:,i), '--', 'DisplayName','Estimate');
    ylabel(ax, ['q_' labs{i}]);
    if i==1, legend(ax,'show','Location','best'); end
    if i==4, xlabel(ax,'Time [s]'); end
end
sgtitle(strrep(base,'_','\_'));

end

