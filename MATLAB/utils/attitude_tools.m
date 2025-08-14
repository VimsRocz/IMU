function out = attitude_tools(action, varargin)
% ATTITUDE_TOOLS  Helper switchboard for quaternion & frame math.
% Usage: out = attitude_tools('funcname', args...)
switch lower(action)
    case 'quat_normalize'
        q = varargin{1};  % [4 x N] or [N x 4] or [4] or [1x4]
        [q, isRow] = ensureQuatShape(q);
        q = q ./ max(eps, vecnorm(q));
        if isRow, q = q.'; end
        out = q;

    case 'quat_hemi'
        q = attitude_tools('quat_normalize', varargin{1});
        [q, isRow] = ensureQuatShape(q);
        for k = 2:size(q,2)
            if dot(q(:,k), q(:,k-1)) < 0, q(:,k) = -q(:,k); end
        end
        if isRow, q = q.'; end
        out = q;

    case 'quat_to_dcm_batch'
        q = attitude_tools('quat_hemi', varargin{1});        % [4 x N] (wxyz)
        [q, isRow] = ensureQuatShape(q);
        q0=q(1,:); q1=q(2,:); q2=q(3,:); q3=q(4,:);
        N = size(q,2);
        R = zeros(3,3,N);
        R(1,1,:) = 1-2*(q2.^2+q3.^2);
        R(1,2,:) = 2*(q1.*q2 - q0.*q3);
        R(1,3,:) = 2*(q1.*q3 + q0.*q2);
        R(2,1,:) = 2*(q1.*q2 + q0.*q3);
        R(2,2,:) = 1-2*(q1.^2+q3.^2);
        R(2,3,:) = 2*(q2.*q3 - q0.*q1);
        R(3,1,:) = 2*(q1.*q3 - q0.*q2);
        R(3,2,:) = 2*(q2.*q3 + q0.*q1);
        R(3,3,:) = 1-2*(q1.^2+q2.^2);
        out = R;

    case 'slerp_series'
        % q_src: [4 x Ns] (wxyz), t_src: [1 x Ns], t_dst: [1 x Nd]
        t_src = varargin{1}; q_src = varargin{2}; t_dst = varargin{3};
        [q_src, ~] = ensureQuatShape(q_src);
        q_src = attitude_tools('quat_hemi', q_src);
        % MATLAB slerp polyfill using Robotics System Toolbox if available; else lerp+renorm
        try
            % Convert to quaternion class if available (wxyz order)
            qObj = quaternion(q_src(1,:), q_src(2,:), q_src(3,:), q_src(4,:));
            qInterp = slerp(qObj, t_src(:), t_dst(:));
            q = compact(qInterp).';  % returns [N x 4] wxyz
            out = q.';
        catch
            % fallback: componentwise linear + renorm
            q = zeros(4, numel(t_dst));
            for i=1:4, q(i,:) = interp1(t_src, q_src(i,:), t_dst, 'linear', 'extrap'); end
            q = attitude_tools('quat_normalize', q);
            out = q;
        end

    case 'ecef2ned_R'
        % lat,lon in radians; NED-from-ECEF
        lat = varargin{1}; lon = varargin{2};
        sL=sin(lat); cL=cos(lat); sO=sin(lon); cO=cos(lon);
        R = [ -sL*cO, -sL*sO,  cL;
                 -sO,     cO,  0;
              -cL*cO, -cL*sO, -sL ];
        out = R;

    case 'ecef2ned_vec'
        v_e = varargin{1}; lat = varargin{2}; lon = varargin{3};
        R = attitude_tools('ecef2ned_R', lat, lon);
        out = (R * v_e.').';

    case 'ned2ecef_vec'
        v_n = varargin{1}; lat = varargin{2}; lon = varargin{3};
        R = attitude_tools('ecef2ned_R', lat, lon).';
        out = (R * v_n.').';

    case 'ned2body_series'
        % v_ned [N x 3], q_b2n [4 x N] (wxyz). body = R_bn * ned with R_bn = R_b2n^T
        v_ned = varargin{1}; q = varargin{2};
        [q, ~] = ensureQuatShape(q);
        Rb2n = attitude_tools('quat_to_dcm_batch', q);  % 3x3xN
        N = size(v_ned,1); vb = zeros(N,3);
        for k=1:N
            vb(k,:) = (Rb2n(:,:,k).') * v_ned(k,:).';
        end
        out = vb;

    case 'interp_to'
        t_src = varargin{1}; X = varargin{2}; t_dst = varargin{3};
        if isempty(X), out = []; return; end
        if size(X,1) ~= numel(t_src)
            error('interp_to: size mismatch: %d vs %d', size(X,1), numel(t_src));
        end
        out = zeros(numel(t_dst), size(X,2));
        for i=1:size(X,2)
            out(:,i) = interp1(t_src, X(:,i), t_dst, 'linear', 'extrap');
        end

    case 'plot_overlay_3x3'
        t = varargin{1}; EST = varargin{2}; TRU = varargin{3}; titleStr = varargin{4}; outBase = varargin{5};
        % EST/TRU are structs with fields pos, vel, acc (each [N x 3])
        fprintf('[DBG-OL] EST(pos=%s vel=%s acc=%s) TRU(pos=%s vel=%s acc=%s)\n', ...
            mat2str(size(EST.pos)), mat2str(size(EST.vel)), mat2str(size(EST.acc)), ...
            mat2str(size(TRU.pos)), mat2str(size(TRU.vel)), mat2str(size(TRU.acc)));

        f = figure('Visible','off','Position',[100 100 1400 900]);
        rowFun = {@(S)S.pos, @(S)S.vel, @(S)S.acc};
        rowLbl = {'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
        colTitle = {'X/N','Y/E','Z/D'};
        tiledlayout(3,3,'Padding','compact','TileSpacing','compact');
        for r=1:3
            A = rowFun{r}(EST); B = rowFun{r}(TRU);
            for c=1:3
                nexttile((r-1)*3+c);
                if ~isempty(A), plot(t, A(:,c), 'LineWidth',1.2); hold on; else, warning('[Task6] Missing EST data r%d c%d', r, c); end
                if ~isempty(B), plot(t, B(:,c), '--', 'LineWidth',1.0); else, warning('[Task6] Missing TRU data r%d c%d', r, c); end
                grid on; box on;
                if r==1, title(colTitle{c}); end
                ylabel(rowLbl{r});
                if r==3, xlabel('Time [s]'); end
            end
        end
        legend({'Estimated','Truth'},'NumColumns',2,'Location','northoutside');
        sgtitle(titleStr);
        pngPath = strcat(outBase, '.png');
        figPath = strcat(outBase, '.fig');
        exportgraphics(f, pngPath, 'Resolution',150);
        try, savefig(f, figPath); catch ME, warning('[Task6] savefig failed: %s', ME.message); end
        close(f);
        out = pngPath;

    otherwise
        error('Unknown action: %s', action);
end

end

% --- helpers ---
function [q, isRow] = ensureQuatShape(q)
    isRow = isrow(q) && numel(q)==4;
    if isRow, q = q.'; end
    if size(q,1)==4 && size(q,2)~=4
        % ok
    elseif size(q,2)==4 && size(q,1)~=4
        q = q.';  % make [4 x N]
    else
        error('Quaternion must be [4 x N] in wxyz order.');
    end
end
