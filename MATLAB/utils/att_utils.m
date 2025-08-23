function varargout = att_utils(op, varargin)
%ATT_UTILS Quaternion helpers: normalize rows, continuity, inverse(conjugate).
%   q = att_utils('normalize_rows', q)
%   q = att_utils('enforce_continuity', q)
%   q = att_utils('inv_unit', q)
%
% All quaternions are assumed [w x y z] and unit or near-unit.

switch lower(op)
    case 'normalize_rows'
        q = varargin{1};
        q = q ./ sqrt(sum(q.^2,2));
        varargout{1} = q;

    case 'enforce_continuity'
        q = varargin{1};
        for k = 2:size(q,1)
            if dot(q(k,:), q(k-1,:)) < 0
                q(k,:) = -q(k,:);
            end
        end
        varargout{1} = q;

    case 'inv_unit'
        q = varargin{1};
        q = [q(:,1), -q(:,2:4)];
        varargout{1} = q;

    otherwise
        error('att_utils: unknown op %s', op);
end

