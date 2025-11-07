function [q,dq, touched] = spinn3d_joint_limit(robot, q, dq, opts)
if nargin < 4, opts = struct(); end
if ~isfield(opts,'deadband_deg'), opts.deadband_deg = 0.5; end
if ~isfield(opts,'freezeInward'), opts.freezeInward = true; end
db = deg2rad(opts.deadband_deg);

bodies = robot.Bodies; n = numel(q);
qmin = -inf(1,n); qmax = inf(1,n);
for i=1:n
    lim = bodies{i}.Joint.PositionLimits;
    if ~isempty(lim), qmin(i)=lim(1); qmax(i)=lim(2); end
end

touched = false(1,n);
for i=1:n
    if q(i) < qmin(i)
        q(i) = qmin(i); touched(i)=true; if opts.freezeInward && dq(i)<0, dq(i)=0; end
    elseif q(i) > qmax(i)
        q(i) = qmax(i); touched(i)=true; if opts.freezeInward && dq(i)>0, dq(i)=0; end
    else
        if q(i) <= (qmin(i)+db) && dq(i) < 0, dq(i)=0; touched(i)=true; end
        if q(i) >= (qmax(i)-db) && dq(i) > 0, dq(i)=0; touched(i)=true; end
    end
end
end
