function rpt = spinn3d_energy_check(robot, log, q_ref, gate, caps)
% Compute Hamiltonian H(t), reference H*, and diagnostics:
% - deltaH(t) = max(H(t)-H*, 0)
% - monotone_ratio: fraction of steps with deltaH(k+1) <= deltaH(k) + tol
% - violations of P(t) <= Pbud(t) if gate provided
% Inputs:
%   robot : rigidBodyTree
%   log   : struct with fields t, q, dq, P (optional)
%   q_ref : 1xN reference configuration (rad). If empty, use last q.
%   gate  : struct('enable',bool,'Pmin',W,'kE',1/s)  (optional)
%   caps  : struct with Pmax (optional, only used when gate provided)
%
% Outputs:
%   rpt: struct with fields H, Hstar, dH, monotone_ratio, Pbud(optional),
%        p_over(optional), idx_nonmono, summary string.
%
% Note: This function does not plot. It only computes diagnostics.
%
if nargin < 3 || isempty(q_ref), q_ref = log.q(end,:); end
if nargin < 4, gate = struct(); end
if nargin < 5, caps = struct('Pmax', inf); end

N = size(log.q,1);
H   = zeros(N,1);
dH  = zeros(N,1);
for k=1:N
    S = spinn3d_state_energy(robot, log.q(k,:), log.dq(k,:));
    H(k) = S.H;
end
Sstar = spinn3d_state_energy(robot, q_ref, zeros(1,size(log.q,2)));
Hstar = Sstar.H;
dH = max(0, H - Hstar);

% monotonic descent ratio
tol = 1e-6;
dec = (dH(2:end) <= dH(1:end-1) + tol);
monotone_ratio = mean(dec);

rpt = struct();
rpt.H = H; rpt.Hstar = Hstar; rpt.dH = dH;
rpt.monotone_ratio = monotone_ratio;
rpt.idx_nonmono = find(~dec) + 1;  % indices where dH increases

% if gate provided, compute Pbud and check P<=Pbud
if isfield(gate,'enable') && gate.enable && isfield(log,'P') && ~isempty(log.P)
    if ~isfield(gate,'Pmin') || isempty(gate.Pmin), gate.Pmin = 20; end
    if ~isfield(gate,'kE')   || isempty(gate.kE),   gate.kE   = 5.0; end
    Pbud = min(caps.Pmax, max(0, gate.Pmin + gate.kE * dH));
    rpt.Pbud = Pbud(:);
    rpt.p_over = max(0, log.P(:) - Pbud(:));
    rpt.p_over_ratio = mean(log.P(:) > Pbud(:));
else
    rpt.Pbud = [];
    rpt.p_over = [];
    rpt.p_over_ratio = NaN;
end

% summary string
rpt.summary = sprintf('Energy descent monotone ratio: %.1f%%;  P<=Pbud violations: %s', ...
                      100*monotone_ratio, ...
                      iif(isfield(rpt,'p_over_ratio') && ~isnan(rpt.p_over_ratio), ...
                          sprintf('%.1f%%', 100*rpt.p_over_ratio), 'n/a'));

end

function y = iif(c,a,b)
if c, y=a; else, y=b; end
end
