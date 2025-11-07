function [ddq, M, Cqd, G] = spinn3d_fd(robot, q, dq, tau, damping, friction)
% Forward dynamics: M(q) ddq + C(q,dq)dq + G(q) + D*dq + tau_fric = tau
% damping : scalar or 1xN viscous coefficients (Nm*s/rad)
% friction: optional struct with fields:
%           .Fc (1xN) Coulomb torque [Nm]
%           .v  (1xN) smoothing speed [rad/s] (tanh(dq./v))
if nargin < 5 || isempty(damping), damping = 0; end
if nargin < 6, friction = []; end

n = numel(q);
if isscalar(damping), D = damping*ones(1,n); else, D = damping; end

M   = massMatrix(robot, q);
Cqd = velocityProduct(robot, q, dq);    % C(q,dq)*dq
G   = gravityTorque(robot, q);

tau_visc = D(:).*dq(:);
tau_coul = zeros(n,1);
if ~isempty(friction) && isstruct(friction) && isfield(friction,'Fc')
    Fc = friction.Fc(:);
    if isfield(friction,'v') && ~isempty(friction.v)
        v = friction.v(:); v(v<=0) = 1e-3;
        tau_coul = Fc .* tanh(dq(:)./v);
    else
        tau_coul = Fc .* sign(dq(:));
    end
end

rhs = tau(:) - Cqd(:) - G(:) - tau_visc - tau_coul;
ddq = M \ rhs;
ddq = ddq(:).';
end
