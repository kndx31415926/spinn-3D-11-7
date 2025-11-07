function [Pbud, gH, dH] = spinn3d_energy_gate(robot, q, dq, q_ref, caps, gate)
% SPINN3D_ENERGY_GATE (patched)
% Energy-aware total power budget with:
%   - adaptive kE: optionally choose kE so that initial ΔH pushes Pbud≈Pmax
%   - terminal deadzone: when ΔH ≤ H_eps -> Pbud = Pmin
%
% Inputs:
%   robot : rigidBodyTree
%   q,dq  : state
%   q_ref : reference configuration for energy baseline
%   caps  : struct with Pmax field
%   gate  : struct with fields (all optional; robust defaults provided):
%       .enable      (bool, default true)
%       .Pmin        (W,   default 10)
%       .kE          (1/s, default 3.5)  % base slope
%       .autoKE      (bool, default true)
%       .gamma       (>=1, default 1.3)  % multiplier for auto kE
%       .dH0         (J,   default [] )  % known initial ΔH; if empty will use max ΔH seen
%       .H_eps       (J,   default 0.05) % terminal deadzone
%       .reset       (bool, default false) % when true, reset internal max-ΔH cache
%
% Outputs:
%   Pbud : power budget (W)
%   gH   : normalized gate in [0,1]
%   dH   : current Delta-H (J)
%
% Notes:
% - Robust to single inputs (q,dq are cast to double via state_energy).
% - If gate.enable is false or gate missing -> Pbud=Pmax, gH=1.
% - Internal persistent max-ΔH cache is per-session; set gate.reset=true to clear.
%
% Copyright: patched for adaptive kE + terminal deadzone + compatibility.
%
% ---- compute ΔH ----
S_now  = spinn3d_state_energy(robot, q, dq);
S_goal = spinn3d_state_energy(robot, q_ref, zeros(size(dq)));
dH = max(0, S_now.H - S_goal.H);

% ---- defaults & early exits ----
if nargin < 6 || isempty(gate) || ~isstruct(gate) || ~isfield(gate,'enable') || ~gate.enable
    Pbud = caps.Pmax; gH = 1.0;
    return;
end
% defaults
Pmin = 10; kE = 3.5; gamma = 1.3; H_eps = 0.05;
if isfield(gate,'Pmin') && ~isempty(gate.Pmin), Pmin = gate.Pmin; end
if isfield(gate,'kE')   && ~isempty(gate.kE),   kE   = gate.kE;   end
if isfield(gate,'gamma')&& ~isempty(gate.gamma),gamma= gate.gamma;end
if isfield(gate,'H_eps')&& ~isempty(gate.H_eps),H_eps= gate.H_eps;end
autoKE = true;
if isfield(gate,'autoKE') && ~isempty(gate.autoKE), autoKE = logical(gate.autoKE); end

% ---- adaptive kE (optional) ----
persistent dHmax_cache
if isempty(dHmax_cache), dHmax_cache = 0; end
if isfield(gate,'reset') && ~isempty(gate.reset) && gate.reset
    dHmax_cache = 0;
end

dHref = [];
if isfield(gate,'dH0') && ~isempty(gate.dH0) && gate.dH0>0
    dHref = gate.dH0;
end
% fallback: use max ΔH seen so far as proxy for initial energy gap
dHmax_cache = max(dHmax_cache, dH);
if isempty(dHref)
    dHref = dHmax_cache;
end

kE_eff = kE;
if autoKE && dHref>0 && isfinite(dHref)
    kE_target = gamma * (caps.Pmax - Pmin) / max(dHref, 1e-9);
    kE_eff = max(kE, kE_target);
end

% ---- terminal deadzone ----
if dH <= H_eps
    Pbud = Pmin;
else
    Pbud = Pmin + kE_eff * dH;
    Pbud = min(caps.Pmax, max(0, Pbud));
end

gH = Pbud / max(1e-9, caps.Pmax);
end
