function [tau_capped, info] = spinn3d_caps(tau, dq, caps)
% 逐轴→总功率，积分前口径；支持 dq_floor，兼容旧 epsw。
if nargin<3, caps=struct(); end
if ~isfield(caps,'useTotalPowerCap'), caps.useTotalPowerCap = true; end
if ~isfield(caps,'useAxisPowerCap'),  caps.useAxisPowerCap  = true; end
if ~isfield(caps,'dq_floor'),         caps.dq_floor        = [];   end
if ~isfield(caps,'epsw'),             caps.epsw            = 1e-3; end

tau_capped = tau(:).';  dq = dq(:).';
info = struct('scaledTotal',false, 'cappedAxis',false(size(tau_capped)), 'Pmax_eff', 0);

% 0) 扭矩硬帽
if isfield(caps,'tauMax') && ~isempty(caps.tauMax)
    tau_capped = sign(tau_capped).*min(abs(tau_capped), caps.tauMax(:).');
end

% 1) 轴额定功率帽（用 dq_safe）
if caps.useAxisPowerCap && isfield(caps,'Prated') && ~isempty(caps.Prated)
    if isempty(caps.dq_floor)
        dq_safe = abs(dq) + caps.epsw;
    else
        dq_safe = max(abs(dq), caps.dq_floor);
    end
    tau_axis_cap = caps.Prated(:).' ./ dq_safe;
    info.cappedAxis = abs(tau_capped) > (tau_axis_cap + 1e-12);
    tau_capped = sign(tau_capped).*min(abs(tau_capped), tau_axis_cap);
end

% 2) 总功率统一缩放
if caps.useTotalPowerCap && (isfield(caps,'Pmax_eff') || isfield(caps,'Pmax'))
    if isfield(caps,'Pmax_eff') && ~isempty(caps.Pmax_eff)
        if isa(caps.Pmax_eff,'function_handle')
            t = getfield(caps,'time',0);
            info.Pmax_eff = caps.Pmax_eff(t);
        else
            info.Pmax_eff = caps.Pmax_eff;
        end
    elseif isfield(caps,'Pmax') && ~isempty(caps.Pmax)
        info.Pmax_eff = caps.Pmax;
    end
    if info.Pmax_eff > 0
        denom = sum(abs(tau_capped(:).*dq(:))) + 1e-12;
        s_tot = min(1.0, info.Pmax_eff / denom);
        if s_tot < 1.0
            tau_capped = s_tot * tau_capped;
            info.scaledTotal = true;
        end
    end
end
end
