function [log, metrics] = spinn3d_simulate(robot, controllerFcn, sim)
% SPINN3D_SIMULATE
%  - 动力学：rigidBodyTree
%  - 功率：积分前记账；caps=逐轴→总功率等比
%  - 首达：始终用 robot 几何 (getTransform) 按 EE 笛卡尔距离[米] 判定
%  - 命中即停：外→内穿越时记录 t_hit/v_hit，并可截断日志

    % ===== defaults =====
    if nargin < 3, sim = struct(); end
    if ~isfield(sim,'dt'),         sim.dt = 0.002;    end
    if ~isfield(sim,'t_final'),    sim.t_final = 6.0; end
    if ~isfield(sim,'integrator'), sim.integrator = 'semi'; end
    if ~isfield(sim,'goal_radius'),       sim.goal_radius = 0.03; end   % [m]
    if ~isfield(sim,'stop_on_first_hit'), sim.stop_on_first_hit = false; end

    nq_guess = numel(homeConfiguration(robot));
    if ~isfield(sim,'q0'),       sim.q0       = zeros(1,nq_guess); end
    if ~isfield(sim,'dq0'),      sim.dq0      = zeros(1,nq_guess); end
    if ~isfield(sim,'q_ref'),    sim.q_ref    = zeros(1,nq_guess); end
    if ~isfield(sim,'dq_ref'),   sim.dq_ref   = zeros(1,nq_guess); end
    if ~isfield(sim,'damping'),  sim.damping  = 0;                 end
    if ~isfield(sim,'friction'), sim.friction = [];                end

    % ===== pre-alloc =====
    N_est = ceil(sim.t_final/sim.dt) + 1;
    nq    = numel(sim.q0);

    log.t    = zeros(N_est,1);
    log.q    = zeros(N_est,nq);
    log.dq   = zeros(N_est,nq);
    log.ddq  = zeros(N_est,nq);
    log.tau      = zeros(N_est,nq);
    log.tau_raw  = zeros(N_est,nq);
    log.P3_raw   = zeros(N_est,nq);
    log.P_raw    = zeros(N_est,1);
    log.P3       = zeros(N_est,nq);
    log.P        = zeros(N_est,1);
    log.Pmax_eff   = zeros(N_est,1);
    log.scaledTotal= false(N_est,1);
    log.cappedAxis = false(N_est,nq);
    log.reached  = false(N_est,1);
    log.eeTform  = repmat(eye(4),1,1,N_est);

    log.idx_hit = []; log.t_hit = NaN; log.v_hit = NaN;

    % ===== init =====
    q  = sim.q0; dq = sim.dq0; k_end = 0;

    % 初始 outside/inside（外→内穿越才算首达）——统一用 robot 几何
    T0   = getTransform(robot, sim.q0,   'tool');
    Tr0  = getTransform(robot, sim.q_ref,'tool');     % t=0 的参考
    ee0  = T0(1:3,4).';  eeref = Tr0(1:3,4).';
    d0   = norm(ee0 - eeref);
    was_inside = (d0 <= sim.goal_radius);
    ee_prev = ee0;

    for k = 1:N_est
        t = (k-1)*sim.dt; log.t(k) = t;

        % 参考（支持时变）
        if isfield(sim,'refFcn') && ~isempty(sim.refFcn)
            [qr, dqr] = sim.refFcn(t);
        else
            qr  = sim.q_ref; dqr = sim.dq_ref;
        end

        % 控制器
        tau_raw = controllerFcn(t, q, dq, qr, dqr, robot);

        % === 功率记账（积分前；与 caps 口径一致） ===
        P3_raw = abs(dq(:)'.*tau_raw(:)');     % 每轴功率 |τ_i ω_i|
        P_raw  = sum(P3_raw);                  % 总功率
        tau = tau_raw;

        % === 限幅（逐轴→总功率统一缩放；仍在积分前） ===
        if isfield(sim,'caps') && ~isempty(sim.caps)
            c = sim.caps; c.time = t;
            [tau, capinfo]      = spinn3d_caps(tau, dq, c);      % 逐轴/总功率等比缩放  (顺序同现有)  :contentReference[oaicite:3]{index=3}
            log.scaledTotal(k)  = capinfo.scaledTotal;
            log.cappedAxis(k,:) = capinfo.cappedAxis;
            log.Pmax_eff(k)     = capinfo.Pmax_eff;
        else
            log.Pmax_eff(k)     = 0;
        end

        % 限幅后功率（仍在积分前）
        P3 = abs(dq(:)'.*tau(:)');  P = sum(P3);

        % === 动力学 + 积分 ===
        [ddq, ~, ~, ~] = spinn3d_fd(robot, q, dq, tau, sim.damping, sim.friction);
        if strcmpi(sim.integrator,'semi')          % 半隐式 Euler
            dq = dq + ddq*sim.dt;
            q  = q  + dq *sim.dt;
        else                                       % RK2 (midpoint)
            dq_mid = dq + 0.5*ddq*sim.dt;  q_mid = q + 0.5*dq*sim.dt;
            [ddq_mid,~,~,~] = spinn3d_fd(robot, q_mid, dq_mid, tau, sim.damping, sim.friction);
            dq = dq + ddq_mid*sim.dt;      q  = q + dq_mid *sim.dt;
        end

        % 关节限位（可选）
        if isfield(sim,'jointLimit') && ~isempty(sim.jointLimit)
            [q, dq] = spinn3d_joint_limit(robot, q, dq, sim.jointLimit);
        end

        % === EE 位姿 & 距离（统一用 robot 几何） ===
        Tnow   = getTransform(robot, q,  'tool');
        Tgoal  = getTransform(robot, qr, 'tool');
        EE_now = Tnow (1:3,4).';  goal = Tgoal(1:3,4).';
        dist   = norm(EE_now - goal);
        v_ee   = norm((EE_now - ee_prev)/sim.dt); ee_prev = EE_now;

        inside_now  = (dist <= sim.goal_radius);
        reached_now = (~was_inside) && inside_now;  was_inside = inside_now;

        % === 写日志 ===
        log.q(k,:)      = q;      log.dq(k,:)   = dq;    log.ddq(k,:) = ddq;
        log.tau_raw(k,:)= tau_raw;log.tau(k,:)  = tau;
        log.P3_raw(k,:) = P3_raw; log.P_raw(k)  = P_raw;
        log.P3(k,:)     = P3;     log.P(k)      = P;
        log.reached(k)  = reached_now;
        log.eeTform(:,:,k) = Tnow;

        if reached_now && isempty(log.idx_hit)
            log.idx_hit = k; log.t_hit = t; log.v_hit = v_ee;
            if sim.stop_on_first_hit, k_end = k; break; end
        end

        k_end = k;
        if t + sim.dt > sim.t_final, break; end
    end

    % ===== 截断到首达（如启用） =====
    if sim.stop_on_first_hit && ~isempty(log.idx_hit), F = log.idx_hit; else, F = k_end; end
    F = max(1, min(F, k_end));
    fns = fieldnames(log);
    for ii = 1:numel(fns)
        v = log.(fns{ii});
        if ~(isnumeric(v) || islogical(v)), continue; end
        if isscalar(v), continue;
        if ndims(v)==3
            if size(v,3)>=F, log.(fns{ii}) = v(:,:,1:F); end
        elseif isvector(v)
            if size(v,1) >= size(v,2)
                if size(v,1)>=F, log.(fns{ii}) = v(1:F,:); end
            else
                if size(v,2)>=F, log.(fns{ii}) = v(:,1:F); end
            end
        else
            if size(v,1)>=F
                log.(fns{ii}) = v(1:F,:);
            elseif size(v,2)>=F
                log.(fns{ii}) = v(:,1:F);
            end
        end
    end

    % ===== metrics & tables =====
    metrics = struct();
    metrics.reached = ~isempty(log.idx_hit);
    metrics.t_hit   = log.t_hit;
    metrics.v_hit   = log.v_hit;
    metrics.P_total_table = table(log.t, log.P, 'VariableNames',{'t','P_total'});
    pnames = strcat("P", string(1:size(log.P3,2)));
    metrics.P_axis_table  = array2table([log.t, log.P3], 'VariableNames', ['t', cellstr(pnames)]);

    % ===== 越限提示 =====
    if any(log.Pmax_eff > 0)
        exceed = (log.P - log.Pmax_eff) > max(1.0, 1e-6);
        if any(exceed)
            max_ov = max(log.P(exceed) - log.Pmax_eff(exceed));
            pct    = 100*nnz(exceed)/numel(log.P);
            warning('Total power exceeded on %d steps (%.1f%%); worst overshoot=%.1f W.', ...
                    nnz(exceed), pct, max_ov);
        end
    end
end
