% SPINN3D_DEMO_NN_V2_PBUD —— NN_v2 推理（无动画）+ 在“Total Power”图中叠加 P_bud(t)
% 口径：P_bud(t) = g(t) * Pmax，按“积分前”的 (q,dq) 计算；限幅到 [Pmin, Pmax]
% 依赖：spinn3d_params_block.m, spinn3d_robot.m, spinn3d_controller_pid.m,
%       spinn3d_controller_nn_v2.m, spinn3d_features_v2.m, spinn3d_simulate.m,
%       spinn3d_static_figure.m, spinn3d_power_check.m, spinn3d_energy_gate.m

clear; clc;

% -------------------------
% 参数与对象
% -------------------------
if ~(exist('params','var')&&exist('sim','var')&&exist('caps','var')&&exist('Kp','var')&&exist('Kd','var')&&exist('Ki','var'))
    run('spinn3d_params_block.m');
end
if ~(exist('MODEL_MAT','var') && exist(MODEL_MAT,'file')==2)
    error('未找到 NN 模型文件：%s', MODEL_MAT);
end
if ~exist('gate','var')
    gate = struct('enable', true, 'Pmin', 20, 'kE', 5.0);
end

% 仿真前清缓存，避免上一轮会话串味
gate.reset = true;

% 构建对象
model        = load(MODEL_MAT);
[robot, kin] = spinn3d_robot(params);
pid          = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);
nnctl        = spinn3d_controller_nn_v2(model, @spinn3d_features_v2, caps, gate);

% -------------------------
% 仿真
% -------------------------
controllerFcn  = @(t,q,dq,qr,dqr,rb) nnctl.step(t,q,dq,qr,dqr,rb,kin,pid);
log            = spinn3d_simulate(robot, controllerFcn, sim);

% -------------------------
% 体检 + 固定风格出图
% -------------------------
spinn3d_power_check(log);
opts_fig = struct('q_goal', sim.q_ref, 'goal_radius',0.03, 'Pmax', caps.Pmax, ...
                  'trim_t0',0.02, 'robust_pct',0.995, 'show_raw', false);
spinn3d_static_figure(kin, log, opts_fig);

% -------------------------
% P_bud(t)（积分前）计算 + 只画一条
% -------------------------
if exist('spinn3d_energy_gate','file')==2 && isstruct(gate) && isfield(gate,'enable') && gate.enable
    % 独立 gate 实例：只在循环前 reset 一次
    gate_local = gate;
    gate_local.reset = true;

    Nt   = numel(log.t);
    Pbud = zeros(Nt,1);

    % 第 1 步用 (q0,dq0)，之后每步用上一帧 log 状态（积分前口径）
    q_prev  = sim.q0;
    dq_prev = sim.dq0;
    for k = 1:Nt
        [Pbud(k), ~] = spinn3d_energy_gate(robot, q_prev, dq_prev, sim.q_ref, caps, gate_local);
        q_prev  = log.q(k,:);   % 下一步的“前态”
        dq_prev = log.dq(k,:);
    end

    % 限幅到 [Pmin, Pmax]（定义上应成立）
    Pbud = min(Pbud, caps.Pmax);
    if isfield(gate,'Pmin'), Pbud = max(Pbud, gate.Pmin); end

    % 找到“Total Power”子图
    fig = gcf; axP = [];
    for a = transpose(findall(fig,'Type','axes'))
        ttl = ""; yl = "";
        if ~isempty(get(a,'Title')),  ttl = get(get(a,'Title'),'String'); end
        if ~isempty(get(a,'YLabel')), yl  = get(get(a,'YLabel'),'String'); end
        if (ischar(ttl) && contains(ttl,'Total Power')) || (isstring(ttl) && contains(ttl,"Total Power"))
            axP = a; break;
        end
        if isempty(axP) && ((ischar(yl) && contains(yl,'Power')) || (isstring(yl) && contains(yl,"Power")))
            axP = a;
        end
    end

    if ~isempty(axP)
        % 先删除已存在的 P_bud 曲线，避免“叠两条像画一圈”
        old = findobj(axP,'Type','line','-and','DisplayName','P_{bud}(t)');
        if ~isempty(old), delete(old); end

        hold(axP,'on');
        % 用 '-.' 与 Pmax_eff 的 ':' 区分
        plot(axP, log.t, Pbud, '-.', 'LineWidth', 1.6, 'DisplayName', 'P_{bud}(t)');
        legend(axP,'show','Location','best'); 
        hold(axP,'off');
    else
        warning('未定位到“Total Power”子图，跳过 P_{bud}(t) 叠加。');
    end

    % 轻量一致性报告（不强行终止）
    worstGap = max(0, max(log.Pmax_eff(:) - Pbud(:)));
    fprintf('[P_bud] max(Pmax_eff - Pbud) = %.3g W（应≈0）\n', worstGap);
else
    fprintf('门控禁用或缺少 spinn3d_energy_gate：未叠加 P_{bud}(t)。\n');
end
