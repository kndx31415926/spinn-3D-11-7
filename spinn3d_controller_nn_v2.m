function controller = spinn3d_controller_nn_v2(model, featureFcn, caps, gate)
% NN 分配 (alpha,g) + 方向保持；重力补偿不参与缩放；g/功率带地板；推理鲁棒
% 重要：geometricJacobian 的线速度块是 J(4:6,:)（前3行是角速度）
% 新版要求：网络输入维度 = 基础特征 + [mass, rx, ry, rz]（4维） → 46 列

% ===== gate 默认 =====
if nargin < 4 || isempty(gate), gate = struct(); end
if ~isfield(gate,'enable'),            gate.enable = true;         end
if ~isfield(gate,'goal_radius'),       gate.goal_radius = 0.03;    end
if ~isfield(gate,'beta_align'),        gate.beta_align = 0.5;      end
if ~isfield(gate,'fast_gmin'),         gate.fast_gmin = 0.0;       end
if ~isfield(gate,'fast_k'),            gate.fast_k    = 0.0;       end
% ★ 与训练对齐：默认使用“关节空间 PID”方向
if ~isfield(gate,'use_task_space_pd'), gate.use_task_space_pd = false; end
if ~isfield(gate,'Kx'),                gate.Kx = 600*eye(3);       end
if ~isfield(gate,'Dx'),                gate.Dx = 40*eye(3);        end

% ===== caps 默认 + 地板（防断电） =====
if ~isfield(caps,'Pmax'),        caps.Pmax = 50;      end
if ~isfield(caps,'Prated'),      caps.Prated = inf;   end
if ~isfield(caps,'tauMax'),      caps.tauMax = inf;   end
if ~isfield(caps,'dq_floor'),    caps.dq_floor = 0.2; end
if ~isfield(caps,'g_min_hold'),  caps.g_min_hold = 0.08; end
if ~isfield(caps,'Pmin'),        caps.Pmin = 0.0;     end

controller.model      = model;
controller.featureFcn = featureFcn;
controller.caps       = caps;
controller.gate       = gate;
controller.step       = @step;

    function varargout = step(t, q, dq, q_ref, dq_ref, robot, kin, pid)
        n = numel(q); epsw = 1e-6;

        % === 1) 特征 + 末端负载四维 ===
        f0   = local_features(controller.featureFcn, t, q, dq, q_ref, dq_ref, robot, kin);
        pl4  = local_payload(robot);              % [mass, rx, ry, rz]
        feat = [f0, pl4];                         % 总维度=46

        % === 2) NN 预测 ===
        y = local_infer(controller.model, feat);  % 期望输出 1×(n+1)
        assert(numel(y) >= n+1, 'NN output dim mismatch.');
        zA = y(1:n); zG = y(n+1);

        % alpha softmax / g sigmoid
        zA = zA - max(zA);  ez = exp(zA);  alpha = ez / (sum(ez)+1e-9);
        g_net = 1./(1+exp(-min(max(zG,-60),60)));

        % === 3) 能量门上界（兼容 2/3 输出） + 外域底座 ===
        g_H = 1.0;
        if controller.gate.enable
            try
                [~, gHtmp, ~] = spinn3d_energy_gate(robot, q, dq, q_ref, controller.caps, controller.gate);
            catch
                try, [~, gHtmp] = spinn3d_energy_gate(robot, q, dq, q_ref, controller.caps, controller.gate);
                catch, gHtmp = [];
                end
            end
            if ~isempty(gHtmp), g_H = min(max(gHtmp,0),1); end
        end
        if controller.gate.fast_gmin > 0
            dist = local_dist_ee_robot(robot, q, q_ref);
            r = controller.gate.goal_radius;
            if isfinite(dist) && dist > r
                lam = min(1.0, max(0.0,(dist - r)/r));
                g_net = max(g_net, min(1.0, controller.gate.fast_gmin + controller.gate.fast_k*lam));
            end
        end

        % ★ g/功率地板：防止“断电”
        g = max(min(g_net, g_H), controller.caps.g_min_hold);
        P_budget = max(g * controller.caps.Pmax, controller.caps.Pmin);

        % === 4) 逐轴允许幅值 ===
        Prated = controller.caps.Prated(:)'; if isscalar(Prated), Prated=repmat(Prated,1,n); end
        tauMax = controller.caps.tauMax(:)'; if isscalar(tauMax), tauMax=repmat(tauMax,1,n); end
        dq_safe = max(abs(dq), controller.caps.dq_floor) + epsw;
        P_axis     = min(alpha .* P_budget, Prated);
        tau_from_P = P_axis ./ dq_safe;
        tau_lim    = min(tau_from_P, tauMax);

        % === 5) 名义方向扭矩（与训练一致：默认 PID） + 重力基线 ===
        if controller.gate.use_task_space_pd
            [tau_pd, ~, Jlin] = local_task_pd_robot(robot, q, dq, q_ref, controller.gate.Kx, controller.gate.Dx);
        else
            if nargin >= 8 && ~isempty(pid) && isstruct(pid) && isfield(pid,'step')
                [tau_pd, ~] = pid.step(t, q, dq, q_ref, dq_ref, robot);
            else
                error('PID controller required for joint-space direction.');
            end
            J = local_J(robot, q); Jlin = J(4:6,:);
        end
        try, tau_gc = gravityTorque(robot, q); catch, tau_gc = zeros(1,n); end
        if controller.gate.use_task_space_pd
          tau_dir0 = tau_pd;          % 任务空间 PD 本就不含重力
        else
          tau_dir0 = tau_pd - tau_gc; % 关节 PID 含重力，这里去重力
        end
        % === 6) α 与推进贡献折中（A：吸能/减势口径） ===
        beta = controller.gate.beta_align;
        if beta > 0
            w = max(0, -tau_dir0(:)'.*dq(:)');    % 与训练一致
            if sum(w) <= 1e-12, w = abs(tau_dir0(:)'.*dq(:)'); end  % 兜底
            if sum(w) > 0
                a_prog = w/sum(w);
                alpha  = (1-beta)*alpha + beta*a_prog;
                alpha  = alpha / sum(alpha);
                % 重新据 α 计算允许幅值
                P_axis     = min(alpha .* P_budget, Prated);
                tau_from_P = P_axis ./ dq_safe;
                tau_lim    = min(tau_from_P, tauMax);
            end
        end

        % === 7) 零空间重分配（保持末端方向） ===
        N = null(Jlin);
        tau_dir = tau_dir0(:);
        if ~isempty(N)
            W = diag(1 ./ (tau_lim(:).^2 + 1e-12));
            A = (N.' * W * N);  b = -(N.' * W * tau_dir);
            if rcond(A) > 1e-8, z = A \ b; tau_dir = tau_dir + N*z; end
        end
        tau_dir = tau_dir.';

        % === 8) 逐轴不越界 + 总功率等比（只作用于“方向”） ===
        s_dir  = min(1.0, min(tau_lim ./ (abs(tau_dir) + 1e-9)));
        tau_cmd = s_dir * tau_dir;
        P_cmd   = sum(abs(tau_cmd .* dq));
        if P_cmd > P_budget && P_cmd > 0
            tau_cmd = tau_cmd * (P_budget / P_cmd);
        end

        % ★ 把重力补偿加回（基线始终在，不受 g 缩放）
        tau = tau_cmd + tau_gc;

        if nargout > 1
            info = struct('alpha',alpha,'g',g,'P_budget',P_budget,'tau_lim',tau_lim, ...
                          'tau_pd',tau_pd,'tau_gc',tau_gc,'tau_cmd',tau_cmd,'Jlin',Jlin);
            varargout = {tau, info};
        else
            varargout = {tau};
        end
    end
end

% ======== helpers ========

function feat = local_features(featureFcn, t, q, dq, q_ref, dq_ref, robot, kin)
    try
        feat = featureFcn(t, q, dq, q_ref, dq_ref, robot, kin);
    catch
        try, feat = featureFcn(t, q, dq, q_ref, dq_ref, robot);
        catch,   feat = double([q(:); dq(:); q_ref(:); dq_ref(:)]).';
        end
    end
    feat = double(feat(:)).';  % 1×D0 行向量
end

function pl4 = local_payload(robot)
% 从当前 robot 中读取末端 tool 的质量与质心，返回 1×4 行向量
    try
        b = getBody(robot, 'tool');
    catch
        names = robot.BodyNames; b = getBody(robot, names{end});
    end
    m  = double(b.Mass);
    rc = double(b.CenterOfMass(:)).';
    if numel(rc) ~= 3, rc = [0 0 0]; end
    pl4 = [m, rc];
end

% —— 推理：鲁棒拆包 + 兼容各类网络对象/函数 —— 
function y = local_infer(model, xrow)
    x = double(xrow);                 % 1×D
    model = unwrap_model(model);
    if isfield(model,'forward') && isa(model.forward,'function_handle')
        y = local_rowify(model.forward(x)); return;
    end
    if isfield(model,'dlnet') && isa(model.dlnet,'dlnetwork')
        y = local_predict_dlnet(model.dlnet, x); return;
    end
    assert(isfield(model,'net'), 'MODEL has no .net/.dlnet/.forward after unwrap.');
    net = model.net;
    if isa(net,'function_handle'), y = local_rowify(feval(net,x)); return; end
    if isa(net,'dlnetwork'),      y = local_predict_dlnet(net, x); return; end
    if strcmpi(class(net),'network'), y = local_rowify(net(x.')); return; end
    if ismethod(net,'predict') || isprop(net,'Layers'), y = local_try_predict_anyshape(net, x); return; end
    if isstruct(net) && isfield(net,'forward') && isa(net.forward,'function_handle')
        y = local_rowify(net.forward(x)); return;
    end
    error('Unsupported model.net type.');
end

function M = unwrap_model(M)
    if isa(M,'dlnetwork') || isa(M,'SeriesNetwork') || isa(M,'DAGNetwork') ...
       || strcmpi(class(M),'network') || isa(M,'function_handle')
        M = struct('net', M); return;
    end
    if isstruct(M) && (isfield(M,'net') || isfield(M,'dlnet') || isfield(M,'forward')), return; end
    if isstruct(M)
        f = fieldnames(M);
        for i=1:numel(f)
            v = M.(f{i});
            if isa(v,'dlnetwork') || isa(v,'SeriesNetwork') || isa(v,'DAGNetwork') ...
               || strcmpi(class(v),'network') || isa(v,'function_handle')
                M = struct('net', v); return;
            end
            if isstruct(v) && (isfield(v,'net') || isfield(v,'dlnet') || isfield(v,'forward')), M = v; return; end
        end
    end
    error('MODEL_MAT does not contain a supported network.');
end

function y = local_predict_dlnet(dlnet, x)
    xdl = dlarray(single(x.'), 'CB');   % (D×B)
    ydl = predict(dlnet, xdl);
    if isa(ydl,'gpuArray'), ydl = gather(ydl); end
    y = double(extractdata(ydl)); y = y(:).';
end

function y = local_try_predict_anyshape(net, x)
    cand = {};
    try
        if isprop(net,'Layers') && numel(net.Layers)>=1 && isprop(net.Layers(1),'InputSize')
            inSz = net.Layers(1).InputSize; Dneed = prod(inSz);
            if isnumeric(inSz) && all(inSz>0) && numel(x)==Dneed
                cand{end+1} = reshape(single(x), [inSz 1]);  % append batch
            end
        end
    end
    cand = [cand, { double(x), single(x), double(x.'), single(x.') }];
    lastErr=''; 
    for i=1:numel(cand)
        try, yi = predict(net, cand{i}); y = local_rowify(yi); return; catch ME, lastErr=ME.message; end
    end
    error('predict(net,X) failed for all shapes. Last: %s', lastErr);
end

function y = local_rowify(yi)
    if isa(yi,'gpuArray'), yi = gather(yi); end
    y = double(yi(:)).';
end

% —— 任务空间 PD（用 robot 几何 + 线速度雅可比 J(4:6,:)） —— 
function [tau0, f, Jlin] = local_task_pd_robot(robot, q, dq, q_ref, Kx, Dx)
    J  = local_J(robot, q);
    Jlin = J(4:6,:);                  % 线速度块
    T  = getTransform(robot, q,     'tool');
    Tg = getTransform(robot, q_ref, 'tool');
    x  = T (1:3,4);  xg = Tg(1:3,4);
    vx = Jlin * dq(:);
    f  = Kx*(xg - x) + Dx*(0 - vx);
    tau0 = (Jlin.' * f).';
end

function J = local_J(robot, q)
    ee = 'tool';
    try, J = geometricJacobian(robot, q, ee);
    catch, names = robot.BodyNames; ee = names{end}; J = geometricJacobian(robot, q, ee);
    end
end

% —— EE 距离（统一 robot） —— 
function d = local_dist_ee_robot(robot, q, q_ref)
    try
        T1 = getTransform(robot, q,     'tool');
        T2 = getTransform(robot, q_ref, 'tool');
        d = norm(T1(1:3,4) - T2(1:3,4));
    catch, d = NaN;
    end
end
