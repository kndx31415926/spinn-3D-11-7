function controller = spinn3d_controller_nn_v2(model, featureFcn, caps, gate)
% NN 分配 (alpha,g) + 方向保持；重力补偿不参与缩放；g/功率带地板；推理鲁棒
% 说明：
% - 线速度雅可比块用 J(4:6,:)（前3行为角速度）
% - 网络输入：基础特征 + [mass, rx, ry, rz] 共 46 维
% - 修复点：
%   1) P_budget 同时考虑 gate.Pmin 与 caps.Pmin
%   2) 去掉“最小比缩放”拖没整向量，改为逐轴夹紧后按总功率等比

% ===== gate 默认 =====
if nargin < 4 || isempty(gate), gate = struct(); end
if ~isfield(gate,'enable'),            gate.enable = true;         end
if ~isfield(gate,'goal_radius'),       gate.goal_radius = 0.03;    end
if ~isfield(gate,'beta_align'),        gate.beta_align = 0.5;      end
if ~isfield(gate,'fast_gmin'),         gate.fast_gmin = 0.0;       end
if ~isfield(gate,'fast_k'),            gate.fast_k    = 0.0;       end
if ~isfield(gate,'use_task_space_pd'), gate.use_task_space_pd = false; end
if ~isfield(gate,'Kx'),                gate.Kx = 600*eye(3);       end
if ~isfield(gate,'Dx'),                gate.Dx = 40*eye(3);        end
% 注：如果参数块只有 gate.enable/Pmin/kE，不影响此默认

% ===== caps 默认 + 地板 =====
if ~isfield(caps,'Pmax'),        caps.Pmax = 50;      end
if ~isfield(caps,'Prated'),      caps.Prated = inf;   end
if ~isfield(caps,'tauMax'),      caps.tauMax = inf;   end
if ~isfield(caps,'dq_floor'),    caps.dq_floor = 0.2; end
if ~isfield(caps,'g_min_hold'),  caps.g_min_hold = 0.08; end  % g 的硬地板，防断电
if ~isfield(caps,'Pmin'),        caps.Pmin = 0.0;     end     % caps 内的功率地板

controller.model      = model;
controller.featureFcn = featureFcn;
controller.caps       = caps;
controller.gate       = gate;
controller.step       = @step;

    function varargout = step(t, q, dq, q_ref, dq_ref, robot, kin, pid)
        n = numel(q); epsw = 1e-6; tiny = 1e-9;

        % === 1) 特征 + 末端负载四维 ===
        f0   = local_features(controller.featureFcn, t, q, dq, q_ref, dq_ref, robot, kin);
        pl4  = local_payload(robot);              % [mass, rx, ry, rz]
        feat = [f0, pl4];                         % 总维度=46

        % === 2) NN 预测 ===
        y = local_infer(controller.model, feat);  % 1×(n+1)
        assert(numel(y) >= n+1, 'NN output dim mismatch.');
        zA = y(1:n); zG = y(n+1);

        % alpha softmax / g sigmoid（数值稳定）
        zA = zA - max(zA);  ez = exp(zA);  alpha = ez / (sum(ez)+tiny);
        g_net = 1./(1+exp(-min(max(zG,-60),60)));

        % === 3) 能量门上界 g_H + 可选远场最小 g ===
        g_H = 1.0;
        if controller.gate.enable
            % 兼容不同输出形式的 energy_gate
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

        % === 4) g 地板 + P_budget（★合并 gate.Pmin） ===
        g = max(min(g_net, g_H), controller.caps.g_min_hold);
        Pmin_eff = controller.caps.Pmin;
        if isfield(controller.gate,'Pmin')
            Pmin_eff = max(Pmin_eff, controller.gate.Pmin);
        end
        P_budget = max(g * max(controller.caps.Pmax,tiny), Pmin_eff);

        % === 5) 逐轴允许幅值（额定功率/扭矩约束） ===
        Prated = controller.caps.Prated(:)'; if isscalar(Prated), Prated=repmat(Prated,1,n); end
        tauMax = controller.caps.tauMax(:)'; if isscalar(tauMax), tauMax=repmat(tauMax,1,n); end
        dq_safe = max(abs(dq), controller.caps.dq_floor) + epsw;
        P_axis     = min(alpha .* P_budget, Prated);
        tau_from_P = P_axis ./ dq_safe;
        tau_lim    = min(tau_from_P, tauMax);

        % === 6) 名义方向（默认关节 PID） + 重力基线 ===
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
            tau_dir0 = tau_pd;           % 任务空间 PD 不含重力
        else
            tau_dir0 = tau_pd - tau_gc;  % 去重力后的反馈方向
        end

        % === 7) α 与推进贡献折中（吸能口径，与训练一致） ===
        beta = controller.gate.beta_align;
        if beta > 0
            w = max(0, -tau_dir0(:)'.*dq(:)');         % 1×n
            if sum(w) <= 1e-12, w = abs(tau_dir0(:)'.*dq(:)'); end
            if sum(w) > 0
                a_prog = w/sum(w);                     % 1×n
                alpha  = (1-beta)*alpha + beta*a_prog;
                alpha  = alpha / sum(alpha);
                % 依新 α 重新算幅值限
                P_axis     = min(alpha .* P_budget, Prated);
                tau_from_P = P_axis ./ dq_safe;
                tau_lim    = min(tau_from_P, tauMax);
            end
        end

        % === 8) 零空间重分配（不浪费在 EE 无关分量） ===
        N = null(Jlin);
        tau_dir = tau_dir0(:);
        if ~isempty(N)
            W = diag(1 ./ (tau_lim(:).^2 + 1e-12));
            A = (N.' * W * N);  b = -(N.' * W * tau_dir);
            if rcond(A) > 1e-8, z = A \ b; tau_dir = tau_dir + N*z; end
        end
        tau_dir = tau_dir.';   % 行向量

        % === 9) 逐轴夹紧 + 总功率等比（★关键修复） ===
        % 先逐轴夹紧，避免“最小比缩放”把整向量拖没
        tau_cmd = sign(tau_dir) .* min(abs(tau_dir), tau_lim);
        % 若总功率超预算，再整体等比
        P_cmd = sum(abs(tau_cmd .* dq));
        if P_cmd > P_budget && P_cmd > 0
            tau_cmd = tau_cmd * (P_budget / P_cmd);
        end

        % === 10) 重力补偿最后加回（不参与缩放） ===
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

% ================= helpers =================

function feat = local_features(featureFcn, t, q, dq, q_ref, dq_ref, robot, kin)
    try
        feat = featureFcn(t, q, dq, q_ref, dq_ref, robot, kin);
    catch
        try, feat = featureFcn(t, q, dq, q_ref, dq_ref, robot);
        catch,   feat = double([q(:); dq(:); q_ref(:); dq_ref(:)]).';
        end
    end
    feat = double(feat(:)).';    % 1×D 行向量
end

function pl4 = local_payload(robot)
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

function y = local_infer(model, xrow)
    x = double(xrow);
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
    if ismethod(net,'predict') || isprop(net,'Layers')
        y = local_try_predict_anyshape(net, x); return;
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
        try
            yi = predict(net, cand{i});
            y  = local_rowify(yi); 
            return;
        catch ME
            lastErr = ME.message;
        end
    end
    error('predict(net,X) failed for all shapes. Last: %s', lastErr);
end

function y = local_rowify(yi)
    if isa(yi,'gpuArray'), yi = gather(yi); end
    y = double(yi(:)).';
end

function [tau0, f, Jlin] = local_task_pd_robot(robot, q, dq, q_ref, Kx, Dx)
    J  = local_J(robot, q);
    Jlin = J(4:6,:);
    T  = getTransform(robot, q,     'tool');
    Tg = getTransform(robot, q_ref, 'tool');
    x  = T (1:3,4);  xg = Tg(1:3,4);
    vx = Jlin * dq(:);
    f  = Kx*(xg - x) + Dx*(0 - vx);
    tau0 = (Jlin.' * f).';
end

function J = local_J(robot, q)
    ee = 'tool';
    try
        J = geometricJacobian(robot, q, ee);
    catch
        names = robot.BodyNames; 
        ee = names{end}; 
        J = geometricJacobian(robot, q, ee);
    end
end

function d = local_dist_ee_robot(robot, q, q_ref)
    try
        T1 = getTransform(robot, q,     'tool');
        T2 = getTransform(robot, q_ref, 'tool');
        d = norm(T1(1:3,4) - T2(1:3,4));
    catch
        d = NaN;
    end
end
