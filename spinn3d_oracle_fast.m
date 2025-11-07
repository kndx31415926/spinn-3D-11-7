function [alpha_star, g_star, dbg] = spinn3d_oracle_fast(q, dq, q_ref, dq_ref, robot, kin, caps, gate, pid)
% 生成“首达更快”风格的监督标签 (alpha*, g*).
% 约束与在线一致：逐轴/总功率上限，功率在积分前按 dq(t) 口径计算。
% 输出：
%   alpha_star : 1×n, 单纯形（Σalpha=1）
%   g_star     : 标量∈[0,1]
%   dbg        : 调试信息（可选）

    % ---- 预处理 ----
    q  = q(:)'; dq = dq(:)'; q_ref = q_ref(:)'; dq_ref = dq_ref(:)'; 
    n  = numel(q); 
    epsw = 1e-6;

    % ---- 1) 方向扭矩（与在线一致；默认 PID）----
    [tau_pd, ~] = pid.step(0, q, dq, q_ref, dq_ref, robot);
    try
      tau_gc = gravityTorque(robot, q);
    catch
      tau_gc = zeros(1, numel(q));
    end
    tau_ref = (tau_pd - tau_gc);     % ← 方向扭矩 = 去重力
    tau_ref = tau_ref(:)';
    % ---- 2) 能量门控上界 g_H（兼容 2/3 输出）----
    gH = 1.0;
    try
        [~, gHtmp, ~] = spinn3d_energy_gate(robot, q, dq, q_ref, caps, gate);
    catch
        try, [~, gHtmp] = spinn3d_energy_gate(robot, q, dq, q_ref, caps, gate);
        catch, gHtmp = [];
        end
    end
    if ~isempty(gHtmp), gH = min(max(gHtmp, 0), 1); end
    g_star = gH;

    % ---- 3) 推进权重：吸能/减势口径（与推理一致） ----
    w = max(0, -tau_ref .* dq);
    if sum(w) <= 1e-9,  w = abs(tau_ref .* dq); end
    if sum(w) <= 1e-12, w = ones(1, n);        end
    alpha_init = w / sum(w);

    % ---- 4) 预算分配与逐轴裁剪 ----
    Pbud    = g_star * caps.Pmax;              % 总功率预算
    Prated  = expand_(caps.Prated, n);
    tauMax  = expand_(caps.tauMax, n);

    % ★ C：与在线一致的近零速地板
    dq_floor = 0;
    if isfield(caps,'dq_floor') && ~isempty(caps.dq_floor), dq_floor = caps.dq_floor; end
    dq_safe = max(abs(dq), dq_floor) + epsw;   % 不能写成 abs(dq+epsw)

    % 初始分配（受额定功率上限）
    Paxis0 = min(alpha_init * Pbud, Prated);

    % 第一次裁剪：受扭矩上限（功率等效）约束
    perAxisCap = min(Prated, tauMax .* dq_safe);
    Paxis1     = min(Paxis0, tauMax .* dq_safe);

    % 被裁掉的预算（可回填）
    freed = sum(Paxis0) - sum(Paxis1);

    % ---- 5) 预算回填（严格不越界）----
    Paxis = Paxis1;
    room  = zeros(1, n);
    if freed > 1e-12
        room = perAxisCap - Paxis;
        mask = room > 1e-12;
        if any(mask)
            w2 = w .* mask; 
            s2 = sum(w2);
            if s2 <= 1e-12
                w2 = mask / max(1, sum(mask));  % 纯均分兜底
            else
                w2 = w2 / s2;
            end
            add = min(freed * w2, room);        % 回填不超过剩余空间
            Paxis = Paxis + add;
        end
    end

    % ---- 6) 导出标签 alpha*（单纯形）----
    alpha_star = Paxis / max(Pbud, 1e-12);
    s = sum(alpha_star);
    if s <= 0
        alpha_star = ones(1, n) / n;
    else
        alpha_star = alpha_star / s;
    end

    % ---- 调试信息 ----
    if nargout > 2
        dbg = struct();
        dbg.w          = w;
        dbg.alpha_init = alpha_init;
        dbg.Pbud       = Pbud;
        dbg.Prated     = Prated;
        dbg.tauMax     = tauMax;
        dbg.dq_safe    = dq_safe;
        dbg.Paxis0     = Paxis0;
        dbg.Paxis1     = Paxis1;
        dbg.perAxisCap = perAxisCap;
        dbg.freed      = freed;
        dbg.room       = room;
        dbg.Paxis      = Paxis;
        dbg.tau_ref    = tau_ref;
    end
end

function y = expand_(x, n)
    if isempty(x), y = inf(1, n); return; end
    if numel(x) == 1, y = repmat(x, 1, n); else, y = x(:)'; end
end
