%% ===== 可配参数 =====
N_TRAJ_ADD     = 300;       % 本次新增轨迹条数
CHUNK_TRAJ     = 10;       % 每分片轨迹条数
DO_TRAIN_AFTER = true;    % 生成后立刻训练
DT_GEN         = 0.0025;   % 生成用初始 dt（失败会自动降到 0.005 再试）
T_FINAL_GEN    = 6.0;      % 每条轨迹时长

%% ===== 输出位置（与原 run_v2 平行的独立目录） =====
MASTER_DIR         = 'data_fast';
CHK_DIR            = fullfile(MASTER_DIR,'chk');
MASTER_TBL_VAR     = 'SPINN3D_FAST_DATASET';
MASTER_TBL_FILE    = fullfile(MASTER_DIR,'spinn3d_fast_dataset_table_master.mat');
MASTER_DATASET_MAT = fullfile(MASTER_DIR,'spinn3d_fast_dataset_master.mat'); % 聚合 X/Y
MODEL_MAT          = fullfile(MASTER_DIR,'spinn3d_v2_net_fast.mat');         % 训练输出

ensure_dir(MASTER_DIR); ensure_dir(CHK_DIR);

%% ===== 前置检查 =====
must_exist({'spinn3d_oracle_fast.m','spinn3d_features_v2.m', ...
            'spinn3d_controller_pid.m','spinn3d_simulate.m','spinn3d_params_block.m'});

%% ===== 公共参数（由参数块提供） =====
run spinn3d_params_block   % 提供 params, caps, Kp/Ki/Kd, pid_opts, sim, gate, payload_cfg 等
% ↑ payload_cfg: 训练用“质量+COM”采样范围（单位 SI），由参数块统一给出。

% 数据生成采样范围（单位：度，用于 q0/qref）
simgen_base = struct('dt', DT_GEN, 't_final', T_FINAL_GEN, ...
                     'Kp', Kp, 'Ki', Ki, 'Kd', Kd, 'pid_opts', pid_opts, ...
                     'damping', sim.damping, ...
                     'q0_range',   [-30 30; -20 20; -20 20; -20 20], ...
                     'qref_range', [ 15 45;  10 30; -40 -10;  10 40]);

%% ===== 载入历史主表与聚合 X/Y =====
[T_master, Xagg, Yagg] = load_master(MASTER_TBL_FILE, MASTER_TBL_VAR, MASTER_DATASET_MAT);
N0_hist = height(T_master);

fprintf('== [1/3] 生成数据: 计划 N_TRAJ=%d, dt=%.3f, T=%.1f, 每 %d 条保存一次 ==\n', ...
        N_TRAJ_ADD, DT_GEN, T_FINAL_GEN, CHUNK_TRAJ);

t0 = tic; added_rows_total = 0;
num_chunks = ceil(N_TRAJ_ADD / CHUNK_TRAJ);

%% ===== 分片循环 =====
for c = 1:num_chunks
    curN = min(CHUNK_TRAJ, N_TRAJ_ADD - (c-1)*CHUNK_TRAJ);
    if curN <= 0, break; end

    % === 生成一个分片（健壮版：凑满 curN 条才返回） ===
    simgen = simgen_base; simgen.nTraj = curN;
    ds = gen_chunk_fast(simgen, params, caps, gate, payload_cfg);  % ← 增加 payload_cfg

    % === 分片落盘 ===
    tag = datestr(now,'yyyymmdd-HHMMSS');
    shard_file = fullfile(CHK_DIR, sprintf('spinn3d_fast_dataset-chunk%03d-%s.mat', c, tag));
    save(shard_file, '-struct', 'ds', '-v7.3');

    % === 追加到主表 & 聚合 X/Y（与 run_v2 一致） ===
    [T_master, Xagg, Yagg, M_new] = append_to_master(T_master, Xagg, Yagg, ds);
    added_rows_total = added_rows_total + M_new;

    % === 保存主表/聚合（每片一次） ===
    SPINN3D_FAST_DATASET = T_master; %#ok<NASGU>
    save(MASTER_TBL_FILE, 'SPINN3D_FAST_DATASET', '-v7.3');

    X = Xagg; Y = Yagg; %#ok<NASGU>
    save(MASTER_DATASET_MAT, 'X','Y','caps','params','gate','-v7.3');

    % === 进度 ===
    elapsed = toc(t0);
    eta = elapsed / c * (num_chunks - c);
    fprintf('[%d/%d] +%d 轨迹 -> 样本 +%d 行, 累计 %d 行 | 用时 %.1fs | 预计剩余 %.1fs\n', ...
            c, num_chunks, curN, M_new, height(T_master), elapsed, eta);
end

%% ===== 汇总 =====
fprintf('== [2/3] 追加完成：历史 %d 行 + 新增 %d 行 -> 现有 %d 行 ==\n', ...
        N0_hist, added_rows_total, height(T_master));
fprintf('已保存表到：%s\n', MASTER_TBL_FILE);
fprintf('已保存聚合数据到：%s（X: %d×%d, Y: %d×%d）\n', ...
        MASTER_DATASET_MAT, size(Xagg,1), size(Xagg,2), size(Yagg,1), size(Yagg,2));

%% ===== (可选) 训练 =====
if DO_TRAIN_AFTER
    fprintf('== [3/3] 训练模型 ==\n');
    try
        spinn3d_train_fast_hit(MASTER_DATASET_MAT, MODEL_MAT);
    catch
        spinn3d_train_fast_hit();
    end
    fprintf('模型已保存：%s\n', MODEL_MAT);
else
    fprintf('== [3/3] 跳过训练（DO_TRAIN_AFTER=%s） ==\n', string(DO_TRAIN_AFTER));
end

fprintf('完成。\n');

%% ================= 本地函数 =================

function ensure_dir(d)
    if exist(d,'dir')~=7, mkdir(d); end
end

function must_exist(list)
    for i=1:numel(list)
        assert(exist(list{i},'file')==2, '缺少依赖：%s', list{i});
    end
end

function [T, Xagg, Yagg] = load_master(tbl_file, tbl_var, agg_file)
    T = table(); Xagg = []; Yagg = [];
    if exist(tbl_file,'file')
        S = load(tbl_file); if isfield(S, tbl_var), T = S.(tbl_var); end
    end
    if exist(agg_file,'file')
        S2 = load(agg_file);
        if isfield(S2,'X'), Xagg = S2.X; end
        if isfield(S2,'Y'), Yagg = S2.Y; end
    end
end

function [T, Xagg, Yagg, M_new] = append_to_master(T, Xagg, Yagg, ds)
    [M_new, D] = size(ds.X);  K = size(ds.Y,2);
    xnames = strcat("x", string(1:D));
    % Y 组织：alpha1..alpha_n, g, w1..wn
    n = (K-1)/2;
    ynames = [ "alpha"+string(1:n), "g", "w"+string(1:n) ];
    Tnew = array2table([ds.X ds.Y], 'VariableNames', [cellstr(xnames) cellstr(ynames)]);
    if isempty(T), T = Tnew; else
        assert(width(T)==width(Tnew), '表结构不一致（历史=%d, 新=%d）。', width(T), width(Tnew));
        T = [T; Tnew]; %#ok<AGROW>
    end
    if isempty(Xagg), Xagg = ds.X; else, Xagg = [Xagg; ds.X]; end %#ok<AGROW>
    if isempty(Yagg), Yagg = ds.Y; else, Yagg = [Yagg; ds.Y]; end %#ok<AGROW>
    assignin('base','SPINN3D_FAST_DATASET',T);
end

function ds = gen_chunk_fast(simgen, params, caps, gate, payload_cfg)
    % 生成 simgen.nTraj 条有效轨迹，返回 ds.X (M×D), ds.Y (M×(2n+1))
    % 相比旧版：每条轨迹都会采样 payload（质量+COM），并把 [m, rx, ry, rz] 拼到特征里。
    pid   = spinn3d_controller_pid(simgen.Kp, simgen.Ki, simgen.Kd, simgen.pid_opts);  % PID 基线

    n = 4;
    X = [];                      % 动态确定列数（基特征 + 4）
    Y = zeros(0, 2*n+1);

    want  = simgen.nTraj;
    got   = 0;
    tries = 0;
    maxTries = 20*want;  % 最多尝试次数（避免死循环）

    while got < want && tries < maxTries
        tries = tries + 1;

        % —— 本条轨迹：采样 payload（质量 + 质心）——
        m_pl = payload_cfg.mass_range(1) + (payload_cfg.mass_range(2)-payload_cfg.mass_range(1))*rand(1,1);
        r_pl = rand_range(payload_cfg.com_range);   % [rx ry rz]（单位 m）
        params_iter = params;
        params_iter.payload = struct('mass', m_pl, 'com', r_pl, ...
                                     'shape', payload_cfg.shape, 'dims', payload_cfg.dims);

        % —— 重建 robot，使本条轨迹的 M/C/G 体现该 payload ——
        [robot, kin] = spinn3d_robot(params_iter);
        robot.DataFormat = 'row';
        robot = sanitize_rbt(robot);   % 仅对非 fixed 体做最小质量/惯量兜底

        sim0 = struct();
        sim0.dt      = simgen.dt;
        sim0.t_final = simgen.t_final;
        sim0.damping = simgen.damping;
        sim0.caps    = caps;

        % 随机初值/目标（度→弧度）
        sim0.q0     = deg2rad(rand_range(simgen.q0_range));
        sim0.dq0    = zeros(1,n);
        sim0.q_ref  = deg2rad(rand_range(simgen.qref_range));
        sim0.dq_ref = zeros(1,n);

        ctl = @(t,q,dq,qr,dqr,robot) pid.step(t,q,dq,qr,dqr,robot);

        % —— 仿真：失败重试（降步长） ——
        ok = false;
        try
            log = spinn3d_simulate(robot, ctl, sim0);
            ok  = is_log_valid(log);
        catch
            ok = false;
        end
        if ~ok
            sim0.dt = min(0.005, 0.5*simgen.dt);
            try
                log = spinn3d_simulate(robot, ctl, sim0);
                ok  = is_log_valid(log);
            catch
                ok = false;
            end
        end
        if ~ok, continue; end

        % —— 累加样本（把 payload 常量特征广播到整条轨迹） ——
        feat_pl = [m_pl, r_pl];    % 只要“质量+质心”（4 维）

        for k = 1:numel(log.t)
            qk  = log.q(k,:);  dqk = log.dq(k,:);
            qr  = sim0.q_ref;  dqr = sim0.dq_ref;

            f  = spinn3d_features_v2(log.t(k), qk, dqk, qr, dqr, robot, kin);
            xk = [f, feat_pl];          % ← 拼上 [mass, rx, ry, rz]

            [aStar, gStar] = spinn3d_oracle_fast(qk, dqk, qr, dqr, robot, kin, caps, gate, pid);
            [tau_pd, ~] = pid.step(0, qk, dqk, qr, dqr, robot);   % PID 输出（含重力补偿）
            try
                tau_gc = gravityTorque(robot, qk);
            catch
                tau_gc = zeros(1, numel(qk));
            end
            tau_fb = tau_pd - tau_gc;                             % 去重力后的名义方向扭矩
            w = max(0, -tau_fb(:)'.*dqk(:)');                     % 吸能/减势口径

        % —— 与 Oracle 相同的两级兜底，避免全零权重 —— 
        if sum(w) <= 1e-9
          w = abs(tau_fb(:)'.*dqk(:)');
        end
        if sum(w) <= 1e-12
          w = ones(1, numel(qk));
        end

       % 其余保持不变
       % X(end+1,:) = xk;
       % Y(end+1,:) = [aStar, gStar, w];

            if isempty(X), X = zeros(0, numel(xk)); end  % 首次确定列数
            X(end+1,:) = xk;                 %#ok<AGROW>
            Y(end+1,:) = [aStar, gStar, w];  %#ok<AGROW>
        end

        got = got + 1;
    end

    if got < want
        warning('本分片仅生成 %d/%d 条有效轨迹（其余尝试均数值不稳被丢弃）。', got, want);
    end

    ds = struct('X',X,'Y',Y);
end

function robot = sanitize_rbt(robot)
% 对非固定关节的叶子体做最低质量/惯量约束，避免 RBT 内部动力学返回 NaN/Inf。
    EPS_M  = 1e-3;    % kg
    EPS_I  = 1e-5;    % kg*m^2
    names = robot.BodyNames;
    for i=1:numel(names)
        b = robot.Bodies{i};
        if ~strcmpi(b.Joint.Type,'fixed')
            if ~isfinite(b.Mass) || b.Mass <= 0
                b.Mass = EPS_M;
            end
            I = b.Inertia;
            if numel(I)~=6 || any(~isfinite(I))
                I = [EPS_I EPS_I EPS_I 0 0 0];
            else
                I(1:3) = max(I(1:3), EPS_I);
            end
            b.Inertia = I;
        end
    end
end

function tf = is_log_valid(log)
    tf = isfield(log,'q') && isfield(log,'dq') && isfield(log,'tau') ...
        && all(isfinite(log.q(:))) && all(isfinite(log.dq(:))) && all(isfinite(log.tau(:)));
end

function v = rand_range(rr)
% rr: [n×2] 数值区间；返回 1×n，元素在各行区间内均匀采样
    lo = rr(:,1).'; hi = rr(:,2).';
    v  = lo + (hi-lo).*rand(1,numel(lo));
end
