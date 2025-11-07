function R = spinn3d_reach_metrics(robot, kin, log, sim, opts)
% SPINN3D_REACH_METRICS
% 计算“第一次到达目标区域”的时间与速度（关节域 & 末端域），并给出保持时间版本。
% - 末端域：以 q_ref 对应末端位置为目标中心，半径 = opts.goal_radius (m)
% - 关节域：||q-q_ref|| ≤ q_eps 且 ||dq|| ≤ w_eps
% - 保持（可选）：持续 time >= opts.hold_s 则给出 hold_time
%
% 输入：
%   robot : rigidBodyTree
%   kin   : (可选) 结构体，用于推断末端名，若含 eeName 则优先使用
%   log   : 仿真日志，需含 t, q, dq
%   sim   : 需含 q_ref (1xN)
%   opts  : struct 可选字段：
%           .goal_radius  (默认 0.03 m)
%           .q_eps_deg    (默认 0.5 deg)
%           .w_eps_deg    (默认 0.1 deg/s)
%           .hold_s       (默认 0.2 s)
%
% 输出 R：结构体（字段见下）
%
% 兼容性：若 geometricJacobian 不可用，末端速度用数值差分近似。

if nargin < 5, opts = struct(); end
if ~isfield(opts,'goal_radius'), opts.goal_radius = 0.03; end
if ~isfield(opts,'q_eps_deg'),   opts.q_eps_deg   = 0.5;  end
if ~isfield(opts,'w_eps_deg'),   opts.w_eps_deg   = 0.1;  end
if ~isfield(opts,'hold_s'),      opts.hold_s      = 0.2;  end

q_goal = double(sim.q_ref(:).');
q_eps  = deg2rad(opts.q_eps_deg);
w_eps  = deg2rad(opts.w_eps_deg);

% === 推断末端名 ===
eeName = '';
if nargin>=2 && isstruct(kin) && isfield(kin,'eeName') && ~isempty(kin.eeName)
    eeName = kin.eeName;
elseif ~isempty(robot.Bodies)
    eeName = robot.Bodies{end}.Name;
else
    eeName = 'end_effector';  %#ok<NASGU>
end

N  = numel(log.t);
dt = mean(diff(log.t));
if ~isfinite(dt) || dt <= 0, dt = 0.002; end

% 目标末端位置
Tg  = getTransform(robot, q_goal, eeName);
xg  = Tg(1:3,4);

% 逐步计算末端位置 & 速度
x  = zeros(N,3);
vq = zeros(N,1);  % joint speed norm
for k=1:N
    qk = double(log.q(k,:));
    dqk= double(log.dq(k,:));
    Tk = getTransform(robot, qk, eeName);
    x(k,:) = Tk(1:3,4).';
    vq(k)  = norm(dqk);
end

% 优先用 Jacobian 线速度
v = zeros(N,1);
useJ = true;
try
    for k=1:N
        qk = double(log.q(k,:));
        dqk= double(log.dq(k,:)).';
        J  = geometricJacobian(robot, qk, eeName);  % 6xn
        vlin = J(4:6,:)*dqk;
        v(k) = norm(vlin);
    end
catch
    useJ = false;
    % 数值差分线速度
    dx = [zeros(1,3); diff(x)];
    v  = sqrt(sum(dx.^2,2)) / dt;
end

% 距离与阈值
dEE = sqrt(sum((x - xg.').^2, 2));
inEE = (dEE <= opts.goal_radius);
inQ  = (vecnorm(log.q - q_goal, 2, 2) <= q_eps) & (vq <= w_eps);

% 第一次进入
idx_first_ee = find(inEE, 1, 'first');
idx_first_q  = find(inQ,  1, 'first');

% 保持窗口
win = max(1, ceil(opts.hold_s / dt));
idx_hold_ee = NaN; idx_hold_q = NaN;
for k=1:N-win
    if all(inEE(k:k+win)), idx_hold_ee = k; break; end
end
for k=1:N-win
    if all(inQ(k:k+win)), idx_hold_q = k; break; end
end

% ===== 输出（用 if-else 防止 NaN/[] 索引在函数实参阶段被提前求值）=====
R = struct();

% EE first
if isempty(idx_first_ee)
    R.ee_first_t = NaN; R.ee_first_v = NaN;
else
    R.ee_first_t = log.t(idx_first_ee);
    R.ee_first_v = v(idx_first_ee);
end

% EE hold
if isnan(idx_hold_ee)
    R.ee_hold_t = NaN; R.ee_hold_v = NaN;
else
    R.ee_hold_t = log.t(idx_hold_ee);
    R.ee_hold_v = v(idx_hold_ee);
end

% Q first
if isempty(idx_first_q)
    R.q_first_t = NaN; R.q_first_w = NaN;
else
    R.q_first_t = log.t(idx_first_q);
    R.q_first_w = vq(idx_first_q);
end

% Q hold
if isnan(idx_hold_q)
    R.q_hold_t = NaN; R.q_hold_w = NaN;
else
    R.q_hold_t = log.t(idx_hold_q);
    R.q_hold_w = vq(idx_hold_q);
end

R.goal_radius  = opts.goal_radius;
R.q_eps_deg    = opts.q_eps_deg;
R.w_eps_deg    = opts.w_eps_deg;
R.hold_s       = opts.hold_s;
R.useJacobian  = useJ;
R.best_dEE     = min(dEE);
R.best_q_err   = min(vecnorm(log.q - q_goal, 2, 2));
R.best_w       = min(vq);

% 辅助打印
R.summary = sprintf(['EE first: t=%.3f s, v=%.3f m/s;  hold: t=%.3f s, v=%.3f m/s  |  ' ...
                     'Q first: t=%.3f s, |dq|=%.3f deg/s; hold: t=%.3f s, |dq|=%.3f deg/s'], ...
                     nanz(R.ee_first_t), nanz(R.ee_first_v), nanz(R.ee_hold_t), nanz(R.ee_hold_v), ...
                     nanz(R.q_first_t), rad2deg(nanz(R.q_first_w)), nanz(R.q_hold_t), rad2deg(nanz(R.q_hold_w)));
end

function y = tern(c,a,b), if c, y=a; else, y=b; end, end
function x = nanz(v), if isempty(v) || isnan(v), x=NaN; else, x=v; end, end
