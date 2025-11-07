% SPINN3D_DEMO_NN_V2 —— NN_v2 推理（首达即停 + 输出功率表）
clear; clc;
if ~(exist('params','var')&&exist('sim','var')&&exist('caps','var')&&exist('Kp','var')&&exist('Kd','var')&&exist('Ki','var')&&exist('MODEL_MAT','var')&&exist('gate','var'))
    run('spinn3d_params_block.m');   % 这里面已有 sim.stop_on_first_hit / goal_radius
end
assert(exist(MODEL_MAT,'file')==2, '未找到模型文件：%s', MODEL_MAT);

model = load(MODEL_MAT);
[robot, kin] = spinn3d_robot(params);
pid  = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);
nnctl= spinn3d_controller_nn_v2(model, @spinn3d_features_v2, caps, gate);

% 关键：把 kin 交给 simulate 做笛卡尔判定
sim.kin = kin;
sim.stop_on_first_hit = true;    % 如需看完整 t_final，可改为 false

controllerFcn = @(t,q,dq,qr,dqr,robot) nnctl.step(t,q,dq,qr,dqr,robot,kin,pid);
[log, metrics] = spinn3d_simulate(robot, controllerFcn, sim);   % 两个输出！

spinn3d_power_check(log);

if metrics.reached
    fprintf('[HIT] t_hit = %.4f s, v_hit = %.4f (m/s)\n', metrics.t_hit, metrics.v_hit);
else
    fprintf('[HIT] 未到达（增加 sim.t_final 或放宽 sim.goal_radius）\n');
end

% 预览功率表
nshow = min(5, height(metrics.P_total_table));
disp(metrics.P_total_table(1:nshow,:));
nshow = min(5, height(metrics.P_axis_table));
disp(metrics.P_axis_table(1:nshow,:));

% 出图：只画到首达
opts_fig = struct('q_goal', sim.q_ref, 'goal_radius', sim.goal_radius, ...
                  'trim_t0',0.02,'robust_pct',0.995,'show_raw', false, ...
                  'trim_after_hit', true, 'Pmax', caps.Pmax);
spinn3d_static_figure(kin, log, opts_fig);
