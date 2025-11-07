% SPINN3D_COMPARE_V2 —— 一键对比：PID vs NN_v2（输出首次到达时间与速度）
clear; clc;
if ~(exist('params','var')&&exist('sim','var')&&exist('caps','var')&&exist('Kp','var')&&exist('Kd','var')&&exist('Ki','var'))
    run('spinn3d_params_block.m');
end
if ~(exist('MODEL_MAT','var')&&exist(MODEL_MAT,'file')==2)
    error('未找到 NN 模型文件：%s（请先运行 spinn3d_run_v2 训练）', MODEL_MAT);
end
if ~exist('gate','var'), gate = struct('enable', true, 'Pmin', 20, 'kE', 5.0); end

[robot, kin] = spinn3d_robot(params);
opts_fig = struct('q_goal', sim.q_ref, 'goal_radius',0.03, 'Pmax', caps.Pmax, ...
                  'trim_t0',0.02, 'robust_pct',0.995, 'show_raw', false);
eval_opts = struct('tolW',1.0,'decim',5);
reach_opts = struct('goal_radius',0.03,'q_eps_deg',0.5,'w_eps_deg',0.1,'hold_s',0.2);

%% PID
pid = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);
f_pid = @(t,q,dq,qr,dqr,robot) pid.step(t,q,dq,qr,dqr,robot);
log_pid = spinn3d_simulate(robot, f_pid, sim);
spinn3d_power_check(log_pid);
spinn3d_static_figure(kin, log_pid, opts_fig);
rptE_pid = try_energy_check(robot, log_pid, sim.q_ref, struct('enable',false), caps, eval_opts);
Rpid = spinn3d_reach_metrics(robot, kin, log_pid, sim, reach_opts);

%% NN_v2
model = load(MODEL_MAT);
pid_ref = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);
nnctl   = spinn3d_controller_nn_v2(model, @spinn3d_features_v2, caps, gate);
f_nn = @(t,q,dq,qr,dqr,robot) nnctl.step(t,q,dq,qr,dqr,robot,kin,pid_ref);
log_nn = spinn3d_simulate(robot, f_nn, sim);
spinn3d_power_check(log_nn);
spinn3d_static_figure(kin, log_nn, opts_fig);
rptE_nn = try_energy_check(robot, log_nn, sim.q_ref, gate, caps, eval_opts);
Rnn = spinn3d_reach_metrics(robot, kin, log_nn, sim, reach_opts);

%% 汇总打印
fprintf('\n=== First Arrival (EE sphere r=%.3f m) & Joint thresholds (%.1f deg / %.1f deg/s) ===\n', ...
        reach_opts.goal_radius, reach_opts.q_eps_deg, reach_opts.w_eps_deg);
fmt1 = '%-8s | t_EE=%.3f s, v_EE=%.3f m/s  | t_Q=%.3f s, |dq|=%.3f deg/s  | hold_EE=%.3f s  hold_Q=%.3f s\n';
fprintf(fmt1, 'PID',   nanz(Rpid.ee_first_t), nanz(Rpid.ee_first_v), nanz(Rpid.q_first_t), rad2deg(nanz(Rpid.q_first_w)), nanz(Rpid.ee_hold_t), nanz(Rpid.q_hold_t));
fprintf(fmt1, 'NN_v2', nanz(Rnn.ee_first_t),  nanz(Rnn.ee_first_v),  nanz(Rnn.q_first_t),  rad2deg(nanz(Rnn.q_first_w)),  nanz(Rnn.ee_hold_t),  nanz(Rnn.q_hold_t));

fprintf('\n=== Energy & Power Stats ===\n');
prt_energy('PID',  rptE_pid);
prt_energy('NN_v2',rptE_nn);

save('spinn3d_compare_v2.mat','log_pid','log_nn','rptE_pid','rptE_nn','Rpid','Rnn','params','caps','sim','gate');
fprintf('Saved: spinn3d_compare_v2.mat\n');

%% helpers
function rpt = try_energy_check(robot, log, qref, gate, caps, opts)
try
    rpt = spinn3d_energy_check(robot, log, qref, gate, caps, opts);
catch
    rpt = spinn3d_energy_check(robot, log, qref, gate, caps);
end
end

function prt_energy(name, rpt)
if isfield(rpt,'monotone_ratio_raw'), raw=rpt.monotone_ratio_raw; else, raw=rpt.monotone_ratio; end
if isfield(rpt,'monotone_ratio_decim'), dec=rpt.monotone_ratio_decim; else, dec=raw; end
if isfield(rpt,'p_over_ratio') && ~isempty(rpt.p_over_ratio) && ~isnan(rpt.p_over_ratio)
    pov = sprintf('%.1f%%', 100*max(0,rpt.p_over_ratio));
else
    pov = 'n/a';
end
fprintf('%-8s | dH mono raw/decim: %5.1f%% / %5.1f%%  | P<=Pbud viol: %s\n', name, 100*raw, 100*dec, pov);
end

function y = nanz(v), if isempty(v) || isnan(v), y=NaN; else, y=v; end, end
