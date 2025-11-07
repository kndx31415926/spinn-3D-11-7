% SPINN3D_DEMO_PID_CFG —— 基于统一参数块的 PID 基线（无动画，且不显示 P_raw）
clear; clc;
if ~(exist('params','var')&&exist('sim','var')&&exist('caps','var')&&exist('Kp','var')&&exist('Kd','var')&&exist('Ki','var'))
    run('spinn3d_params_block.m');
end

[robot, kin] = spinn3d_robot(params);
pid = spinn3d_controller_pid(Kp, Ki, Kd, pid_opts);

controllerFcn = @(t,q,dq,qr,dqr,robot) pid.step(t,q,dq,qr,dqr,robot);
log = spinn3d_simulate(robot, controllerFcn, sim);
rpt = spinn3d_power_check(log); %#ok<NASGU>

opts_fig = struct('q_goal', sim.q_ref, 'goal_radius',0.03, 'Pmax', caps.Pmax, ...
                  'trim_t0',0.02,'robust_pct',0.995,'show_raw', false);  % PID 不显示 P_raw
stats = spinn3d_static_figure(kin, log, opts_fig); %#ok<NASGU>
