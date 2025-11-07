% SPINN3D_PARAMS_BLOCK  —— 统一参数块（含 SPINN 能量门控 & 末端负载）

% === 机器人参数 ===
params = struct();
params.L           = [0.24 0.214 0.324];
params.gravity     = [0 0 -9.81];
params.base_mass   = 6.0;
params.base_radius = 0.15;
params.base_height = 0.04;

% （可选）连杆质量，如不设置 spinn3d_robot 会用默认值
% params.mass      = [2.8 2.2 1.6];     % [shoulder, elbow, wrist] [kg]
% （可选）连杆 COM 比例，如不设置 spinn3d_robot 会用默认值
% params.com_ratio = [0.5 0.5 0.5];

% === 末端负载（payload）——默认表达“只有夹爪自重”（用于推理/演示）===
% 说明：这里恒定表示“夹爪”本体；无工件时也计入动力学。
% 若抓取工件，请把 mass/com/I6_com 或 shape+dims 更新为“夹爪+工件”的合成值，再调用 spinn3d_robot 生成新模型。
params.payload = struct();
params.payload.mass = 0.45;              % 夹爪自重 [kg]
params.payload.com  = [0.03 0 0];        % ← 夹爪质心在 tool 坐标系下 [m]

% 二选一提供“在 COM 处”的惯量（kg·m^2）：
% 方式 A：直接给 I6_com（优先采用）
% params.payload.I6_com = [Ixx Iyy Izz Iyz Ixz Ixy];

% 方式 B：没有 I6_com 就用几何近似（本脚本按盒体近似）
params.payload.shape = 'box';            % 'box' | 'cyl_x' | 'cyl_z' | 'sphere'
params.payload.dims  = struct('a',0.06,'b',0.04,'c',0.03);  % 夹爪外形尺寸 [m]

% === 训练用：末端负载采样范围（payload_cfg；单位 SI；生成器读取）===
% 只采样“质量 + 质心”，不采样惯量；惯量由 shape+dims + mass 自动推得并平移。
payload_cfg = struct();
payload_cfg.mass_range = [0.2, 1.5];                         % [kg] 末端质量范围（含“无工件”）
payload_cfg.com_range  = [0.02 0.06;  -0.01 0.01;  -0.02 0.02]; % [m] COM 在 tool 系下的 XYZ 范围
payload_cfg.shape = params.payload.shape;                     % 与默认夹爪形状一致
payload_cfg.dims  = params.payload.dims;                      % 与默认夹爪外形一致

% === PID 增益 ===
Kp = [20 20 20 20];
Ki = [5 5 5 5];
Kd = [10 10 10 10];
pid_opts = struct('useGravityComp',true, 'uMax',[120 90 70 50], 'antiWindup',true, 'Imax',2);

% === 功率/扭矩上限 ===
caps = struct();
caps.Pmax   = 100;                     % 总功率上限 [W]
caps.Prated = [100 100 100 60];        % 逐轴额定功率 [W]
caps.tauMax = [120  90  70  50];       % 逐轴扭矩上限 [Nm]
caps.useTotalPowerCap = true;
caps.useAxisPowerCap  = true;
caps.dq_floor = 0.2;                   % 近零速速度地板 [rad/s]（按需调整）

% === SPINN 能量门控（训练监督与推理同参） ===
gate = struct('enable', true, 'Pmin', 20, 'kE', 5.0);   % 近目标自动收权
gate.goal_radius = 0.02;               % [m] 末端空间到达判定半径

% === 仿真参数 ===
sim = struct();
sim.goal_radius = gate.goal_radius;    % 与 gate 统一
sim.stop_on_first_hit = true;          % 首达即停（演示/采样用）
sim.dt      = 0.002;
sim.t_final = 20;
sim.q0      = deg2rad([0 0 0 0]);
sim.dq0     = [0 0 0 0];
sim.q_ref   = deg2rad([30, 20, 35, 60]);
sim.dq_ref  = [0 0 0 0];
sim.damping = [0.06 0.05 0.05 0.04];
sim.jointLimit = struct('deadband_deg',0.5,'freezeInward',true);
sim.integrator = 'semi';
sim.caps = caps;

% === 模型文件 ===
MODEL_MAT = 'spinn3d_v2_net_fast.mat'
