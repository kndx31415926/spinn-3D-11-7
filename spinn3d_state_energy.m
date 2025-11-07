function S = spinn3d_state_energy(robot, q, dq)
%SPINN3D_STATE_ENERGY  计算系统总能量 H = T + U
% 修复点：
%   1) U 使用每个刚体的质心 COM：pCOM = p0 + R*CenterOfMass
%   2) 强制将 q/dq 转为 double，避免 RST 接口 single 类型报错

    % -------- 0) 形状与类型处理 --------
    if size(q,1) ~= 1 && size(q,2) ~= 1
        error('q 必须是向量（1xN 或 Nx1）。');
    end
    if size(dq,1) ~= 1 && size(dq,2) ~= 1
        error('dq 必须是向量（1xN 或 Nx1）。');
    end
    % 统一为 1xN，且强制 double（RST 要求）
    q  = double(q(:).');
    dq = double(dq(:).');

    if isempty(robot.Gravity)
        robot.Gravity = [0 0 -9.81];
    end
    g = double(robot.Gravity(:));   % 3x1

    % -------- 1) 动能 T = 0.5 * dq' * M(q) * dq --------
    M = massMatrix(robot, q);         % NxN, double
    dq_col = dq(:);                   % Nx1
    T = 0.5 * (dq_col.' * M * dq_col);

    % -------- 2) 势能 U（每个刚体 COM 求和）--------
    U = 0.0;
    nb = numel(robot.Bodies);
    U_parts = zeros(1, nb);
    com_W   = zeros(3, nb);

    for i = 1:nb
        bi = robot.Bodies{i};
        if bi.Mass <= 0
            continue;
        end
        % 世界坐标下 body 变换
        Ti = getTransform(robot, q, bi.Name);  % 4x4，world <- body
        R  = Ti(1:3,1:3);
        p0 = Ti(1:3,4);
        % body 坐标 COM
        cB = bi.CenterOfMass(:);               % 3x1
        % 世界坐标 COM
        pCOM = p0 + R * cB;

        % 势能：与 gravityTorque 口径一致
        Ui = - bi.Mass * (g.' * pCOM);

        U = U + Ui;
        U_parts(i) = Ui;
        com_W(:,i) = pCOM;
    end

    % -------- 3) 总能 --------
    H = T + U;

    % -------- 4) 输出 --------
    S = struct();
    S.T = double(T);
    S.U = double(U);
    S.H = double(H);
    S.g = g;

    S.details = struct('U_parts', U_parts, 'com_W', com_W);
end
