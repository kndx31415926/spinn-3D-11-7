classdef fastHitLossLayer < nnet.layer.RegressionLayer
    % Fast-Hit:  L = MSE(alpha,g) + lambda * (-log(alpha·w + eps))
    % 兼容 Y,T 为 [B×R] 或 [R×B]，R = 2n+1，列顺序：[alpha(n), g, w(n)]

    properties
        N              % 关节数 n
        LambdaMSE = 1  % MSE 权重
        LambdaProg = .7% 进度项权重
        Eps = 1e-6
    end

    methods
        function layer = fastHitLossLayer(n, varargin)
            layer.Name = "fastHitLoss"; layer.N = n;
            for i=1:2:numel(varargin)
                layer.(varargin{i}) = varargin{i+1};
            end
        end

        function loss = forwardLoss(layer, Y, T)
            n = layer.N; R = 2*n + 1;
            [Ybr, ~] = local_toBR(Y, R);   % -> [B×R]
            [Tbr, ~] = local_toBR(T, R);   % -> [B×R]

            % 列切片（B×R）
            zA = Ybr(:, 1:n);          % 预激活 α
            zG = Ybr(:, n+1);          % 预激活 g
            aS = Tbr(:, 1:n);          % α*
            gS = Tbr(:, n+1);          % g*
            w  = Tbr(:, n+2:end);      % >=0

            % softmax(行) & sigmoid
            zA   = zA - max(zA, [], 2);
            expA = exp(zA);
            denom= sum(expA, 2) + cast(layer.Eps, 'like', Y);
            aHat = expA ./ denom;

            clip = cast(60, 'like', Y);
            zG   = max(min(zG, clip), -clip);
            gHat = 1 ./ (1 + exp(-zG));

            % 损失
            mse   = mean( sum((aHat - aS).^2, 2) + (gHat - gS).^2 );
            w     = max(w, 0);
            prog  = sum(aHat .* w, 2) + cast(layer.Eps, 'like', Y);
            lprog = mean( -log(prog) );

            loss = layer.LambdaMSE*mse + layer.LambdaProg*lprog;
        end

        function dLdY = backwardLoss(layer, Y, T)
            n = layer.N; R = 2*n + 1;
            [Ybr, fmtY] = local_toBR(Y, R);   % Y 原朝向->fmtY；计算用 BR
            [Tbr, ~   ] = local_toBR(T, R);

            B  = size(Ybr,1);
            zA = Ybr(:, 1:n);
            zG = Ybr(:, n+1);
            aS = Tbr(:, 1:n);
            gS = Tbr(:, n+1);
            w  = Tbr(:, n+2:end);

            % hats
            zA   = zA - max(zA, [], 2);
            expA = exp(zA);
            denom= sum(expA, 2) + cast(layer.Eps, 'like', Y);
            aHat = expA ./ denom;

            clip = cast(60, 'like', Y);
            zG   = max(min(zG, clip), -clip);
            gHat = 1 ./ (1 + exp(-zG));

            % dL/daHat
            w   = max(w, 0);
            dLa = (2*layer.LambdaMSE/B) * (aHat - aS) ...
                + (layer.LambdaProg/B) * ( - w ./ ( sum(aHat.*w, 2) + cast(layer.Eps, 'like', Y) ) );

            % softmax 反传（逐样本）：a ∘ (v - (a∘v)_sum)
            s    = sum(aHat .* dLa, 2);           % B×1
            dZ_A = aHat .* (dLa - s);             % B×n

            % dL/dzG via sigmoid
            dLg  = (2*layer.LambdaMSE/B) * (gHat - gS);
            dZ_G = dLg .* gHat .* (1 - gHat);     % B×1

            % 组装为 [B×R]；w 的 n 列不回传梯度（置 0）
            dBR            = zeros(size(Ybr), 'like', Y);
            dBR(:,1:n)     = dZ_A;
            dBR(:,n+1)     = dZ_G;
            % dBR(:,n+2:end) = 0;

            % 按 Y 的原始朝向还原
            dLdY = local_fromBR(dBR, fmtY, size(Y));
        end
    end
end

% ===== 工具：统一/还原张量朝向（仅转置/reshape，不改元素数） =====
function [BR, fmt] = local_toBR(X, R)
    sz = size(X);
    if ismatrix(X)
        if sz(2) == R
            BR  = X;    fmt = struct('mode','BR');
        elseif sz(1) == R
            BR  = X.';  fmt = struct('mode','RB');
        else
            error('fastHitLoss: unexpected 2D size [%d×%d], expect *×%d or %d×*', sz(1), sz(2), R, R);
        end
    elseif numel(sz)==4 && sz(1)==R && sz(2)==1 && sz(3)==1
        BR  = reshape(X, [R, sz(4)]).';  fmt = struct('mode','RB4','sz',sz);
    else
        error('fastHitLoss: unsupported dims.');
    end
end

function Y = local_fromBR(dBR, fmt, szY)
    switch fmt.mode
        case 'BR',  Y = dBR;
        case 'RB',  Y = dBR.';                         % -> [R×B]
        case 'RB4', Y = reshape(dBR.', [fmt.sz(1) fmt.sz(2) fmt.sz(3) fmt.sz(4)]);
        otherwise,  Y = reshape(dBR, szY);             % 兜底
    end
end
