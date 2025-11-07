function spinn3d_train_fast_hit(dataset_mat, model_out)
% 训练“首达优先(Fast‑Hit)”分配器；兼容 X/Y、X/T、主表 table 三种数据格式。
% Y 期望列：[alpha*(n) , g*(1) , w(n)]  —— 共 2n+1 列；无 w 时自动补零并禁用进度项。

    % -------- 1) 数据文件定位 --------
    if nargin < 1 || isempty(dataset_mat)
        cand = { ...
            'data_fast/spinn3d_fast_dataset_master.mat', ...
            'spinn3d_fast_dataset.mat', ...
            'data/spinn3d_v2_dataset_master.mat' ...
        };
        dataset_mat = '';
        for i=1:numel(cand)
            if exist(cand{i},'file')==2
                dataset_mat = cand{i}; break;
            end
        end
        if isempty(dataset_mat)
            error(['未找到数据集。期望任一存在：\n', ...
                   '  - data_fast/spinn3d_fast_dataset_master.mat\n', ...
                   '  - spinn3d_fast_dataset.mat\n', ...
                   '  - data/spinn3d_v2_dataset_master.mat']);
        end
    end
    if nargin < 2 || isempty(model_out)
        model_out = 'data_fast/spinn3d_v2_net_fast.mat';
    end

    % -------- 2) 读取并抽取 X/Y --------
    S = load(dataset_mat);
    [X, Y, n, hasW] = local_get_XY(S);   % 兼容 X/Y、X/T、主表 table
    X = double(X);  Y = double(Y);
    [N, D] = size(X);  R = size(Y,2);
    assert(N==size(Y,1), 'X/Y 行数不一致。');

    % 统计信息
    fprintf('[DATA] %s | X: %d×%d  Y: %d×%d  (n=%d, hasW=%d)\n', ...
        dataset_mat, N, D, size(Y,1), size(Y,2), n, hasW);

    % -------- 3) 划分训练/验证（二维数值输入） --------
    idx = randperm(N);
    nTr = max(1, round(0.85*N));
    tr = idx(1:nTr);  va = idx(nTr+1:end);
    Xtr = X(tr,:);  Ttr = Y(tr,:);
    Xva = X(va,:);  Tva = Y(va,:);

    % -------- 4) 网络结构（轻量 MLP） --------
    % 头部输出与标签列数完全对齐：n+1+n
    layers = [
        featureInputLayer(D, "Normalization","none", "Name","in")
        fullyConnectedLayer(128, "Name","fc1")
        reluLayer("Name","r1")
        fullyConnectedLayer(128, "Name","fc2")
        reluLayer("Name","r2")
        fullyConnectedLayer(2*n+1, "Name","head")   % 与标签列数一致
        fastHitLossLayer(n, 'LambdaMSE',1.0, 'LambdaProg', hasW*0.7, 'Eps',1e-6)
    ];

    opts = trainingOptions('adam', ...
        'InitialLearnRate',1e-3, ...
        'MiniBatchSize',256, ...
        'Shuffle','every-epoch', ...
        'MaxEpochs',35, ...
        'ValidationData',{Xva, Tva}, ...
        'ValidationFrequency', max(1, floor(numel(va)/256)), ...
        'Verbose',true, ...
        'Plots','none');

    % -------- 5) 训练 --------
    net = trainNetwork(Xtr, Ttr, layers, opts);  % 注意：二维输入

    % -------- 6) 保存模型（与 demo/控制器兼容） --------
    model = struct(); model.net = net;
    local_ensure_dir(fileparts(model_out));
    save(model_out, '-struct', 'model');
    fprintf('Saved model: %s\n', model_out);
end

% ================= helpers =================

function [X, Y, n, hasW] = local_get_XY(S)
% 兼容三种数据来源：
%   1) 直接矩阵：X、Y
%   2) 旧文件：  X、T (把 T 当 Y)
%   3) 主表table：SPINN3D_FAST_DATASET / SPINN3D_DATASET
    if isfield(S,'X') && isfield(S,'Y')
        X = S.X;  Y = S.Y;
    elseif isfield(S,'X') && isfield(S,'T')
        X = S.X;  Y = S.T;   % 兼容旧保存名
    else
        T = [];
        if isfield(S,'SPINN3D_FAST_DATASET'), T = S.SPINN3D_FAST_DATASET; end
        if isempty(T) && isfield(S,'SPINN3D_DATASET'), T = S.SPINN3D_DATASET; end
        assert(~isempty(T), '数据文件缺少 X/Y 或 X/T 或主表 table(SPINN3D_*_DATASET)。');
        vnames = T.Properties.VariableNames;
        xmask  = startsWith(vnames,'x');
        X = table2array(T(:, xmask));
        Y = table2array(T(:, ~xmask));
    end
    % 识别标签列结构
    R = size(Y,2);
    if mod(R-1,2)==0 && (R-1)>=2
        n = (R-1)/2; hasW = true;   % [alpha(n), g(1), w(n)]
    else
        % 旧版可能只有 [alpha(n), g]；补 w=zeros 并禁用进度项
        n = R-1; hasW = false;
        Y = [Y, zeros(size(Y,1), n)];
    end
end

function local_ensure_dir(d)
    if ~isempty(d) && exist(d,'dir')~=7, mkdir(d); end
end
