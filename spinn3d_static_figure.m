function stats = spinn3d_static_figure(kin, log, opts)
% 静态三联图：EE 轨迹 + 分关节功率 + 总功率
if nargin<3, opts=struct(); end
if ~isfield(opts,'goal_radius'),    opts.goal_radius    = 0.03;  end
if ~isfield(opts,'trim_t0'),        opts.trim_t0        = 0.02;  end
if ~isfield(opts,'robust_pct'),     opts.robust_pct     = 0.995; end
if ~isfield(opts,'show_raw'),       opts.show_raw       = true;  end
if ~isfield(opts,'trim_after_hit'), opts.trim_after_hit = true;  end

t_full = log.t(:); Nfull = numel(t_full);
if opts.trim_after_hit && isfield(log,'idx_hit') && ~isempty(log.idx_hit) && log.idx_hit>=1
    idx_end = log.idx_hit;
else
    idx_end = Nfull;
end
sl = 1:idx_end; t = t_full(sl);

% --- per-joint power ---
if isfield(log,'P3') && ~isempty(log.P3)
    P3 = log.P3(sl,:);
elseif isfield(log,'dq') && isfield(log,'tau') && ~isempty(log.dq) && ~isempty(log.tau)
    P3 = abs(log.dq(sl,:) .* log.tau(sl,:));
else
    error('log must contain P3 or (dq & tau).');
end

% --- total power ---
if isfield(log,'P') && ~isempty(log.P)
    P = log.P(sl);
else
    P = sum(P3,2);
end
has_raw = isfield(log,'P_raw') && ~isempty(log.P_raw); if has_raw, Praw = log.P_raw(sl); end
has_eff = isfield(log,'Pmax_eff') && ~isempty(log.Pmax_eff); if has_eff, Pmax_eff = log.Pmax_eff(sl); else, Pmax_eff = []; end

% --- EE 轨迹与最终骨架 ---
N = idx_end; EE=zeros(N,3); Pend=zeros(5,3); Tend=eye(4);
for k=1:N
    [Pk,Tk] = spinn3d_fkine(kin, log.q(k,:));
    EE(k,:) = Pk(end,:);
    if k==N, Pend = Pk; Tend = Tk; end
end

% 统计
rd = @(x) x*180/pi;
stats = struct();
stats.q_final_deg = rd(log.q(idx_end,:));
stats.ee_final    = EE(end,:);
stats.ee_start    = EE(1,:);
stats.ee_path_length = sum(sqrt(sum(diff(EE).^2,2)));
stats.ee_bbox_min = min(EE,[],1);
stats.ee_bbox_max = max(EE,[],1);

% 画图
figure('Name','SPINN3D Static','Color','w','Position',[100 100 1200 600]);
tl = tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% 左：EE + 目标球
ax1 = nexttile(tl,[2 1]); hold(ax1,'on'); grid(ax1,'on'); axis(ax1,'equal'); view(ax1,135,25);
plot3(ax1, EE(:,1), EE(:,2), EE(:,3), '-', 'LineWidth', 1.6);
plot3(ax1, EE(1,1),EE(1,2),EE(1,3), 'go', 'MarkerSize', 7, 'LineWidth',1.5);
plot3(ax1, EE(end,1),EE(end,2),EE(end,3), 'ro','MarkerSize', 7, 'LineWidth',1.5);
plot3(ax1, Pend(:,1), Pend(:,2), Pend(:,3), '-o', 'LineWidth', 2, 'MarkerSize', 5);
if isfield(opts,'q_goal') && ~isempty(opts.q_goal)
    [~,Tgoal] = spinn3d_fkine(kin, opts.q_goal);
    g=Tgoal(1:3,4).'; [sx,sy,sz]=sphere(18); r=opts.goal_radius;
    surf(ax1, r*sx+g(1), r*sy+g(2), r*sz+g(3), 'EdgeColor','none','FaceAlpha',0.2);
    legend(ax1, {'EE traj','start','final','final skeleton','goal'}, 'Location','best');
else
    legend(ax1, {'EE traj','start','final','final skeleton'}, 'Location','best');
end
xlabel(ax1,'X [m]'); ylabel(ax1,'Y [m]'); zlabel(ax1,'Z [m]'); title(ax1,'EE Trajectory + Final Pose');

% 右上：分关节功率
ax2 = nexttile(tl); hold(ax2,'on'); grid(ax2,'on');
plot(ax2, t, P3, 'LineWidth', 0.9);
xlabel(ax2,'t [s]'); ylabel(ax2,'Joint Power P_i [W]'); title(ax2,'Per-Joint Power');
legend(ax2, arrayfun(@(i)sprintf('J%d',i), 1:size(P3,2), 'UniformOutput', false), 'Location','best');
idxTrim = (t >= max(0, opts.trim_t0));
y2 = local_robust_max(P3(idxTrim,:), opts.robust_pct); y2 = max(y2, 1);
ylim(ax2, [0, y2*1.05]); xlim(ax2, [t(1) t(end)]);

% 右下：总功率
ax3 = nexttile(tl); hold(ax3,'on'); grid(ax3,'on');
h=[]; names={};
h(end+1)=plot(ax3,t,P,'LineWidth',1.0); names{end+1}='P(t)';
if isfield(opts,'Pmax') && ~isempty(opts.Pmax)
    h(end+1)=yline(ax3,opts.Pmax,'r--','LineWidth',1.0); names{end+1}='P_{max}';
end
if ~isempty(Pmax_eff) && any(Pmax_eff>0)
    h(end+1)=plot(ax3,t,Pmax_eff,'--','LineWidth',1.0); names{end+1}='P_{max,eff}(t)';
end
if opts.show_raw && has_raw
    h(end+1)=plot(ax3,t,Praw,':','LineWidth',0.9); names{end+1}='P_{raw}(t)';
end
legend(ax3,h,names,'Location','best');
y3 = local_robust_max(P(idxTrim,:), opts.robust_pct);
if isfield(opts,'Pmax') && ~isempty(opts.Pmax), y3=max(y3, opts.Pmax); end
if ~isempty(Pmax_eff), y3=max([y3; Pmax_eff(idxTrim)]); end
if opts.show_raw && has_raw, y3=max(y3, local_robust_max(Praw(idxTrim), opts.robust_pct)); end
y3 = max(y3, 1);
ylim(ax3,[0, y3*1.05]); xlim(ax3,[t(1) t(end)]);
xlabel(ax3,'t [s]'); ylabel(ax3,'Power [W]'); title(ax3,'Total Power');
end

function y = local_robust_max(X,pct)
x = X(:); x = x(isfinite(x)); if isempty(x), y = 0; return; end
x = sort(abs(x)); k = max(1, min(numel(x), round(pct*numel(x))));
y = x(k);
end
