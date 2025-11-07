function rpt = spinn3d_power_check(log)
okmask = (log.Pmax_eff(:) <= 0) | (log.P(:) <= log.Pmax_eff(:) + 1.0);
rpt.ok_ratio    = mean(okmask);
rpt.violate_n   = nnz(~okmask);
rpt.violate_pct = 100*(1 - rpt.ok_ratio);
if any(~okmask)
    d = log.P(~okmask) - log.Pmax_eff(~okmask);
    rpt.max_overshoot  = max(d);
    rpt.mean_overshoot = mean(d);
else
    rpt.max_overshoot  = 0;
    rpt.mean_overshoot = 0;
end
fprintf('[POWER CHECK] ok %.2f%%, violate %d (%.2f%%), worst +%.1f W, mean +%.1f W\n', ...
        100*rpt.ok_ratio, rpt.violate_n, rpt.violate_pct, rpt.max_overshoot, rpt.mean_overshoot);
end
