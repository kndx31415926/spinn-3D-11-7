function controller = spinn3d_controller_pid(Kp, Ki, Kd, opts)
if nargin < 4, opts = struct(); end
if ~isfield(opts,'useGravityComp'), opts.useGravityComp = true; end
if ~isfield(opts,'antiWindup'),     opts.antiWindup     = true; end
if ~isfield(opts,'Imax'),           opts.Imax           = inf;  end

Kp = Kp(:).'; Ki = Ki(:).'; Kd = Kd(:).';
controller.state.ei = zeros(size(Kp));
controller.state.last_t = []; controller.state.last_err = [];

    function [tau, st] = step(t, q, dq, q_ref, dq_ref, robot)
        e  = q_ref - q; ed = dq_ref - dq;
        if isempty(controller.state.last_t), dt = 0; else, dt = max(0, t - controller.state.last_t); end
        controller.state.ei = controller.state.ei + e * dt;
        if opts.antiWindup, controller.state.ei = max(-opts.Imax, min(opts.Imax, controller.state.ei)); end
        tau = Kp.*e + Kd.*ed + Ki.*controller.state.ei;
        if opts.useGravityComp
            try, tau = tau + gravityTorque(robot, q); catch, end
        end
        if isfield(opts,'uMax') && ~isempty(opts.uMax)
            tau = sign(tau).*min(abs(tau), opts.uMax(:).');
        end
        controller.state.last_t = t; controller.state.last_err = e; st = controller.state;
    end
controller.step = @step;
end
