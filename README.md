# SPINN3D (physics-checked PID baseline)

This package contains a 4-DoF manipulator (base yaw + 3 pitch) built with
Robotics System Toolbox and a physics-consistent simulation loop.

**Key points**
- Dynamics via `massMatrix`, `velocityProduct`, `gravityTorque`.
- Power is computed **before integration** using `dq(t)` and the **capped** torque, so
  `P(t) = sum_i |tau_i(t) * dq_i(t)|` matches caps.
- Caps order: **torque hard cap** → **per-axis rated power** → **total power cap**.
- Joint order: `[J1 yaw(Z), J2 shoulder(Y), J3 elbow(Y), J4 wrist(Y)]`.
- No animation; a static figure shows EE trajectory + final pose + power panels.

Run:
```matlab
spinn3d_demo_pid
```

Optional:
- Add time-varying references by setting `sim.refFcn = @(t) deal(qr(t), dqr(t));`
- Enable Coulomb friction with `sim.friction = struct('Fc',[...], 'v',[...]);`
