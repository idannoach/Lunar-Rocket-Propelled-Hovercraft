# Attitude control, allocation, and actuation

Inner loop: converts the guidance layer's NWU thrust-acceleration command into 6 physical
engine throttle commands, plus the hardware-realistic PWM/minimum-impulse-bit shaping and
the ODE wiring function.

## `run_attitude_controller(x_curr, cmd_x, cmd_z, hovercraft_parameters)` — `gnc/run_attitude_controller.m`

- **Inputs**: `x_curr` (full state vector; reads `mass=x_curr(13)`, `theta=x_curr(8)`,
  `q=x_curr(11)`), `cmd_x`/`cmd_z` (NWU acceleration command X/Z components from
  `calculate_guidance_law` — **Y is not accepted**, matching the documented Y-axis
  limitation), `hovercraft_parameters`.
- **Output**: `[desired_Fz, desired_My]` — body-frame vertical force and pitch moment.
- **1. Pitch command**: `desired_theta = atan2(cmd_x, cmd_z)`, clamped to
  ±`hovercraft_parameters.degMaxPitchAngle` (`max_pitch`) BEFORE computing force magnitude,
  so the two stay consistent — prevents flip-over and total loss of vertical lift
  authority. Keep this well clear of ±90°: that's a hard gimbal-lock singularity in
  `calculate_euler_kinematics.m` (`tan(theta)`/`1/cos(theta)` blow up there), not just a
  tunable safety margin.
- **2. Thrust magnitude (altitude-priority)**: `desired_Fz = mass * cmd_z / cos(desired_theta)`
  sizes body-Z force so its Z-projection after pitching exactly delivers `cmd_z`. When
  pitch is saturated at the configured limit, this accepts reduced X authority rather than
  over-thrusting in Z (altitude priority). When pitch is unsaturated this reduces to
  `+mass*norm([cmd_x, cmd_z])` (NOT the negated form used under the old NED-frame
  convention). Positive `desired_Fz` = upward body force.
- **3. Dynamic-inversion PD**: tuned via natural frequency/damping (`omega_n=3.0 rad/s`,
  `zeta=0.85`) rather than raw Nm gains: `Kp=omega_n^2`, `Kd=2*zeta*omega_n`. Theta error
  is wrapped to `[-pi,pi]` via `atan2(sin(e),cos(e))` for stability. Angular-acceleration
  command `alpha_y_cmd = Kp*theta_error + Kd*q_error` is scaled by instantaneous `Iyy`
  (from `calculate_moments_of_inertia`) to get `desired_My = Iyy * alpha_y_cmd`.

## `allocate_controls(desired_Fz, desired_My, B_pinv_sym)` — `gnc/allocate_controls.m`

Maps `[Fz; My]` to 4 symmetric channels via `sym_throttles_ideal = B_pinv_sym * nu`, then
unpacks to 6 physical engines:

```
Engine:  1=Front  2=FL  3=BL  4=Back  5=BR  6=FR
Channel: throttles(1)=ch1  throttles(2)=throttles(6)=ch2  throttles(3)=throttles(5)=ch3  throttles(4)=ch4
```

Clamps every throttle to `[0,1]` — prevents negative throttle (which would invert thrust
direction and, per the engine dynamics model, spuriously increase mass) and over-unity
commands that would bypass the physical saturation logic elsewhere.

## `calculate_allocation_matrices(hovercraft_parameters)` — `gnc/calculate_allocation_matrices.m`

Called **once** in `main.m` (not per-tick) to precompute `hovercraft_parameters.B_pinv`.

- Builds `Cz = newtonMaxThrust*cos(radTiltAngle)` (per-engine max Z-force) and
  `Cm = -newtonMaxThrust*C_arm` (per-engine max moment arm, where
  `C_arm = R*cos(radTiltAngle) - delta_z*sin(radTiltAngle)`, `delta_z` = engine axial
  position minus CG Z).
- Assembles the 2×4 matrix `B_sym` mapping the 4 symmetric channels
  (1=Front only, 2=Front-Left+Front-Right combined, 3=Back-Left+Back-Right combined,
  4=Back only) to `[Fz; My]`, with each channel's moment computed from its engines'
  angular positions (0°, ±60°, ±120°, 180°) via `cos(angle)`.
- `hovercraft_parameters.B_pinv = pinv(B_sym)` — rank-2 system, pseudo-inverse is stable.

## `calculate_throttles_command(dt, ideal_throttles, engine_on_timers, hovercraft_parameters)` — `gnc/calculate_throttles_command.m`

Hardware-realistic PWM + minimum-impulse-bit (MIB) throttle-shaping state machine, run
every discrete tick (**only used by `run_discrete_simulation.m`**, not the continuous
path). Per-engine (loop over 6), four regimes:

1. **MIB pulse active** (`engine_on_timers(i) > 0`): engine must stay on for
   `secMinDt = newtonsecMinImpulseBit / newtonMaxThrust`; throttle clamped to
   `[minThrottle, maxThrottle]` (the 40%-ish hardware floor from `pctThrottleRange`).
   Timer decrements by `dt`; clamped to 0 when the pulse finishes.
2. **Continuous throttling** (`ideal_throttles(i) >= minThrottle`): pass through, clamped
   to `maxThrottle`.
3. **PWM pulse triggered** (`pwmDeadband < ideal_throttles(i) < minThrottle`): fires at
   100% for `secMinDt` to satisfy the MIB (e.g. 5 Ns) within the deadband window, starts
   a new `engine_on_timers(i) = secMinDt - dt`.
4. **Engine off** (`ideal_throttles(i) <= pwmDeadband`): 0.

## `closed_loop_system(t, x, GP, hovercraft_parameters, mission_parameters)` — `gnc/closed_loop_system.m`

`ode15s`-compatible RHS for the **continuous** simulation path (`run_continuous_simulation.m`).
Pipeline: Navigation (passthrough — `current_state = x`) → Guidance
(`calculate_guidance_law`, Y discarded with `~`) → Attitude control
(`run_attitude_controller`) → Allocation (`allocate_controls`) → Physics
(`calculate_dynamics`, returns `dxdt`).

**Two documented architectural limitations** (read the header comment before "fixing" these):

1. **East-axis (Y) guidance is discarded** — `cmd_accel_y` from `calculate_guidance_law`
   is dropped with `~`. `run_attitude_controller` only controls pitch (X-Z plane), so any
   mission with non-zero East target position accumulates uncorrected Y error. Fixing
   requires a roll-control loop and an expanded allocation matrix.
2. **GNC runs at ODE sub-step rate, not ZOH 50 Hz** — `ode15s` calls this function at each
   internal evaluation (~5-10× per 20 ms window), so guidance and attitude control run at
   effectively infinite rate. The engine first-order lag integrates against a
   continuously-updated command instead of a zero-order-hold signal, making the
   continuous simulation unconservatively optimistic vs. the discrete
   (hardware-representative) path in `run_discrete_simulation.m`.

The discrete path (`run_discrete_simulation.m`, project root) reimplements the same 4 GNC
calls per 50 Hz tick manually (not via `closed_loop_system`) and additionally applies
`calculate_throttles_command` (PWM/MIB) and a 1-step transport-delay buffer — see that
file for the fuel-exhaustion guard and the analytic engine-lag RK4 sub-stepping.
