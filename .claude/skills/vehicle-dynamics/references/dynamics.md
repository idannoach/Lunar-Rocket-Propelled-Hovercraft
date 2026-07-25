# `src/` — 6-DOF physics plant detail

All functions use `GP` (global parameters, `config/GP.m`: `g0=9.80665`, `g_lunar=1.625`)
and `HP` (hovercraft parameters, from `hovercraft.json` + the two allocation/inertia
precompute steps in `main.m`) as the standard parameter-struct names.

## `calculate_dynamics(~, x, cmd_throttles, GP, HP)` — `src/calculate_dynamics.m`

Top-level ODE RHS, used by the **continuous** path via `ode15s` (called from
`gnc/closed_loop_system.m`). Splits `x` into rigid-body (`x(1:13)`) and engine-lag
(`x(14:19)` = `actual_throttles`) state, calls `calculate_rigid_body_dynamics` (using the
**actual**, lagged throttles — not `cmd_throttles`) and `calculate_engine_dynamics`
(which drives `actual_throttles` toward `cmd_throttles`), concatenates `[dxdt_rb; throttles_dot]`.

## `calculate_rigid_body_dynamics(x_rb, actual_throttles, GP, HP)` — `src/calculate_rigid_body_dynamics.m`

The 13-state derivative. Order of operations: `calculate_thrust_forces` (throttles →
body forces/moments/mdot) → `calculate_body_to_nwu_matrix` (DCM from current Euler
angles) → `pos_dot = R_b2n * vel` → `calculate_euler_kinematics` →
`calculate_translational_dynamics` → `calculate_rotational_dynamics` →
`mass_dot = -mdot_total`. Returns `[pos_dot; vel_dot; euler_dot; rates_dot; mass_dot]`.
Called directly (not via `calculate_dynamics`) by `run_discrete_simulation.m`'s RK4
sub-stepping, which evaluates the engine lag analytically instead of integrating it.

## `calculate_translational_dynamics(Forces_body, mass, R_b2n, GP, r, p, q, u, v, w)` — `src/calculate_translational_dynamics.m`

`a_body = F_body/mass + g_body - omega×v` (Newton's 2nd law in a rotating Body frame).
Gravity is defined in NWU as `[0;0;-g_lunar]` then rotated into Body via
`R_n2b = R_b2n'` (transpose = inverse for an orthogonal DCM). Coriolis term:
`cross_omega_v = [q*w-r*v; r*u-p*w; p*v-q*u]`.

## `calculate_rotational_dynamics(Moments_body, mass, HP, r, p, q)` — `src/calculate_rotational_dynamics.m`

Euler's rigid-body equations: `alpha = I \ (M - omega×(I*omega))`, where `I` is built
diagonal from `calculate_moments_of_inertia(mass, HP)` (**recomputed every call** from
the current instantaneous mass — inertia changes as fuel burns). Gyroscopic/coupling
term `cross_omega_H = [q*(Izz*r)-r*(Iyy*q); r*(Ixx*p)-p*(Izz*r); p*(Iyy*q)-q*(Ixx*p)]`.
Note the input order is `(r, p, q)` (yaw, roll, pitch), not `(p, q, r)` — matches the
caller in `calculate_rigid_body_dynamics.m`.

## `calculate_euler_kinematics(rates, phi, theta)` — `src/calculate_euler_kinematics.m`

Body rates `[p;q;r]` → Euler-angle rates via the standard Z-Y-X kinematic transformation
matrix (involves `tan(theta)` and `1/cos(theta)`). **Errors** (`calculate_euler_kinematics:Singularity`)
if `|cos(theta)| < 1e-6` — gimbal lock at pitch ±90°. The attitude controller
(`gnc/run_attitude_controller.m`) clamps commanded pitch to ±30° so this shouldn't fire
in normal operation; it's a guard against unexpected initial conditions/large disturbances.
Fix direction if it ever needs to be removed: switch to a quaternion attitude representation.

## `calculate_thrust_forces(throttles, HP, GP)` — `src/calculate_thrust_forces.m`

Loops over `HP.nEngines`, for each: computes engine azimuth
`theta_rad = deg2rad((i-1)*HP.degEngineRadialAngle)` and 3D position
`[R*cos(theta); R*sin(theta); mEngineAxialPos]`, moment arm `r_arm = pos_engine - [0;0;mCG(3)]`,
converts throttle to mass flow (`throttles(i) * gpsecMaxPropFlowRate / 1000`), gets the
thrust vector via `calculate_thrust_vec`, accumulates `Forces_body += T_vec`,
`Moments_body += cross(r_arm, T_vec)`, `mdot_total += mdot_i`.

**Engine indexing assumption**: this function derives each engine's azimuth from
`(i-1)*degEngineRadialAngle` (i.e. assumes engines are evenly spaced starting at 0°) —
this is a **different, simpler indexing scheme** than `gnc/calculate_allocation_matrices.m`,
which hardcodes each engine's angle explicitly (0°, 60°, 120°, 180°, 240°, 300°) and a
different physical engine-number mapping (1=Front, 2=FL, 3=BL, 4=Back, 5=BR, 6=FR). If
you change engine count/spacing, both places need updating consistently.

## `calculate_thrust_vec(newtonMaxThrust, secIsp, kgpsecMassFlowRate, g, radEngineTheta, radEngineTiltAngle)` — `src/calculate_thrust_vec.m`

Single-engine thrust vector: direction from `(radEngineTheta, radEngineTiltAngle)` (cant
angle off Z-axis, azimuth around Z), magnitude from the rocket equation
`Thrust = Isp * mdot * g0`. Saturates to `newtonMaxThrust` by scaling the vector (preserves
direction) if `norm(thrust_vec) >= newtonMaxThrust`.

## `calculate_engine_dynamics(cmd_throttles, actual_throttles, HP)` — `src/calculate_engine_dynamics.m`

First-order lag: `tau = (HP.msecResponseTo90PctThrustTime/1000) / 2.3` (2.3 = time
constants to reach 90% of a first-order step response), `throttles_dot = (cmd - actual)/tau`.

## `calculate_moments_of_inertia(kgMass, HP)` — `src/calculate_moments_of_inertia.m`

Evaluates `Ixx/Iyy/Izz = poly(1)*kgMass + poly(2)` using the linear polynomials
precomputed by `calculate_moments_of_inertia_poly`. **Assumes body axes = principal
axes** (symmetric engine/tank placement) → strictly diagonal inertia tensor, products of
inertia are assumed exactly zero.

## `calculate_moments_of_inertia_poly(hovercraft_parameters)` — `src/calculate_moments_of_inertia_poly.m`

Called **once** in `main.m` (offline precompute, not per-tick). Fits a line
`I(mass) = slope*mass + intercept` between the empty-mass and full-mass inertia bounds
(`HP.kgsqmIxx.min/.max` etc.) for each axis, given `delta_m = kgFuelMass`. **Errors**
(`ZeroFuelMass`) if `kgFuelMass == 0` (would divide by zero, NaN-poisoning every inertia
value downstream) — fix by setting `kgFuelMass > 0` or defining constant inertia
directly instead of calling this function.
