# Mapping this project's dynamics onto a FALCON.m model

## What the project already has

`Project/` is a 6-DOF closed-loop simulation of a lunar hovercraft powered-descent
lander, driven by an analytic guidance law — not currently by FALCON.m or any direct
collocation solver. Ground truth, read directly from the code:

- **Environment**: `config/GP.m` → `g_lunar = 1.625 m/s^2` (Moon gravity), `g0 = 9.80665`
  used only for Isp→thrust conversion.
- **Vehicle**: `config/hovercraft.json` → `kgTotalMass = 60`, `kgFuelMass = 30` (so dry
  mass = 30 kg), `nEngines = 6` arranged radially, `newtonMaxThrust = 20` N per engine,
  `secIsp = 285`, throttle range `[40, 100] %`, plus inertia bounds and engine lag time
  constants (`msecResponseTo90PctThrustTime`).
- **Mission**: `config/mission.json` → start at the origin at rest, target
  `mTargetPosition = [25000, 0, -4000]` m (NWU frame: X north, Y west, Z up — so
  -4000 means 4000 m below the start altitude), nominal `sFinalSimulationTime = 400` s.
- **Full state** (`utils/pack_initial_state_vector.m`, `src/calculate_rigid_body_dynamics.m`):
  13 rigid-body states `[pos(3) NWU; vel(3) body; euler(3); rates(3); mass]` plus 6
  engine-lag states appended for the discrete-time simulation (19 total). Translational
  dynamics (`src/calculate_translational_dynamics.m`) account for gravity rotated into
  the body frame and Coriolis terms from body rates; rotational dynamics
  (`src/calculate_rotational_dynamics.m`) solve Euler's equations with mass-dependent
  inertia.
- **Guidance**: `gnc/calculate_guidance_law.m` is a closed-form ZEM/ZEV-style LQ law —
  commanded inertial acceleration `= (6/t_go²)·Δr − (4/t_go)·v`, gravity-compensated,
  recomputed every control cycle from current state and time-to-go. This is what a
  FALCON.m-generated trajectory would be benchmarked against.

## Recommended scope for a first FALCON.m model: translational point-mass, not full 6-DOF

Match what the cited literature in `Essays/` actually optimizes (G-FOLD-style convex
guidance, ZEM/ZEV, LQ powered descent) — these are point-mass powered-descent
formulations with thrust vector + magnitude as the control, not full attitude dynamics.
Building the full 6-DOF + engine-lag model as a FALCON.m dynamics function is possible
but adds rotational dynamics, control allocation, and PWM/MIB nonlinearities that the
cited comparison methods don't model either — so a point-mass model is both the fair
comparison and the tractable one to get solving first. Attitude/allocation can be added
later as a second, harder phase if the thesis wants it.

**Suggested states** (7): `[X, Y, Z, Vx, Vy, Vz, m]` in the NWU inertial frame directly
(skip the body-frame velocity + Euler angle indirection — FALCON.m doesn't care which
frame you use, and NWU-frame velocity states make the dynamics and constraints simpler
to write and to compare against `x_out(:,1:3)` from `run_discrete_simulation.m`).

**Suggested controls** (4): commanded thrust vector components `[Tx, Ty, Tz]` in NWU
directly, i.e. optimize the *net* thrust vector rather than 6 individual throttles —
this is the standard simplification in the powered-descent-guidance literature and
sidesteps needing the engine allocation matrix (`gnc/calculate_allocation_matrices.m`)
inside the optimizer. Bound `|T| ≤ n_engines_active_effective × newtonMaxThrust` via a
path constraint (see below) rather than per-engine bounds.

**Dynamics function** (`states_dot = f(states, controls)`):

```matlab
function [states_dot] = source_powered_descent(states, controls)
X = states(1); Y = states(2); Z = states(3);
Vx = states(4); Vy = states(5); Vz = states(6);
m = states(7);

Tx = controls(1); Ty = controls(2); Tz = controls(3);

g_lunar = 1.625;      % GP().g_lunar
Isp     = 285;        % HP.secIsp
g0      = 9.80665;    % HP.g0 (for Isp -> mdot conversion)

T_mag = sqrt(Tx^2 + Ty^2 + Tz^2 + 1e-9);   % small epsilon avoids a non-smooth sqrt(0)
mdot  = -T_mag / (Isp * g0);

Xdot  = Vx;  Ydot  = Vy;  Zdot  = Vz;
Vxdot = Tx / m;
Vydot = Ty / m;
Vzdot = Tz / m - g_lunar;   % Z is up in NWU, gravity acts -Z

states_dot = [Xdot; Ydot; Zdot; Vxdot; Vydot; Vzdot; mdot];
end
```

**Boundaries** — from `config/mission.json`:

```matlab
phase.setInitialBoundaries([0; 0; 0; 0; 0; 0; hovercraft_parameters.kgTotalMass]);
phase.setFinalBoundaries([25000; 0; -4000; 0; 0; 0; 30], ...   % lower: touchdown, m >= dry mass
                          [25000; 0; -4000; 0; 0; 0; 60]);      % upper: allow any remaining fuel
```

(Set the final mass bound as an inequality `[dry_mass, initial_mass]` rather than an
equality, since minimizing fuel means letting the solver find the true optimum instead
of prescribing it.)

**Path constraint — thrust magnitude bound** (throttle range × active engines): build
`T_mag` as a `falcon.Output` from the model (`y = T_mag`) or via
`falcon.PathConstraintBuilder` on `[Tx,Ty,Tz]`, then constrain
`0.4 × n_eff × newtonMaxThrust ≤ T_mag ≤ n_eff × newtonMaxThrust` (with `n_eff` a
judgment call — e.g. 4-6 depending how many engines can contribute to a given thrust
direction given the radial engine layout).

**Cost** — fuel-optimal: `problem.addNewLinearPointCost(-m_state_at_tf)` (maximize
final mass ⇔ minimize propellant used). Time-optimal instead: make `tf` a
`falcon.Parameter` and `problem.addNewLinearPointCost(tf)`.

**Final time**: either fix `tf` to `mission_parameters.sFinalSimulationTime` for a
direct apples-to-apples comparison with the existing simulation's `t_f`, or free it as a
`falcon.Parameter` if the point is to find the time-optimal descent duration.

## Suggested project layout

Keep this separate from the existing closed-loop sim rather than editing it in place —
they answer different questions (closed-loop tracking performance vs. open-loop
optimal benchmark):

```
Project/
  opt_control/
    main_falcon_descent.m       % builds + solves the FALCON.m problem, reads the same
                                 % config/*.json via read_from_json for parameter parity
    source_powered_descent.m    % the states_dot model above
    source_thrust_path.m        % thrust-magnitude path constraint, if built by hand
    compare_to_lq_guidance.m    % overlays FALCON.m's optimal trajectory against
                                 % run_discrete_simulation's closed-loop trajectory
```

Reuse `utils/read_from_json.m` to load `hovercraft.json`/`mission.json` so the FALCON.m
problem and the closed-loop simulation always run against the same numbers — avoids the
comparison being contaminated by parameter drift between the two.

## Validating the result

After `solver.Solve()`, call `problem.Simulate()` and compare its re-simulated states
against `phase.StateGrid.Values` — a large mismatch usually means the discretization
grid (`tau`) is too coarse for how fast the dynamics move near touchdown, not a solver
failure. Increasing sample count near the end of `tau` (a non-uniform grid, e.g. denser
near tau=1) is the standard fix for powered-descent problems where the interesting
dynamics happen in the final seconds.
