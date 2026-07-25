# `utils/` and `config/GP.m` — shared support functions

## `config/GP.m` — `GP()`

No-argument function returning the global-constants struct: `g0 = 9.80665` (standard
Earth gravity, used only for Isp→thrust conversion in `calculate_thrust_vec`),
`g_lunar = 1.625` (surface gravity used everywhere in guidance/dynamics). Called fresh
each time (`GPl = GP()` or similar) rather than passed as a persistent global.

## `calculate_body_to_nwu_matrix(phi, theta, psi)` — `utils/calculate_body_to_nwu_matrix.m`

3×3 DCM, Z-Y-X Euler sequence (yaw psi about Z, pitch theta about Y, roll phi about X):
`R_b2n = Rz(psi)*Ry(theta)*Rx(phi)`, so `V_nwu = R_b2n * V_body`. Used throughout
`src/` and by `gnc/calculate_guidance_law.m` to rotate body velocity into NWU.

## `flip_z_axis(v)` — `utils/flip_z_axis.m`

Covered in the `gnc-functions` skill (NWU ↔ paper-frame Z-down conversion, involutory,
flips Z sign only). Documented there since it's guidance-law-specific machinery.

## `calculate_miss_distance(x_out, target_pos)` — `utils/calculate_miss_distance.m`

`norm(x_out(end,1:3) - target_pos)` — Euclidean distance from the final logged position
to the target. Transposes `target_pos` to a row vector if given as a column. Used by
`main.m`'s t_f-convergence loop and by `log_results`.

## `pack_initial_state_vector(mission_parameters, hovercraft_parameters)` — `utils/pack_initial_state_vector.m`

Builds the 19×1 initial state `x0` from `mission_parameters` (`mInitialPosition`,
`mpsInitialVelocity`, `radEulerAngles`, `radpsecRates`) and
`hovercraft_parameters.kgTotalMass`, with `throttles_0 = zeros(6,1)`. See SKILL.md's
state-vector-layout table for index meanings.

## `read_from_json(file_name)` — `utils/read_from_json.m`

`jsondecode(fileread(which(file_name)))` — resolves `file_name` via the MATLAB path
(so `config/` must be on the path, set up by `startup.m`), errors
(`main:MissingFile`) if not found. Used to load `mission.json` and `hovercraft.json`.

## `touchdown_event(t, x, target_Z, arrival_direction)` — `utils/touchdown_event.m`

`ode45`/`ode15s`-compatible event function for `run_continuous_simulation.m`. Fires when
`x(3) - target_Z` crosses zero, `isterminal=1` (stops integration), `direction=
arrival_direction`. `arrival_direction` is NOT hardcoded — it's
`sign(target_Z - mission_parameters.mInitialPosition(3))`, computed by the caller
(`run_continuous_simulation.m`): `-1` for a normal descent (NWU Z decreases toward the
target), `+1` for `main.m`'s Return Ascent leg (paper.tex Phase 3 — same mission,
boundary conditions reversed, so Z *increases* toward its target instead). Getting this
wrong means the event either fires immediately (Z already satisfies the wrong-direction
crossing at `t=0`) or never fires at all. Note: `run_discrete_simulation.m` implements
its own inline touchdown/arrival check (same `is_ascending` logic, not reusing this
function) rather than an ODE event — the discrete loop isn't ODE-event-driven.

## `log_results(t_out, x_out, mission_parameters, log_folder)` — `utils/log_results.m`

Writes a timestamped `sim_run_HH-MM-SS_dd-mm-yyyy.log` text file (defaults to `pwd` if
`log_folder` omitted, creates the folder if missing) with flight time, fuel burned,
miss distance (via `calculate_miss_distance`), intermediate-point info (if
`bUseIntermediatePoint` and any `gammaIntermediatePositionWeights` nonzero), impact
speed, and terminal attitude/rates. Purely a reporting side-effect — does not return
anything or affect the simulation.

## `visualization(hovercraft_parameters, mission_parameters, t_out, x_out)` — `utils/visualization.m`

Generates 4 figures (skips Figure 4 if `x_out` doesn't have the 19-state actuator
columns, i.e. `size(x_out,2) < 19`):

1. **3D trajectory** — `plot3` of NWU position, start/touchdown markers, intermediate
   waypoint marker (same `has_intermediate_point` check as `log_results`), terminal
   metrics text annotation.
2. **Translational telemetry** — 2×4 tiled: x/y/z/range-from-start, u/v/w/total velocity
   vs. time.
3. **Rotational telemetry & mass** — 3×3 tiled: roll/pitch/yaw + rates (deg, deg/s), plus
   system mass spanning the bottom tile.
4. **Actuator telemetry** (only if actuator states present) — 2×3 tiled, all 6 engine
   throttles in % with dashed lines at `hovercraft_parameters.pctThrottleRange` limits.

Uses `tiledlayout` when available (`~verLessThan('matlab','9.1')`) with a `subplot`
fallback for older MATLAB. Purely presentational — no return value, called last in
`main.m` after `log_results`.
