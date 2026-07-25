# Guidance law (Nataf & Shaferman 2024)

Paper: "Optimal Linear Quadratic Powered Descent With An Optimally Selected Intermediate
Point," AIAA SciTech 2024. Softly weights terminal position miss, terminal velocity
error, and (optionally) miss at an intermediate waypoint, while minimizing integrated
control effort. All frame notes assume the target-relative, Z-down paper frame unless
stated otherwise — see SKILL.md's "Frame convention" section first.

## `calculate_guidance_law(t, current_state, GP, mission_parameters)` — `gnc/calculate_guidance_law.m`

Real-time outer-loop entry point, called every controller tick (or every ODE sub-step in
the continuous path).

- **Inputs**: `t` (sim time), `current_state` (19×1: pos[1:3] NWU, vel[4:6] body frame,
  euler[7:9], ... — see `pack_initial_state_vector.m` for the full layout), `GP` (global
  parameters struct, has `g_lunar`), `mission_parameters` (mission JSON, see fields below).
- **Output**: `[cmd_accel_x, cmd_accel_y, cmd_accel_z]` — NWU thrust-acceleration command.
  This is the **complete** command; gravity drift is already accounted for inside the
  Zero-Effort-Miss terms, so callers must NOT add separate gravity compensation.
- **Flow**: rotate body velocity to NWU via `calculate_body_to_nwu_matrix`, convert
  pos/vel/gravity to target-relative Z-down via `flip_z_axis`, compute `tgo_f = tf - t`
  and (if `mission_parameters.bUseIntermediatePoint`) `tgo_1 = t1 - t`, compute
  Zero-Effort-Miss/Velocity/Intermediate-Miss per axis (Eq 48a-c), call
  `calculate_axis_control` once per axis, flip the result back to NWU.
- **Intermediate point disabled**: when `bUseIntermediatePoint` is false, `t1=tf`,
  `rc=[0;0;0]`, `gamma_w=[0;0;0]` are forced — `gamma=0` makes the intermediate-point
  term vanish identically in `calculate_axis_control` regardless of `t1`/`rc`, so the law
  degrades automatically to the classical terminal-only soft-constrained law.
- **Known limitation** (documented in `gnc/closed_loop_system.m`): the Y-axis
  (`cmd_accel_y`) is discarded by the only caller that matters (`closed_loop_system`),
  since `run_attitude_controller` only controls pitch (X-Z plane). A non-zero East target
  will accumulate uncorrected Y error until a roll-control loop is added.
- **mission_parameters fields read**: `mTargetPosition`, `sFinalSimulationTime`,
  `bUseIntermediatePoint`, `sIntermediatePointTime`, `mIntermediatePointPosition`,
  `gammaIntermediatePositionWeights`, `alphaFinalPositionWeights`, `betaFinalVelocityWeights`.

## `calculate_axis_control(tgo_f, tgo_1, tf, t1, alpha, beta, gamma, Zbar_fr, Zbar_fV, DeltaZbar_1)` — `gnc/calculate_axis_control.m`

Solves ONE decoupled-axis scalar two-point boundary value problem (the 3-axis problem
decouples because the weighting matrices M_frr/M_fVV/M_1rr are diagonal, Eq 7-8).

- **Inputs are all scalars in the paper's frame/units** — caller does any NWU conversion.
  `tgo_1` should be 0 for `t > t1` (matches the paper's step function 1(tgo_1)).
  `gamma=0` disables the intermediate-point term for this axis (law degrades automatically).
- **Outputs**: `u` (complete thrust-acceleration command for this axis, Eq 49 — already
  includes gravity drift via the Zbar terms, no separate compensation needed) and `f`
  (the 3×3 inverse matrix from Eq 43/44, returned so `propagate_axis_closed_form` can
  reuse it without re-solving).
- **Implementation choice**: builds the symmetric 3×3 system `S` (Eq 41-42) directly and
  calls `inv(S)`, rather than transcribing the paper's expanded closed-form f_ij/Lambda_i
  rational expressions (Eq 44-45) by hand — mathematically identical, far less
  error-prone. Proposition 1 in the paper (S = I + P, P PSD) guarantees `S` is always
  invertible, so there's no singularity handling.
- **Gotcha if extending**: the gains (`gain_r`, `gain_V`, `gain_1`) already fold in
  `alpha`/`beta`/`gamma` a second time when multiplying by `Zbar_fr`/`Zbar_fV`/
  `DeltaZbar_1` in the final `u` expression — don't double-weight if refactoring.

## `propagate_axis_closed_form(r0, V0, g, alpha, beta, gamma, rc, tf, t1)` — `gnc/propagate_axis_closed_form.m`

**Offline** exact (non-numerical) trajectory shape for one axis. Used by
`select_intermediate_point` for trajectory shaping — NOT by the real-time loop, which
re-solves `calculate_axis_control` every tick against the true evolving state.

- **Key insight it exploits**: along the optimal trajectory, the terminal
  Zero-Effort-Miss/Velocity and Zero-Effort-Intermediate-Miss are CONSTANTS of motion, so
  the 3×3 system only needs solving once (at t=0) via `calculate_axis_control`. The
  resulting `u*(t)` is then piecewise-affine in t (breakpoint at `t1`), so exact
  closed-form quadratic V(t) / cubic r(t) polynomials follow by direct analytic
  integration (reproduces the paper's Eq 58/60 structure without hand-transcribing it).
- **Output struct `traj`** fields: `r_t1, V_t1` (state at t1), `r_tf, V_tf` (state at tf),
  `u_sq_integral` (exact ∫u²dt, no numerical-integration error), `max_r` (exact max of
  r(t) over [0,tf]), `seg` (1×2 struct array of per-segment polynomial coefficients for
  `evaluate_axis_trajectory`).
- **Local helper `local_build_segment(ta, tb, c0, c1, ra, Va, g)`**: given affine control
  `u(t)=c0+c1*t` on `[ta,tb]` and state at `ta`, returns exact state at `tb`, exact
  `u_sq_integral`, and exact extrema of `r(t)` (via `roots()` on dr/dt=0, filtered to
  real roots inside `(0, dur)`).

## `evaluate_axis_trajectory(traj, t_query)` — `gnc/evaluate_axis_trajectory.m`

Evaluates a `propagate_axis_closed_form` struct at arbitrary query times (vector).
Selects segment 1 or 2 based on `t <= traj.t1`, clamps `tau` defensively to the segment
domain, then evaluates the closed-form position/velocity/control polynomials directly.
Returns `[r, V, u]`, each same size as `t_query`.

## `select_intermediate_point(mission_parameters)` — `gnc/select_intermediate_point.m`

**Offline**, called once per `main.m` t_f-iteration (before `run_simulation`). Implements
Sec IV.B of the paper: automatically choose waypoint position + time so the trajectory
passes through an approach cone above the target and avoids ground collision, while
minimizing cost — operating on the idealized point-mass model (matching the paper's own
methodology), independent of the full 6-DOF dynamics.

**Direction-agnostic (descent OR ascent)**: every check below is written assuming the
trajectory starts ABOVE the target in the paper's Z-down frame (`r0(3) < 0`, paper-Z
rising to 0 — a descent). `main.m`'s Return Ascent leg (paper.tex Phase 3) reverses the
boundary conditions, so `r0(3) > 0` instead (starts below the target, paper-Z falling to
0) — without correction every check below is either trivially satisfied at `t=0` or looks
for the wrong crossing, and the function never converges (h inflates to its iteration cap
every candidate). Fixed via `z_mirror = -sign(r0(3))`: axis-3 calls to
`propagate_axis_closed_form` get `z_mirror*r0(3)/V0(3)/g(3)` instead of the raw values —
a valid transform of the scalar dynamics (flipping r, V, g by the same constant just
flips the resulting trajectory by that constant too) that makes an ascent's Z-trajectory
look like a canonical descent to every check, so none of them needed rewriting. Only the
final `mIntermediatePointPosition` un-mirrors back (`z_mirror*(-h)`), so the waypoint
lands on the correct side of the target either way. X/Y are never touched (`flip_z_axis`
never flips them, so there's no ambiguity there).

Four steps, each worth knowing when debugging a bad waypoint:

1. **Cone-pass check (gamma=0)**: propagate all 3 axes with no intermediate point,
   sample 2000 points over `[0,tf]`, find where `r_z >= -h` (cone height), check the
   XY radius at that crossing is `<= Rmax` AND no earlier sample had `r_z >= 0`
   (ground collision before reaching the cone). If it already passes, sets
   `gammaIntermediatePositionWeights = [0;0;0]` and returns early — **no waypoint used**.
2. **t1 selection (Eq 57)**: last time `||r(t)||` crosses the cone's slant distance
   `Rslant = sqrt(Rmax^2 + h^2)`, found via `interp1` on the sign change of
   `rnorm - Rslant`. Falls back to `tf/2` with a warning if no crossing is found.
3. **Ground-collision check**: z-axis only, restricted to `[0, t1]` (NOT `[0,tf]` —
   matches the paper's worked example; using the full domain would false-positive on
   any converged trajectory since `r_z(tf)~=0` is the intended touchdown). If it
   collides, grows `h` by `mConeHeightIncrement` (keeping `Rmax/h` ratio fixed) and
   retries from step 1, up to `iMaxConeHeightIterations`.
4. **Optimal `(rc_x, rc_y)` (Eq 70-78)**: cost `J_j(rc_j)` is exactly quadratic in
   `rc_j`, so 3 sample points (`-Rmax, 0, Rmax`) determine the parabola exactly via
   `polyfit` (not a lossy fit). If the unconstrained minimum lies inside the circle of
   radius `Rmax`, use it directly; otherwise solve the constrained minimum on the
   circle boundary via a Lagrange multiplier root-find (`fzero`, monotonic
   `h_lambda`, guaranteed one root in `[0,inf)` per the paper's Eq 73a curvature argument).

- **mission_parameters fields read**: `mTargetPosition`, `mInitialPosition`,
  `mpsInitialVelocity`, `sFinalSimulationTime`, `alpha/beta/gammaFinalPositionWeights`,
  `mApproachConeRadius`, `mApproachConeHeight`, `mConeHeightIncrement`,
  `iMaxConeHeightIterations`.
- **mission_parameters fields written**: `gammaIntermediatePositionWeights`,
  `mIntermediatePointPosition`, `sIntermediatePointTime`, `mApproachConeRadius`,
  `mApproachConeHeight`.

## Tests

`tests/test_point_mass_guidance.m` — standalone script (no test framework in this repo;
prints PASS/FAIL), exercises the guidance law directly on the point-mass model using the
project's real `mission.json`/`hovercraft.json`. Five checks: (1) gamma=0 convergence
sign sanity, (2) intermediate-point pull-through, (3) gravity not double-counted at
target with zero error, (4) reduction to the old hard-constraint law
`(6/tgo^2)*dr - (4/tgo)*V` as alpha/beta→∞, (5, informational) peak commanded
acceleration vs. vehicle thrust envelope — warns if throttle saturation is expected.
Run via `test_point_mass_guidance` from the project root after `startup()`.
