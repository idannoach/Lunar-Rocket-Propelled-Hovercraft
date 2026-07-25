---
name: vehicle-dynamics
description: Reference for the 6-DOF physics plant in src/ (calculate_dynamics, calculate_rigid_body_dynamics, calculate_translational_dynamics, calculate_rotational_dynamics, calculate_euler_kinematics, calculate_thrust_forces, calculate_thrust_vec, calculate_engine_dynamics, calculate_moments_of_inertia, calculate_moments_of_inertia_poly) and the shared support functions in utils/ and config/GP.m for the lunar hovercraft powered-descent GNC project (0880785 - Final Project). Use this whenever the user asks about the rigid-body/6-DOF dynamics, thrust/engine modeling, moments of inertia, Euler kinematics, the state vector layout, frame conversion, mission/hovercraft JSON loading, logging, or visualization — so the functions don't need to be re-read from disk to recall their signatures, physics, or how they call each other.
---

# src/, utils/, config/ — physics plant and support functions

`src/` is the **physics plant** ("the world") — a self-contained 6-DOF rigid-body
simulation of the hovercraft, driven purely by throttle commands (no guidance/control
logic lives here; that's `gnc/`, see the `gnc-functions` skill). `utils/` and
`config/GP.m` are shared support functions used across the whole pipeline (frame
conversion, state packing, JSON I/O, logging, plotting, constants).

## State vector layout

19×1, packed by `utils/pack_initial_state_vector.m`:

| Indices | Field | Frame/units |
|---|---|---|
| 1:3 | position | NWU [m] |
| 4:6 | velocity `[u;v;w]` | Body frame [m/s] |
| 7:9 | Euler angles `[phi;theta;psi]` (roll/pitch/yaw) | rad |
| 10:12 | angular rates `[p;q;r]` | Body frame [rad/s] |
| 13 | mass | kg |
| 14:19 | actual (lagged) engine throttles, 6 engines | [0,1] |

`x_rb = x(1:13)` is the "rigid-body" sub-state most `src/` functions operate on;
`x(14:19)` are the engine-lag states handled separately by `calculate_engine_dynamics`.

## Call graph

```
calculate_dynamics(t, x, cmd_throttles, GP, HP)        [ode15s RHS for the 13+6 state, called by gnc/closed_loop_system.m]
 ├─ calculate_rigid_body_dynamics(x(1:13), actual_throttles, GP, HP)
 │   ├─ calculate_thrust_forces(throttles, HP, GP)
 │   │   └─ calculate_thrust_vec(...)                   [per engine, ×nEngines]
 │   ├─ calculate_body_to_nwu_matrix(phi, theta, psi)    [utils/, DCM]
 │   ├─ calculate_euler_kinematics(rates, phi, theta)
 │   ├─ calculate_translational_dynamics(Forces_body, mass, R_b2n, GP, r,p,q,u,v,w)
 │   └─ calculate_rotational_dynamics(Moments_body, mass, HP, r,p,q)
 │       └─ calculate_moments_of_inertia(mass, HP)
 └─ calculate_engine_dynamics(cmd_throttles, actual_throttles, HP)

calculate_moments_of_inertia_poly(hovercraft_parameters)   [offline, called once in main.m — precomputes Ixx/Iyy/Izz_poly consumed by calculate_moments_of_inertia]
```

`run_discrete_simulation.m` (project root) calls `calculate_rigid_body_dynamics`
directly per RK4 sub-stage (bypassing `calculate_dynamics`/`calculate_engine_dynamics`)
because it evaluates the engine lag analytically instead — see the `gnc-functions`
skill's control-allocation reference for that loop.

## Quick reference — `src/`

| Function | Role |
|---|---|
| `calculate_dynamics` | Top-level ODE RHS: splits state into rigid-body (1:13) + engine-lag (14:19), calls both sub-dynamics, concatenates derivatives. |
| `calculate_rigid_body_dynamics` | 13-state derivative: thrust → forces/moments → translational/rotational accel → pos/euler kinematics → mass depletion. |
| `calculate_translational_dynamics` | Newton's 2nd law in the rotating Body frame: `a = F/m + g_body - omega×v` (Coriolis term). |
| `calculate_rotational_dynamics` | Euler's rigid-body equations: `alpha = I⁻¹(M - omega×(I*omega))`, diagonal `I` (assumes principal-axis symmetry). |
| `calculate_euler_kinematics` | Body rates → Euler-angle rates via the Z-Y-X kinematic matrix. Errors on gimbal lock (`|cos(theta)| < 1e-6`). |
| `calculate_thrust_forces` | Sums all engines' thrust vectors + moments (`M = r_arm × T`) + total mass flow. |
| `calculate_thrust_vec` | Single-engine thrust vector from mass flow/Isp (rocket equation), saturated at `newtonMaxThrust`. |
| `calculate_engine_dynamics` | First-order throttle lag: `throttles_dot = (cmd - actual)/tau`, `tau = t90%/2.3`. |
| `calculate_moments_of_inertia` | Evaluates the linear mass→inertia polynomials for current `Ixx,Iyy,Izz` (diagonal tensor). |
| `calculate_moments_of_inertia_poly` | Precomputes those polynomials (once, in `main.m`) from full/empty mass and min/max inertia bounds. Errors if `kgFuelMass == 0` (undefined slope). |

Full per-function detail, equations, and gotchas: `references/dynamics.md`.

Support functions (`utils/`, `config/GP.m`): `references/support-utils.md`.
