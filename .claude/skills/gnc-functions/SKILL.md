---
name: gnc-functions
description: Reference for every function in gnc/ (calculate_guidance_law, calculate_axis_control, propagate_axis_closed_form, evaluate_axis_trajectory, select_intermediate_point, run_attitude_controller, allocate_controls, calculate_allocation_matrices, calculate_throttles_command, closed_loop_system) for the lunar hovercraft powered-descent GNC project (0880785 - Final Project). Use this whenever the user asks about the guidance law, the Nataf & Shaferman LQ/ZEM-ZEV intermediate-point law, attitude control, control allocation, throttle/PWM/MIB logic, the closed-loop ODE function, or wants to modify/debug/extend anything under gnc/ — so the functions don't need to be re-read from disk to recall their signatures, math, or how they call each other.
---

# gnc/ functions — lunar hovercraft powered-descent GNC project

Two independent subsystems live in `gnc/`:

1. **Guidance** (outer loop) — implements Nataf & Shaferman (2024), "Optimal Linear
   Quadratic Powered Descent With An Optimally Selected Intermediate Point," AIAA
   SciTech 2024. Produces a commanded thrust-acceleration vector.
   → `references/guidance-law.md`
2. **Attitude control + allocation** (inner loop / actuation) — converts the guidance
   acceleration command into 6 physical engine throttles. → `references/control-allocation.md`

`gnc/closed_loop_system.m` wires both subsystems plus the physics plant together for the
continuous (`ode15s`) simulation path; `run_discrete_simulation.m` (project root, not in
`gnc/`) wires them together for the discrete/hardware-representative path.

## Call graph

```
main.m
 └─ calculate_allocation_matrices(hovercraft_parameters)      [once, precomputes B_pinv]
 └─ select_intermediate_point(mission_parameters)              [offline, before each run_simulation]
     ├─ propagate_axis_closed_form  (×3 axes, gamma=0 pass — Step 1 cone check)
     ├─ evaluate_axis_trajectory
     ├─ propagate_axis_closed_form  (Step 3 ground-collision check, z-axis only)
     └─ propagate_axis_closed_form  (×3 samples ×2 axes — Step 4 quadratic-cost fit)
 └─ run_simulation(mission_parameters, hovercraft_parameters)
     ├─ run_continuous_simulation → ode15s(closed_loop_system, ...)
     │    └─ closed_loop_system(t, x, GP, hovercraft_parameters, mission_parameters)
     │        ├─ calculate_guidance_law → calculate_axis_control (×3 axes)
     │        ├─ run_attitude_controller
     │        ├─ allocate_controls
     │        └─ calculate_dynamics                              [physics, not in gnc/]
     └─ run_discrete_simulation (50 Hz loop, same 4 GNC calls per tick, plus)
          └─ calculate_throttles_command                          [PWM/MIB hardware logic]
```

`calculate_guidance_law` and `calculate_axis_control` are also exercised directly (no
simulation loop) by `tests/test_point_mass_guidance.m` and by `select_intermediate_point`
via `propagate_axis_closed_form`, which re-derives the same per-axis 3×3 system once at
t=0 instead of calling `calculate_axis_control` every tick.

## Frame convention — read this before touching guidance math

The project state uses **NWU** (North-West-Up, Z positive up, origin at mission start).
The Nataf & Shaferman paper's frame has **origin at the target, Z positive down**.
`utils/flip_z_axis.m` converts between them: flips the sign of Z only (X/Y already align
with downrange/crossrange). It's involutory — same function both directions. Positions
must be made target-relative (`pos - target_nwu`) before flipping; velocities/
accelerations/gravity flip directly.

Every guidance function below (`calculate_guidance_law`, `select_intermediate_point`,
`propagate_axis_closed_form`, `calculate_axis_control`, `evaluate_axis_trajectory`)
operates in the **paper's target-relative Z-down frame** except `calculate_guidance_law`
and `select_intermediate_point`, which are the two boundary functions that convert
NWU ↔ paper frame at entry/exit.

## Quick reference

| Function | Role |
|---|---|
| `calculate_guidance_law` | Real-time outer-loop guidance; NWU state in, NWU accel-command out. Calls `calculate_axis_control` ×3. |
| `calculate_axis_control` | Solves one decoupled-axis 3×3 LQ system (Eq 41-49) at the current tgo; returns `u` and the 3×3 inverse `f`. |
| `propagate_axis_closed_form` | Offline exact polynomial trajectory shape for one axis (solves the 3×3 system once at t=0, integrates analytically). Used by `select_intermediate_point`, not the real-time loop. |
| `evaluate_axis_trajectory` | Evaluates a `propagate_axis_closed_form` struct at arbitrary query times. |
| `select_intermediate_point` | Offline: picks waypoint position + time so the trajectory passes through an approach cone without ground collision, minimizing cost. Mutates `mission_parameters`. |
| `run_attitude_controller` | Inner-loop pitch PD (dynamic inversion) + thrust-magnitude sizing; NWU accel command → desired Fz/My. |
| `allocate_controls` | Fz/My → 6 engine throttles via 4-channel symmetric pseudo-inverse mixer, clamped to [0,1]. |
| `calculate_allocation_matrices` | Precomputes `B_pinv` (the mixer) from vehicle geometry; called once in `main.m`. |
| `calculate_throttles_command` | Hardware PWM/MIB (minimum-impulse-bit) throttle-shaping state machine; run every discrete tick. |
| `closed_loop_system` | `ode15s`-compatible RHS: navigation (passthrough) → guidance → attitude control → allocation → physics. Has 2 documented architectural limitations (Y-axis guidance discarded, GNC runs at ODE sub-step rate). |

See the two reference files for full signatures, equation numbers, and per-function gotchas.
