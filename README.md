# Soft-Landing Guidance and Control for a Lunar Rocket-Propelled Hovercraft

MATLAB simulation and trajectory-optimization environment for a Master's final
research project (Faculty of Aerospace Engineering, supervised by Prof. Oded
Golan). It models a rocket-propelled hovercraft that descends from the rim of
the Shoemaker crater to a target way-point, using a closed-form LQ/ZEM-ZEV
guidance law with an optimally-selected intermediate way-point, and validates
the resulting trajectories against numerically-optimal benchmarks generated
with the FALCON.m direct-collocation toolbox.

Full technical background, derivations, and results are documented in
`paper.tex` (parent directory).

## Overview

The project implements two independent ways of solving the same
powered-descent problem, so they can be compared directly:

- **Closed-form GNC stack** (`gnc/`, `src/`) — a discrete-time
  navigation → guidance → control → plant loop built on the Nataf & Shaferman
  Intermediate Way-point LQ/ZEM-ZEV guidance law, driving a full 6-DOF rigid
  body vehicle model (or a reduced point-mass model for benchmarking).
- **Numerically-optimal benchmark** (`opt_control/`) — the same boundary
  conditions solved as a minimum-fuel/minimum-effort optimal control problem
  via direct collocation in FALCON.m/IPOPT, used as an independent check on
  the closed-form law's optimality.

Running `main.m` executes three paired comparisons (point-mass, point-mass
with an intermediate point, and the full 6-DOF project mission), each
reporting flight time, fuel consumption, and terminal miss distance/velocity
for the closed-form law against its FALCON.m benchmark, and produces
matching visualizations.

## Repository layout

```
config/         Vehicle (hovercraft.json) and mission (mission.json) parameter
                files, plus global constants (GP.m)
gnc/
  guidance/     LQ/ZEM-ZEV guidance law, intermediate-point selection
  control/      Control allocation and engine PWM/MIB throttle logic
  navigation/   Navigation pass-through/state estimation stub
src/            6-DOF and point-mass rigid-body dynamics, thrust/engine model
opt_control/    FALCON.m problem setup and the point-mass optimal-control runs
fm_models/      FALCON.m-generated (baked/MEX) model artifacts
fm_constraints/ FALCON.m-generated (baked/MEX) constraint artifacts
utils/          JSON loading, logging, visualization, and comparison reporting
tests/          Test scripts
logs/           Simulation run logs (generated at runtime)
main.m                    Top-level entry point: runs all mission comparisons
solve_mission.m            Outer loop that converges the final time t_f
run_phase1_simulation.m    Discrete-time navigation/guidance/control/plant loop
startup.m                  Path setup and parameter loading
```

## Requirements

- MATLAB (R2026a or later recommended)
- [FALCON.m](https://www.falcon-m.com/) direct-collocation toolbox and a
  licensed IPOPT installation, for the optimal-control benchmark runs
- A MEX-compatible C/C++ compiler configured in MATLAB (`mex -setup`), used by
  FALCON.m to build the benchmark problems

## Usage

From MATLAB, with the project folder as the working directory:

```matlab
main
```

This loads global, vehicle, and mission parameters via `startup.m`, then runs
each mission pair enabled in `config/mission.json` (`isRunBenchmark`,
`isRunFullSim`). Console output reports guidance convergence and a
side-by-side comparison table against the FALCON.m benchmark; results are
logged to `logs/` and plotted via `utils/visualization.m` or
`utils/report_point_mass_comparison.m`.

Mission and vehicle parameters (initial/target state, tolerances, thrust
budget, mass properties, etc.) are configured in `config/mission.json` and
`config/hovercraft.json` — no code changes are required to adjust a scenario.

### Note on the point-mass benchmark's thrust budget

The point-mass missions (1-4) use a thrust budget boosted 2x above the
vehicle's real value in `config/hovercraft.json` (6 x 20 N). At the real
budget, FALCON.m's hard-thrust-bounded minimum-fuel problem does not converge
for this scenario, since net maneuvering authority (~0.375 m/s²) is
insufficient for the required ~150 km descent from a ~1 km/s closing speed —
a materially harder problem than the closed-form law's unconstrained
minimum-control-effort formulation. This adjustment is isolated to the
point-mass benchmark vehicle only; the full 6-DOF project mission (5-6) uses
the vehicle's real, unmodified thrust budget throughout.
