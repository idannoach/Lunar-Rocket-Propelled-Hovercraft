---
name: falcon-optimal-control
description: How to build, solve, and troubleshoot direct-collocation optimal control problems with the FALCON.m MATLAB toolbox (installed at C:\Program Files\falcon) for the lunar hovercraft powered-descent GNC project (0880785 - Final Project). Use this whenever the user mentions FALCON.m, falcon.Problem, falcon.State/Control/Parameter, trajectory optimization, optimal control, IPOPT, direct collocation, minimum-fuel or minimum-time powered descent, or wants to generate/compare an optimal reference trajectory against the project's closed-form LQ guidance law (gnc/calculate_guidance_law.m). Also use it if a FALCON.m script throws a Build/Bake error, a "sym is not recognized as a class" error, or any error involving falcon.core.builder — this skill has known, install-specific fixes.
---

# FALCON.m for the lunar hovercraft powered-descent project

## What FALCON.m is

FALCON.m is the optimAL CONtrol toolbox for MATLAB from TU Munich's Institute of
Flight System Dynamics. It solves open-loop optimal control problems of the form
"minimize a cost functional subject to dynamics ẋ = f(x,u,p), bounds, and path/point
constraints" using direct collocation + a gradient-based NLP solver (IPOPT by default).

It is installed locally at `C:\Program Files\falcon` (already on this machine — do not
reinstall). Key resources there:

- `UserGuideMain.pdf` — the full 190-page manual. Section 3 (Quick Start Guide) is the
  fastest path to a working problem; Section 5 is the class-by-class API reference;
  Section 5.16 lists reusable cost/constraint building blocks.
- `examples\` — runnable example problems, most usefully `SimpleCarProblem`,
  `IntroductionExamples\01_Aircraft2D` and `02_Aircraft3D`, and `FalconHuntingPrey`
  (a genuine multi-phase problem with path constraints).
- `StartupCheck.m` — run this once per MATLAB session/install to confirm the toolbox,
  compiler, and IPOPT are all wired up correctly.

Condensed, project-oriented references (read these before re-deriving things from the
190-page PDF):

- `references/api-cheatsheet.md` — the exact FALCON.m syntax patterns (State/Control/
  Parameter/Constraint constructors, Problem/Phase methods, cost types, solver setup).
- `references/project-integration.md` — how this specific hovercraft project's dynamics,
  units, and mission files map onto a FALCON.m model, with a ready-to-adapt skeleton.
- `references/troubleshooting.md` — known failure modes on this exact install (FALCON.m
  v1.33 on MATLAB R2026a), including a real error this project already hit.

## Why this project would reach for FALCON.m

`gnc/calculate_guidance_law.m` already implements a **closed-form** LQ/ZEM-ZEV soft-
landing guidance law (analytic, evaluated every control cycle). The `Essays/` folder
(convex-programming powered descent, ZEM/ZEV waypoint guidance, LQ powered descent
with an optimally-selected intermediate point) shows the thesis is comparing guidance
strategies. FALCON.m's role in that comparison is to produce an **independently
optimized reference trajectory** (e.g. minimum-fuel or minimum-time powered descent)
by direct collocation, so the analytic guidance law's performance/optimality gap can be
quantified against a numerically-optimal benchmark — not to replace the real-time
guidance law itself (FALCON.m solves offline/open-loop; it is not a flight computer).

So the natural use is: build a separate FALCON.m problem describing the powered-descent
translational (and optionally rotational) dynamics, solve it for the same mission
(`config/mission.json` initial state → `mTargetPosition`), and compare the resulting
trajectory/fuel usage against `run_discrete_simulation.m`'s output for the same mission.
See `references/project-integration.md` for a concrete state/control mapping and a
skeleton `main.m` + model function to start from.

## The four-step mental model

Every FALCON.m problem follows the same sequence — keep this in mind whenever writing
or debugging a script:

1. **Define** — value objects (`falcon.State`, `falcon.Control`, `falcon.Parameter`),
   the model dynamics function, the `falcon.Problem`, its phase(s), boundaries, costs,
   and constraints.
2. **Build derivatives** — triggered by `problem.Bake()` (or the first `Solve()`, which
   calls `Bake()` for you). FALCON.m symbolically differentiates your model/constraint
   functions and compiles MEX files. This step is slow the first time and fast on
   reruns (cached), and it's where most beginner errors surface (see troubleshooting).
3. **Prepare** — `problem.Bake()` checks consistency (every state/control/parameter
   referenced by every model/constraint must actually be registered on that phase).
4. **Solve** — `solver = falcon.solver.ipopt(problem); solver.Solve();`

## Minimal working skeleton

```matlab
falcon.init();

% 1. Value definitions: Name, LowerBound, UpperBound, Scaling
states   = [falcon.State('x', -inf, inf, 1e-3); ...];
controls = [falcon.Control('u', -1, 1, 1); ...];
tf       = falcon.Parameter('FinalTime', 20, 0, 40, 0.1);  % free final time

% 2. Problem + phase
problem = falcon.Problem('MyProblem');
tau = linspace(0, 1, 101);                       % normalized time grid
phase = problem.addNewPhase(@my_dynamics, states, tau, 0, tf);
phase.addNewControlGrid(controls, tau);
phase.Model.setModelOutputs([]);                 % or falcon.Output(...) if your model returns extras

% 3. Boundaries — one vector = equality, two vectors = [lower, upper]
phase.setInitialBoundaries([...]);
phase.setFinalBoundaries([...]);

% 4. Cost (pick one appropriate to the objective)
problem.addNewLinearPointCost(tf);               % e.g. minimize final time
% or: phase.addNewQuadraticPathCost(controls(1));  % e.g. minimize control effort

% 5. Bake + solve
problem.Bake();
solver = falcon.solver.ipopt(problem);
solver.Options.MajorIterLimit = 500;
solver.Options.MajorFeasTol   = 1e-5;
solver.Options.MajorOptTol    = 1e-5;
solver.Solve();

% 6. Results
t = phase.RealTime;
x = phase.StateGrid.Values;      % states x samples
u = phase.ControlGrids.Values;   % controls x samples
```

`my_dynamics` must have the exact signature FALCON.m expects:

```matlab
function [states_dot] = my_dynamics(states, controls)
% (add a second output `y` here only if you called setModelOutputs with falcon.Output objects)
x1 = states(1); ...
u1 = controls(1); ...
states_dot = [ ... ];
end
```

If you reference `@my_dynamics` before the file exists, FALCON.m offers to
auto-generate the correctly-shaped stub for you the first time you run the script —
answer `y` in the MATLAB console, then fill in the body.

Dynamics must be **autonomous** (no direct dependence on time `t`); if a problem
genuinely needs non-autonomous dynamics, add a state `t` with `ṫ = 1` instead.

For the full syntax of every constructor, `Problem`/`Phase` method, cost type, and the
`falcon.PathConstraintBuilder`/`PointConstraintBuilder` (recommended once a model
stabilizes — they cut down which symbolic derivatives get built, and avoid the
awkward "FALCON.m owns the file" auto-stub flow), see `references/api-cheatsheet.md`.

## Multi-phase problems

Use multiple phases when the mission has distinct legs (e.g. this project's
outwards/return trip, or a braking phase followed by a terminal vertical-descent
phase — a common pattern in the powered-descent literature in `Essays/`). Chain phases
with `problem.ConnectAllPhases()` for simple start/end continuity, or
`falcon.PointConstraintBuilder` for connecting phases at arbitrary time points (e.g.
symmetry/periodicity constraints). See `references/api-cheatsheet.md` §Multi-phase.

## Scaling and initial guesses

FALCON.m is a gradient-based solver — it is sensitive to poor scaling and needs
initial guesses for everything being optimized:

- Choose each `Scaling` argument so `value * Scaling ≈ O(1)`. For this project, position
  in meters (~tens of thousands) needs a small scaling factor (~1e-4–1e-5); mass in kg
  (~tens) needs ~1e-2–1; lunar-scale accelerations are already O(1).
- If you don't supply an initial guess, FALCON.m tries to build one itself — for a
  descent trajectory it's usually worth supplying a straight-line-ish guess via
  `phase.StateGrid.setValues(tau, guess_matrix)` so IPOPT starts somewhere physical.

## Known install-specific issue

This exact FALCON.m install (v1.33 on MATLAB R2026a) has already thrown a
`Build trace entry failed ... 'sym' is not recognized as a class` error during
`Bake()`/`Solve()` — see `errorReport_20260703T130750.txt` next to the toolbox and
`references/troubleshooting.md` for the diagnosis and fix before assuming the model
code itself is wrong.
