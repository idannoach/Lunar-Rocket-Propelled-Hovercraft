# FALCON.m API cheatsheet

Condensed from `C:\Program Files\falcon\UserGuideMain.pdf` (Sections 3 and 5) and the
bundled examples. This covers the ~90% case; for anything not here, read the PDF page
number listed — the table of contents maps 1:1 to section numbers.

## Value definition objects

```matlab
falcon.State(Name, LowerBound, UpperBound, Scaling)
falcon.Control(Name, LowerBound, UpperBound, Scaling)
falcon.Parameter(Name, InitialValue, LowerBound, UpperBound, Scaling)
falcon.Constraint(Name, LowerBound, UpperBound)          % used for path/point constraints
falcon.Output(Name)                                       % declares an extra model output
falcon.Value(Name)                                        % for post-processing steps
```

States/Controls are always vectors evaluated on a time grid; Parameters are scalars
(e.g. a free final time). Bounds use `-inf`/`inf` freely. A single-vector call to
`setInitialBoundaries`/`setFinalBoundaries` means lower==upper (equality); two vectors
mean `[lower, upper]` (inequality).

## Problem / Phase construction

```matlab
problem = falcon.Problem('Name');
tau = linspace(0, 1, N);                                  % normalized time grid, per phase

phase = problem.addNewPhase(@dynamics_fn, states_vec, tau, t0, tf);
% t0/tf can be numeric (fixed) or a falcon.Parameter (free, subject to optimization)

phase.addNewControlGrid(controls_vec, tau);                % tau optional, defaults to state grid
phase.Model.setModelOutputs([falcon.Output('y1'); falcon.Output('y2')]);  % if dynamics_fn returns y

phase.setInitialBoundaries(lb [, ub]);
phase.setFinalBoundaries(lb [, ub]);
phase.StateGrid.setValues(tau, guessMatrix);                % optional initial guess, states x samples
```

Model function signature (with outputs):

```matlab
function [states_dot, y] = dynamics_fn(states, controls)
```

If the model needs `falcon.Parameter`s too (e.g. vehicle mass as a design parameter),
the phase's model call is instead built through the model wrapper — simplest path is
still `addNewPhase(@fn, states, tau, t0, tf)` and let FALCON.m ask to regenerate the stub
if the parameter set changes; for hand-authored models with parameters use
`phase.Model.setModelParameters(paramVec)`.

## Cost functions

Pick based on what's being minimized (`falcon.Problem` page ~29, `falcon.core.Phase`
page ~49):

```matlab
problem.addNewLinearPointCost(value)          % Mayer term, e.g. minimize tf or -m(tf) for max mass
problem.addNewMayerCost(...)                  % general point cost at phase boundary
phase.addNewLinearPathCost(value)             % Lagrange term, integrated over the phase
phase.addNewQuadraticPathCost(value)          % e.g. minimize integral of control^2 (min-effort)
problem.addNewQuadraticPointCost(...)
```

To maximize instead of minimize, negate the cost expression (`J = -J̄`).

For fuel-optimal problems specifically: minimizing `-m(tf)` (final mass) via a linear
point cost is the standard formulation, since mass depletion is monotonic with thrust
usage and this avoids needing `|thrust|` inside a path cost.

## Path and point constraints

```matlab
% Auto-generated function (FALCON.m offers to create the stub):
pc = [falcon.Constraint('c1', -inf, 0); falcon.Constraint('c2', -inf, 0)];
phase.addNewPathConstraint(@source_path, pc, tau);
% source_path(states, controls) -> constraints   (or (outputs, states, controls) if outputs used)

% Hand-built, reduced-input version (faster Bake, fewer symbolic derivatives):
pconMdl = falcon.PathConstraintBuilder('Name', outputsSubset, statesSubset, controlsSubset, paramsSubset, @source_path_reduced);
pconMdl.Build();
phase.addNewPathConstraint(@Name, pc, tau);
```

Point constraints (evaluated at one time sample, e.g. touchdown conditions or phase
connections) use the analogous `falcon.PointConstraintBuilder` — see PDF §3.3.6 for the
full phase-connection walkthrough. For simple start/end phase continuity, prefer:

```matlab
problem.ConnectAllPhases();
```

## Reusable cost/constraint building blocks (§5.16, no model-builder needed)

```matlab
falcon.lib.SimpleLinearPathFunction(variables, 'Weight', w, 'Offset', v0)
falcon.lib.SimpleQuadraticPathFunction(variables, 'WeightMatrix', W, 'Offset', v0)
falcon.lib.SimpleLinearPointFunction(variables, ...)
falcon.lib.SimpleQuadraticPointFunction(variables, ...)
falcon.lib.RateLimit(...)         % finite-difference rate limit on states/controls/outputs
```

`variables` can mix States, Controls, Parameters, Outputs in any order. Call
`.evaluate()` on the result and add it as a Lagrange cost / path constraint (path
functions) or Mayer cost / point constraint (point functions).

## Solver setup

```matlab
problem.Bake();                                % prepare (auto-called by Solve if skipped)

solver = falcon.solver.ipopt(problem);
solver.Options.MajorIterLimit = 500;           % max iterations
solver.Options.MajorFeasTol   = 1e-5;          % feasibility tolerance
solver.Options.MajorOptTol    = 1e-5;          % optimality tolerance
solver.Solve();
```

Alternatives (both require separately-licensed/installed tools, see PDF §2.2/2.3):

```matlab
solver = falcon.solver.snopt(problem);         % commercial, needs SNOPT license
solver = falcon.solver.fmincon_algo(problem);  % needs MATLAB Optimization Toolbox, small problems only
```

## Reading results / simulating / plotting

```matlab
t  = phase.RealTime;                 % real (unnormalized) time vector
x  = phase.StateGrid.Values;         % [numStates x numSamples]
u  = phase.ControlGrids.Values;      % [numControls x numSamples]

[states, outputs, simTime, statesDot] = problem.Simulate();   % re-simulate optimized controls open-loop
                                                                % (sanity check on discretization validity)

falcon.gui.plot.show(problem, 'AskSaveOnClose', false);        % built-in GUI plotter (needs working graphics/Java)
```

## Multi-phase quick reference

```matlab
t_mid = falcon.Parameter('MidTime', 20, 0, 40, 0.1);
t_end = falcon.Parameter('EndTime', 40, 0, 80, 0.1);

phase1 = problem.addNewPhase(@dyn, states, tau, 0, t_mid);
phase1.addNewControlGrid(controls, tau);
phase1.setInitialBoundaries(x0);

phase2 = problem.addNewPhase(@dyn, states, tau, t_mid, t_end);
phase2.addNewControlGrid(controls, tau);
phase2.setFinalBoundaries(xf);

problem.ConnectAllPhases();          % links phase1(end) == phase2(start) automatically
```

## Page map into UserGuideMain.pdf

| Topic | Section | Page |
|---|---|---|
| Problem formulation FALCON.m solves | 3.1 | 9 |
| Step-by-step first problem (car) | 3.3.1 | 12 |
| Post-processing steps | 3.3.2 | 16 |
| Path constraints | 3.3.3 | 17 |
| Path constraint builder | 3.3.4 | 19 |
| Multi-phase (ConnectAllPhases) | 3.3.5 | 20 |
| Multi-phase (PointConstraintBuilder) | 3.3.6 | 21 |
| Full aircraft examples (2D/3D) | 3.4 | 22 |
| Collocation theory | 4.2 | 27 |
| falcon.Problem full API | 5.3 | 29 |
| falcon.core.Phase full API | 5.4 | 49 |
| falcon.core.Grid full API | 5.5 | 61 |
| falcon.core.Model full API | 5.6 | 72 |
| falcon.State / Control / Parameter | 5.7-5.9 | 76-93 |
| falcon.Constraint | 5.10 | 101 |
| falcon.solver.ipopt options | 5.15 | 119 |
| Common cost/constraint building blocks | 5.16 | 133 |
| Parameter estimation mode | 6 | 138 |
| Simulation Model Builder (advanced) | 7.3 | 145 |
| Path/Point Constraint Builder (advanced) | 7.4-7.5 | 158-165 |
