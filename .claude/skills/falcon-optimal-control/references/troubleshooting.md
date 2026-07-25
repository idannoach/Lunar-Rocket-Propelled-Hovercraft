# Troubleshooting this FALCON.m install

Install details (from `errorReport_20260703T130750.txt` and `StartupCheck.m`):

- FALCON.m release v1.33 (build `v1.33_fa623d1d_public`)
- MATLAB R2026a Update 1
- Symbolic Math Toolbox, MATLAB Coder, Optimization Toolbox, Curve Fitting Toolbox all
  reported installed and licensed
- Compiler: MinGW64 (C and C++)

## Known error already hit on this machine: `'sym' is not recognized as a class`

Full trace (captured 2026-07-03, while solving the `FalconHuntingPrey` example at
`main.m` line 95, inside `myProblem.Solve()`):

```
Build trace entry failed: SYSTEM at ?
Error using falcon.core.builder.CreateGradient
Error defining property 'Assumptions' of class 'wrap.ctx.AssumptionContext'.
'sym' is not recognized as a class. Make sure it is a valid class on the path.
  ...
Error in falcon.core.builder.AnalyticDerivative/addSubsystem_trace
Error in falcon.core.builder.DerivativeBuilder/EvaluateBuildTraceEntry
Error in falcon.core.builder.DerivativeBuilder/EvaluateBuildTrace
Error in falcon.core.builder.DerivativeBuilder/BuildDerivatives
Error in falcon.core.builder.BaseBuilder/Build
Error in falcon.core.Model/CheckConsistency
Error in falcon.core.Phase/CheckConsistency
Error in falcon.Problem/CheckConsistency
Error in falcon.Problem/Bake
Error in falcon.Problem/Solve
```

**What this means**: FALCON.m's derivative builder calls into MATLAB's Symbolic Math
Toolbox (`sym` class) while symbolically differentiating the model. The error means
MATLAB could not resolve the `sym` class on the path at that moment — this is a path/
toolbox-resolution problem, not a bug in the model equations, and it happened on the
*bundled example*, so it is not specific to any user-written model.

**Fixes, in order of likelihood**:

1. **Toolbox shadowing** — another folder earlier on the MATLAB path defines something
   named `sym` (a variable, function, or class) that shadows the Symbolic Math Toolbox's
   `sym` class. Run `which sym -all` in MATLAB and confirm the top hit resolves inside
   `.../toolbox/symbolic/symbolic/@sym`. If a project folder or another toolbox wins,
   reorder the path (`addpath`/`rmpath`) so the Symbolic Math Toolbox isn't shadowed.
2. **Stale MATLAB session state** — this class-resolution error is a classic symptom of
   a MATLAB session where the Symbolic Math Toolbox was loaded, then paths changed, then
   FALCON.m's `+falcon` folder was added later. Restart MATLAB, run `falcon.init()` (or
   `StartupCheck()`) fresh, and retry before assuming anything is broken.
3. **Rerun `StartupCheck()`** — it calls `falcon.init(true)` (forced re-init) and
   `falcon.console.CheckCommand().call()`, which re-validates the toolbox setup and
   surfaces a clearer message if a required toolbox truly isn't licensed/installed.
4. **Confirm licensing, not just installation** — `errorReport` shows Symbolic Math
   Toolbox as both "Installed" and "Licensed" for this install, so a license problem is
   unlikely to be the cause here, but it's worth a quick `license('test','Symbolic_Toolbox')`
   check (should return `1`) if the above don't resolve it.
5. **Clear cached derivative builds** — FALCON.m caches built MEX derivatives
   (`fm_mex_*.mexw64` next to `falcon.m`, and `fm_models`/`fm_constraints` folders next
   to each script). If a build was interrupted mid-way, delete the stale
   `fm_mex_FalconModel.mexw64` / `fm_mex_HeightConstraint.mexw64` (or the equivalent for
   your own model name) and rerun so FALCON.m rebuilds from scratch.

If none of the above resolves it, capture a fresh `errorReport_*.txt` (FALCON.m
generates one automatically next to `falcon.m` on uncaught errors) and compare the
MATLAB/toolbox version block against this one — a MATLAB or toolbox update between runs
is the most common way this class of error appears or disappears.

## General first-run gotchas (from the Quick Start Guide, not this project's error log)

- **"Do I need to pre-create the model/constraint file?"** No — reference it with
  `@my_function` before it exists and FALCON.m offers to generate the correctly-shaped
  stub the first time you run the script (answer `y` in the console). It will "fail" that
  first run (the stub returns nothing meaningful) — that's expected; fill in the body and
  rerun.
- **Consistency errors at `Bake()`** ("parameter X not found on phase") almost always
  mean a `falcon.Parameter` or `falcon.Output` used inside a model/constraint function
  wasn't also registered on the phase/constraint object via `setModelParameters`/
  `setParameters`/`setModelOutputs`. FALCON.m requires symmetric registration — every
  value a function *uses* must be declared where that function is attached.
- **Solver "converges" to an infeasible-looking trajectory** — usually a scaling issue
  (see SKILL.md §Scaling) or too coarse a `tau` grid, not a genuine infeasibility. Try
  tightening `MajorFeasTol`/`MajorOptTol` and increasing sample density before
  concluding the problem is infeasible.
- **Plot GUI (`falcon.gui.plot.show`) throws/does nothing** — this is a known,
  graphics/Java-driver-dependent feature; every example wraps it in `try/catch` for this
  reason. Extract `phase.RealTime` / `phase.StateGrid.Values` / `phase.ControlGrids.Values`
  and plot manually if the GUI doesn't render.
