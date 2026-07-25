 Main script: runs 6 missions in 3 comparison pairs, each pair getting its
 own visualization.

   Missions 1-2: the basic closed-form LQ/ZEM-ZEV guidance law (no
                 intermediate point) vs. a FALCON.m minimum-fuel
                 benchmark, both solved point-mass for the Nataf &
                 Shaferman (2024) Sec. V.B "Sample run" scenario
                 (opt_control/essay_mission_parameters.m).
   Missions 3-4: the soft-constrained LQ law WITH an optimally-selected
                 intermediate point vs. FALCON.m, recreating the same
                 paper's Sec. V.B "with an intermediate point" case.
   Missions 5-6: this project's own final mission (config/mission.json,
                 documented in paper.tex's "Mission Configuration"
                 appendix), run as the full 6-DOF closed-loop LQ/ZEM-ZEV
                 guidance simulation (t_f-converged) vs. FALCON.m, at the
                 vehicle's real thrust budget from config/hovercraft.json
                 (unmodified - see below). This is paper.tex's Phase 1
                 (Crater Descent).
   Phase 3:      immediately follows Missions 5-6 - the Return Ascent leg
                 (paper.tex Sec. "Mission Formulation"): the SAME mission
                 and guidance law with the initial/target boundary
                 conditions swapped, carrying over the fuel/mass
                 remaining after Phase 1 rather than relaunching with a
                 full tank. Reported as its own LQ-vs-FALCON.m comparison
                 plus a combined round-trip (Phase 1 + Phase 3) summary
                 matching paper.tex's Results table.

 Missions 1-4 are point-mass (see opt_control/main_lq_analytic_descent.m,
 opt_control/main_falcon_descent.m) and are visualized with
 opt_control/report_point_mass_comparison.m. Missions 5-6 run the full
 6-DOF vehicle plant (see solve_lq_guidance_mission.m) and are visualized
 with utils/visualization.m, matching the rest of the project.

 THRUST BUDGET, MISSIONS 1-4 ONLY: at the vehicle's real thrust budget
 (config/hovercraft.json: 6 x 20 N = 120 N total), FALCON.m's minimum-FUEL
 problem (hard thrust bound) does not converge for the essay's own
 scenario - net maneuvering authority is only ~0.375 m/s^2 over a ~150 km
 descent with a ~1 km/s initial closing speed, which is a fundamentally
 harder problem than the paper's own soft-constrained minimum-CONTROL-
 EFFORT law (gnc/calculate_axis_control.m, which has no thrust bound and
 is exactly solvable in closed form) - see
 opt_control/main_falcon_descent.m's header for the empirical
 investigation. Missions 1-4 therefore use a separate hovercraft_parameters
 copy with a 2x-boosted thrust budget (verified empirically to converge
 directly, cleanly, at a realistic flight time/fuel use) so FALCON.m's
 benchmark is a genuine converged optimum rather than a flagged
 non-convergent result. This ONLY affects missions 1-4's point-mass
 benchmark vehicle - missions 5-6 use hovercraft.json's real thrust
 budget, completely unmodified.