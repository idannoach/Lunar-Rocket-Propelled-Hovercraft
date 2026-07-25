function falcon_out = main_falcon_descent(global_parameters, mission_parameters, hovercraft_parameters, tf_margin_bounds)
% main_falcon_descent - Builds and solves a FALCON.m minimum-fuel,
%                        point-mass powered-descent problem for
%                        benchmarking against this project's closed-form
%                        LQ/ZEM-ZEV guidance law (see solve_mission.m).
%
% Model: 7 states [X Y Z Vx Vy Vz m] (NWU), 3 controls [Tx Ty Tz] (net NWU
% thrust vector, unbounded - see source_powered_descent.m). Final time is
% fixed to mission_parameters.sFinalSimulationTime unless tf_margin_bounds
% is given, in which case it becomes a free falcon.Parameter bounded to
% tf_nominal .* tf_margin_bounds - a narrow search band around the
% closed-form law's own converged flight time, rather than a fully free
% final time (which would conflate a fuel-optimality comparison with a
% time-optimality one).
%
% Inputs:
%   global_parameters, mission_parameters, hovercraft_parameters - see
%     startup.m / config/*.json
%   tf_margin_bounds - (Optional) 2-vector [lower_frac, upper_frac]
%     applied to mission_parameters.sFinalSimulationTime. Default [1, 1]
%     (fixed tf).
%
% Output falcon_out: t, x [Nx7 NWU: X Y Z Vx Vy Vz m], u [Nx3 NWU thrust
%   force, N], tf, fuel_used, converged, exit_status - the same shape
%   consumed by utils/report_point_mass_comparison.m and utils/log_results.m.

if nargin < 4 || isempty(tf_margin_bounds)
    tf_margin_bounds = [1, 1];
end

GP = global_parameters;
HP = hovercraft_parameters;
MP = mission_parameters;

falcon.init();

%% States: [X, Y, Z, Vx, Vy, Vz, m], NWU
pos_scale  = 1e-4;
vel_scale  = 1e-1;
mass_scale = 1e-1;

m_dry = HP.kgTotalMass - HP.kgFuelMass;

states = [
    falcon.State('X',  -inf, inf, pos_scale)
    falcon.State('Y',  -inf, inf, pos_scale)
    falcon.State('Z',  -inf, inf, pos_scale)
    falcon.State('Vx', -inf, inf, vel_scale)
    falcon.State('Vy', -inf, inf, vel_scale)
    falcon.State('Vz', -inf, inf, vel_scale)
    falcon.State('m',  m_dry, HP.kgTotalMass, mass_scale)
    ];

%% Controls: net NWU thrust vector, unbounded - matches the closed-form LQ
% law's own idealized control-authority assumption (see
% gnc/guidance/optimal_LQ_guidance_with_intermediate_point.m), so neither
% side of the comparison has an actuator-saturation nonlinearity.
thrust_scale = 1e-1;
controls = [
    falcon.Control('Tx', -inf, inf, thrust_scale)
    falcon.Control('Ty', -inf, inf, thrust_scale)
    falcon.Control('Tz', -inf, inf, thrust_scale)
    ];

%% Final time
tf_nominal = MP.sFinalSimulationTime;
tf = falcon.Parameter('FinalTime', tf_nominal, ...
    tf_nominal * tf_margin_bounds(1), tf_nominal * tf_margin_bounds(2), 1e-2);

%% Problem / phase
problem = falcon.Problem('PoweredDescentPointMass');
N = 101;
% Non-uniform grid, denser near touchdown (tau=1) where the dynamics are
% most nonlinear (thrust ramps to arrest velocity) - see
% .claude/skills/falcon-optimal-control/references/project-integration.md.
tau_uniform = linspace(0, 1, N);
tau = 1 - (1 - tau_uniform).^2;

phase = problem.addNewPhase(@source_powered_descent, states, tau, 0, tf);
phase.addNewControlGrid(controls, tau);

%% Boundary conditions
pos_0  = MP.mInitialPosition(:);
vel_0  = MP.mpsInitialVelocity(:);
mass_0 = HP.kgTotalMass;

phase.setInitialBoundaries([pos_0; vel_0; mass_0]);

target = MP.mTargetPosition(:);

% Final position/velocity are equality constraints (soft-landing target);
% final mass is an inequality [dry_mass, initial_mass] so the solver finds
% the true fuel-optimal remaining mass rather than it being prescribed.
phase.setFinalBoundaries( ...
    [target; 0; 0; 0; m_dry], ...
    [target; 0; 0; 0; HP.kgTotalMass]);

%% Intermediate waypoint (optional)
% gnc/guidance/select_intermediate_point.m only SOFTLY penalizes missing
% this waypoint (weighted by gammaIntermediatePositionWeights - see
% gnc/control/calculate_axis_control.m), but the final target above is
% already a hard equality constraint for FALCON, so for a consistent
% hard-constraint formulation the intermediate point is enforced the same
% way here: a small tolerance box (not exact equality, for solver
% robustness) at the waypoint's normalized time. Without this, FALCON's
% trajectory has no reason to go anywhere near the waypoint at all.
%
% Normalized time uses the NOMINAL tf (t1/tf_nominal): exact when tf is
% fixed (tf_margin_bounds = [1,1], the point-mass missions this matters
% for), and only approximate if tf is later allowed to move (Mission 6),
% since t1 is an absolute time chosen by select_intermediate_point.m
% independent of any FALCON-side tf search.
if MP.bUseIntermediatePoint
    t1 = MP.sIntermediatePointTime;
    tau1 = t1 / tf_nominal;
    ip_pos = MP.mIntermediatePointPosition(:);
    ip_tol = 50; % meters, per axis

    ip_constraints = [
        falcon.Constraint('IP_X', ip_pos(1) - ip_tol, ip_pos(1) + ip_tol)
        falcon.Constraint('IP_Y', ip_pos(2) - ip_tol, ip_pos(2) + ip_tol)
        falcon.Constraint('IP_Z', ip_pos(3) - ip_tol, ip_pos(3) + ip_tol)
        ];

    problem.addNewPointConstraint(@source_intermediate_point, ip_constraints, phase, tau1);
end

%% Initial guess: piecewise-linear position/velocity, linear mass depletion
% Routed through the intermediate waypoint when active, rather than a
% straight line to the target that would start well outside the new
% tolerance box at tau1.
if MP.bUseIntermediatePoint
    before_ip = tau <= tau1;
    frac1 = tau(before_ip) / tau1;
    frac2 = (tau(~before_ip) - tau1) / (1 - tau1);

    pos_guess = zeros(3, N);
    pos_guess(:, before_ip)  = pos_0 + (ip_pos - pos_0) * frac1;
    pos_guess(:, ~before_ip) = ip_pos + (target - ip_pos) * frac2;

    vel_guess = zeros(3, N);
    vel_guess(:, before_ip)  = repmat((ip_pos - pos_0) / t1, 1, nnz(before_ip));
    vel_guess(:, ~before_ip) = repmat((target - ip_pos) / (tf_nominal - t1), 1, nnz(~before_ip));
else
    pos_guess = pos_0 + (target - pos_0) * tau;
    vel_guess = repmat((target - pos_0) / tf_nominal, 1, N);
end
mass_guess = mass_0 - 0.3 * (mass_0 - m_dry) * tau;
phase.StateGrid.setValues(tau, [pos_guess; vel_guess; mass_guess]);

%% Cost: fuel-optimal <=> maximize final mass <=> minimize -m(tf)
problem.addNewLinearPointCost(states(7), phase, 1, 'Weight', -1);

% Minimum-fuel powered-descent problems have a near bang-bang optimal
% thrust profile: the cost above is linear in the controls (curvature
% only enters through the dynamics/constraints), which is a well-known
% cause of IPOPT's quasi-Newton Hessian approximation oscillating near the
% optimum instead of tightening (see
% .claude/skills/falcon-optimal-control/references/troubleshooting.md).
% A small quadratic control-effort regularization gives the NLP strictly
% positive curvature in the control directions without materially
% changing the fuel-optimal trajectory.
reg_weight = 1e-6;
phase.addNewQuadraticPathCost(controls, 'WeightMatrix', reg_weight * eye(3));

%% Solve
problem.Bake();
solver = falcon.solver.ipopt(problem);
solver.Options.MajorIterLimit = 1000;
solver.Options.MajorFeasTol   = 1e-5;
solver.Options.MajorOptTol    = 1e-5;
[~, ~, status] = solver.Solve();

%% Results
t          = phase.RealTime;
x_states   = phase.StateGrid.Values';    % [N x 7]
u_controls = phase.ControlGrids.Values'; % [N x 3]

falcon_out.t           = t;
falcon_out.x           = x_states;
falcon_out.u           = u_controls;
falcon_out.tf          = t(end);
falcon_out.fuel_used   = x_states(1, 7) - x_states(end, 7);
falcon_out.converged   = (status == "successful");
falcon_out.exit_status = char(status);

end
