function cmd_accel_nwu = optimal_LQ_guidance_with_intermediate_point(t, current_state, global_parameters, mission_parameters, is6DOF)
% calculate_guidance_law - Optimal LQ Powered Descent Guidance With an
%                           Intermediate Point.
%
% Implements the closed-form guidance law from Nataf & Shaferman (2024),
% "Optimal Linear Quadratic Powered Descent With An Optimally Selected
% Intermediate Point," AIAA SciTech 2024. Softly weights the terminal
% position miss, terminal velocity error, and (optionally) the miss at an
% intermediate waypoint, while minimizing the integrated control effort.
% Reduces automatically to the classical terminal-only soft-constrained
% law when the intermediate-point weights are zero (see
% gnc/calculate_axis_control.m).
%
% The paper's frame has its origin AT THE TARGET with Z positive DOWN.
% This project uses NWU (origin at mission start, Z positive UP), so all
% quantities are converted to the paper's target-relative, Z-down frame
% before applying the law, and the resulting command is converted back.
% See utils/flip_z_axis.m.

%% Extract Current Kinematics
pos   = current_state(1:3); % [X; Y; Z] (NWU)
vel   = current_state(4:6); % [u; v; w] (Body Frame)

if is6DOF
    % Rotate velocity from Body frame to NWU inertial frame
    euler = current_state(7:9); % [phi; theta; psi]
    R_b2n = calculate_body_to_nwu_matrix(euler(1), euler(2), euler(3));
    vel_nwu = R_b2n * vel;
else
    vel_nwu = vel;
end

%% Convert to the paper's target-relative, Z-down frame
target_nwu = mission_parameters.mTargetPosition(:);

r = flip_z_axis(pos - target_nwu); % position relative to target, Z-down
V = flip_z_axis(vel_nwu);          % velocity, Z-down
g = flip_z_axis([0; 0; -global_parameters.g_lunar]); % gravity, Z-down: g = [0;0;+g_lunar]

%% Time-to-go to the final time
tf = mission_parameters.sFinalSimulationTime;
tgo_f = max(tf - t, 0);

%% Intermediate point (optional)
use_intermediate = mission_parameters.bUseIntermediatePoint;

if use_intermediate
    t1 = mission_parameters.sIntermediatePointTime;
    rc = flip_z_axis(mission_parameters.mIntermediatePointPosition(:) - target_nwu);
    gamma_w = mission_parameters.gammaIntermediatePositionWeights(:);
else
    % No intermediate point: gamma=0 makes calculate_axis_control's
    % intermediate-point term vanish identically, regardless of t1/rc.
    t1 = tf;
    rc = zeros(3, 1);
    gamma_w = zeros(3, 1);
end

tgo_1 = max(t1 - t, 0);

%% Per-axis weights (Eq 7-8: M_frr, M_fVV, M_1rr are diagonal)
alpha_w = mission_parameters.alphaFinalPositionWeights(:);
beta_w  = mission_parameters.betaFinalVelocityWeights(:);

%% Solve each axis independently (Eq 41-49) and assemble the command
u = zeros(3, 1);
for i = 1:3
    Zbar_fr = r(i) + tgo_f * V(i) + 0.5 * g(i) * tgo_f^2; % Zero-Effort-Miss (Eq 48a)
    Zbar_fV = V(i) + g(i) * tgo_f;                          % Zero-Effort-Velocity (Eq 48b)
    Zbar_1  = r(i) + tgo_1 * V(i) + 0.5 * g(i) * tgo_1^2;   % Zero-Effort-Intermediate-Miss (Eq 48c)
    DeltaZbar_1 = Zbar_1 - rc(i);                            % Eq 24

    u(i) = calculate_axis_control(tgo_f, tgo_1, tf, t1, ...
        alpha_w(i), beta_w(i), gamma_w(i), Zbar_fr, Zbar_fV, DeltaZbar_1);
end

%% Convert the commanded thrust acceleration back to NWU
% u is already the complete thrust-acceleration command (gravity drift is
% already accounted for inside the Zbar terms) - no separate gravity
% compensation step is needed here, unlike the previous implementation.
cmd_accel_nwu = flip_z_axis(u);

end