function [cmd_accel_x, cmd_accel_y, cmd_accel_z] = calculate_guidance_law(t, current_state, GP, mission_parameters)
% calculate_guidance_law - Linear Quadratic (LQ) Optimal Soft Landing Guidance
%
% Computes the energy-optimal inertial acceleration required to drive both
% position error and velocity error to zero, using a dynamically updated t_go.

%% Extract Current Kinematics
pos   = current_state(1:3); % [X; Y; Z] (NED)
vel   = current_state(4:6); % [u; v; w] (Body Frame)
euler = current_state(7:9); % [phi; theta; psi]

% Rotate velocity from Body frame to NED inertial frame
R_b2n = calculate_body_to_ned_matrix(euler(1), euler(2), euler(3));
vel_ned = R_b2n * vel;

%% Target State Errors
% We want to reach the waypoint with exactly zero velocity (Soft Touchdown)
target_pos = mission_parameters.mTargetPosition;
delta_r = target_pos - pos;

%% Time-To-Go (t_go) Estimation
t_final = mission_parameters.sFinalSimulationTime;
t_go = t_final - t;

% Prevent division by zero singularity
if t_go < 0.5
    t_go = 0.5;
end

%% LQ Optimal Control Law (Soft Landing)
% This minimizes the integral of acceleration squared while satisfying
% the boundary conditions of delta_r = 0 and vel_ned = 0.
accel_kinematic_ned = (6 / t_go^2) * delta_r - (4 / t_go) * vel_ned;

%% Compensate for Lunar Gravity
% The engines must output this kinematic acceleration PLUS hold the vehicle
% up against the constant downward pull of lunar gravity.
g_ned = [0; 0; GP.g_lunar];
accel_thrust_ned = accel_kinematic_ned - g_ned;

%% Output Commanded Accelerations
cmd_accel_x = accel_thrust_ned(1);
cmd_accel_y = accel_thrust_ned(2);
cmd_accel_z = accel_thrust_ned(3);

end