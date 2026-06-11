function dxdt = calculate_dynamics(t, x, cmd_throttles, GP, HP)
% CORE 6-DOF DYNAMICS FUNCTION (THE PLANT)
%
%% Unpack State Vector
vel   = x(4:6);   % [u; v; w] (Body)
euler = x(7:9);   % [phi; theta; psi] (Euler Angles)
rates = x(10:12); % [p; q; r] (Body rates)
mass  = x(13);    % [m] (Instantaneous mass)

phi = euler(1); theta = euler(2); psi = euler(3);
p = rates(1); q = rates(2); r = rates(3);
u = vel(1); v = vel(2); w = vel(3);
actual_throttles = x(14:19);

%% Calculate 3D Engine Forces & Moments
[Forces_body, Moments_body, mdot_total] = calculate_thrust_forces(actual_throttles, HP, GP);

%% KINEMATICS (Translational & Rotational)
% Get Body-to-NED rotation matrix
R_b2n = calculate_body_to_ned_matrix(phi, theta, psi);

% Calculate position derivative (NED velocities)
pos_dot = R_b2n * vel;

% Calculate Euler angle derivatives (from body rates)
euler_dot = calculate_euler_kinematics(rates, phi, theta);

%% DYNAMICS (Forces & Moments)
% Translational Dynamics
vel_dot = calculate_translational_dynamics(Forces_body, mass, R_b2n, GP, r, p, q, u, v, w);

% Rotational Dynamics (Euler's Equations)
rates_dot = calculate_rotational_dynamics(Moments_body, mass, HP, r, p, q);

% Mass Dynamics
mass_dot = -mdot_total;

% Engine Dynamics
throttles_dot = calculate_engine_dynamics(cmd_throttles, actual_throttles, HP);

%% PACK DERIVATIVE VECTOR
dxdt = [pos_dot; vel_dot; euler_dot; rates_dot; mass_dot; throttles_dot];

end