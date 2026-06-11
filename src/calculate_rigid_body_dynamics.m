function dxdt_rb = calculate_rigid_body_dynamics(x_rb, actual_throttles, GP, HP)
% 13-state rigid-body derivative (positions, velocities, Euler angles,
% angular rates, mass).  Engine lag states are handled separately.

vel   = x_rb(4:6);
euler = x_rb(7:9);
rates = x_rb(10:12);
mass  = x_rb(13);

phi = euler(1); theta = euler(2); psi = euler(3);
p = rates(1); q = rates(2); r = rates(3);
u = vel(1); v = vel(2); w = vel(3);

[Forces_body, Moments_body, mdot_total] = calculate_thrust_forces(actual_throttles, HP, GP);

R_b2n = calculate_body_to_ned_matrix(phi, theta, psi);

pos_dot   = R_b2n * vel;
euler_dot = calculate_euler_kinematics(rates, phi, theta);
vel_dot   = calculate_translational_dynamics(Forces_body, mass, R_b2n, GP, r, p, q, u, v, w);
rates_dot = calculate_rotational_dynamics(Moments_body, mass, HP, r, p, q);
mass_dot  = -mdot_total;

dxdt_rb = [pos_dot; vel_dot; euler_dot; rates_dot; mass_dot];

end
