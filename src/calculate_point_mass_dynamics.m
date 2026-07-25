function xdot = calculate_point_mass_dynamics(x, T_nwu, GP, HP)
% calculate_point_mass_dynamics - 7-state point-mass derivative
%                                 [pos(3) NWU; vel(3) NWU; mass].
%
% Thrust is commanded directly as a net NWU force vector (no attitude,
% allocation, or per-engine throttling) - this is the same model used by
% opt_control/source_powered_descent.m for the FALCON.m benchmark, so the
% closed-loop point-mass simulation and the open-loop benchmark solve
% literally the same equations of motion.

vel  = x(4:6);
mass = x(7);

g_nwu = [0; 0; -GP.g_lunar]; % NWU: Z positive up, so gravity acts -Z

T_mag = norm(T_nwu);
mdot  = -T_mag / (HP.secIsp * GP.g0);

pos_dot = vel;
vel_dot = T_nwu / mass + g_nwu;

xdot = [pos_dot; vel_dot; mdot];

end
