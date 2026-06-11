function dxdt = calculate_dynamics(~, x, cmd_throttles, GP, HP)
% CORE 6-DOF DYNAMICS FUNCTION (THE PLANT)
% Used by the continuous simulation via ode15s.

actual_throttles = x(14:19);

dxdt_rb       = calculate_rigid_body_dynamics(x(1:13), actual_throttles, GP, HP);
throttles_dot = calculate_engine_dynamics(cmd_throttles, actual_throttles, HP);

dxdt = [dxdt_rb; throttles_dot];

end