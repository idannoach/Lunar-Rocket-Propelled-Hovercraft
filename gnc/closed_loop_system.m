function dxdt = closed_loop_system(t, x, GP, hovercraft_parameters, mission_parameters)
% Runs the GNC algorithms to get throttles, then runs the physics.

%% NAVIGATION
% In a perfect simulation, Navigation simply passes the exact state.
current_state = x;

%% GUIDANCE (Outer Loop)
[cmd_accel_x, ~, cmd_accel_z] = calculate_guidance_law(t, current_state, GP, mission_parameters);

%% ATTITUDE CONTROL (Inner Loop)
[desired_Fz, desired_My] = run_attitude_controller(current_state, cmd_accel_x, cmd_accel_z, hovercraft_parameters);

%% CONTROL ALLOCATION
% Convert the abstract Fz and My into 6 physical throttle commands
% using the 4-channel symmetric mixer.
throttles = allocate_controls(desired_Fz, desired_My, hovercraft_parameters.B_pinv);

%% THE PLANT (Physics Dynamics)
% Pass the physical throttles into the 6-DOF physics plant
dxdt = calculate_dynamics(t, x, throttles, GP, hovercraft_parameters);

end