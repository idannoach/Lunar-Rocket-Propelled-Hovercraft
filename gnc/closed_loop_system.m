function dxdt = closed_loop_system(t, x, GP, hovercraft_parameters, mission_parameters)
% Runs the GNC algorithms to get throttles, then runs the physics.
%
% KNOWN ARCHITECTURAL LIMITATIONS
%
% 1. East-axis (Y) guidance is discarded: calculate_guidance_law outputs a
%    3-axis NED acceleration command, but cmd_accel_y is dropped here with '~'.
%    run_attitude_controller controls only pitch (X-Z plane), so any mission
%    with a non-zero East target position will accumulate uncorrected Y error.
%    Fixing this requires a roll-control loop and an expanded allocation matrix.
%
% 2. GNC runs at ODE sub-step rate (not ZOH 50 Hz): ode15s calls this function
%    at each internal evaluation (~5-10x per 20 ms window), so guidance and the
%    attitude controller effectively run at infinite rate.  The engine first-order
%    lag therefore integrates against a continuously-updated command rather than a
%    zero-order-hold signal, making the continuous simulation unconservatively
%    optimistic compared to the discrete (hardware-representative) path.

%% NAVIGATION
% In a perfect simulation, Navigation simply passes the exact state.
current_state = x;

%% GUIDANCE (Outer Loop)
% cmd_accel_y (East axis) is intentionally discarded — see limitation #1 above.
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