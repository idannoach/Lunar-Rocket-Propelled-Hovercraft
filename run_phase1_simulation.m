function [time_vector, x_history, engine_on_timers, command_buffer, u_history] = run_phase1_simulation(global_parameters, mission_parameters, hovercraft_parameters, is6DOF)
% u_history: 6-column throttle command history if is6DOF, 3-column NWU
% thrust-vector history otherwise (see control_module_point_mass.m) - used
% by main.m to build the result struct utils/report_point_mass_comparison.m
% expects for the point-mass benchmark missions.

%% Pack the initial state vector
if is6DOF
    x0 = pack_initial_state_vector(mission_parameters, hovercraft_parameters);
else
    x0 = pack_initial_state_vector_point_mass(mission_parameters, hovercraft_parameters);
end

%% Discrete Flight Computer Setup
dt = 1 / mission_parameters.hzControllerRate;
t_end = mission_parameters.sFinalSimulationTime;
time_vector = 0:dt:t_end;
N_steps = length(time_vector);

%% Storage arrays
x_history = zeros(N_steps, length(x0));
x_history(1, :) = x0';

% Engine State Trackers for PWM/MIB (6-DOF actuator chain only)
engine_on_timers = zeros(6,1);   % Tracks how long an engine has been ON
command_buffer = zeros(6, 1);    % Simple 1-step buffer for the 10ms delay

% Fuel exhaustion parameters
fuel_exhausted = false;

% Control/thrust history
u_history = zeros(N_steps, 3 + 3*is6DOF); % 3 cols (NWU thrust) or 6 (throttles)

%% Simulation Loop
for k = 1:N_steps-1
    t_curr = time_vector(k);

    % --- Navigation ---
    x0 = navigation_module(t_curr, x0);

    % --- Guidance ---
    [cmd_n, cmd_w, cmd_u] = guidance_module(t_curr, x0, global_parameters, mission_parameters, is6DOF);

    if is6DOF
        % --- Control ---
        [throttles, engine_on_timers, command_buffer, fuel_exhausted] = control_module(t_curr, dt, x0, cmd_n, cmd_w, cmd_u, engine_on_timers, command_buffer, fuel_exhausted, hovercraft_parameters);
        u_history(k, :) = throttles';

        % --- Plant ---
        x0 = plant_module(dt, x0, throttles, global_parameters, hovercraft_parameters);
    else
        % --- Control ---
        [T_nwu, fuel_exhausted] = control_module_point_mass(t_curr, x0, cmd_n, cmd_w, cmd_u, hovercraft_parameters, fuel_exhausted);
        u_history(k, :) = T_nwu';

        % --- Plant ---
        x0 = plant_module_point_mass(dt, x0, T_nwu, global_parameters, hovercraft_parameters);
    end

    % --- Save history ---
    x_history(k+1, :) = x0';

    % Touchdown/arrival check
    isTouchDown = x0(3) <= mission_parameters.mTargetPosition(3);
    if isTouchDown
        x_history = x_history(1:k+1, :);
        time_vector = time_vector(1:k+1);
        u_history = u_history(1:k+1, :);
        u_history(end, :) = u_history(end-1, :); % no control computed past touchdown
        disp('Touchdown detected.');
        break;
    end
end
if ~isTouchDown
    u_history(end, :) = u_history(end-1, :); % no control computed for the final sample
end
disp('Simulation complete.');

end