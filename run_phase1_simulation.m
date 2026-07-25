function [time_vector, x_history, engine_on_timers, command_buffer] = run_phase1_simulation(global_parameters, mission_parameters, hovercraft_parameters, is6DOF)

%% Pack the initial state vector
x0 = pack_initial_state_vector(mission_parameters, hovercraft_parameters);

%% Discrete Flight Computer Setup
dt = 1 / mission_parameters.hzControllerRate;
t_end = mission_parameters.sFinalSimulationTime;
time_vector = 0:dt:t_end;
N_steps = length(time_vector);

%% Storage arrays
x_history = zeros(N_steps, length(x0));
x_history(1, :) = x0';

% Engine State Trackers for PWM/MIB
engine_on_timers = zeros(6,1);   % Tracks how long an engine has been ON
command_buffer = zeros(6, 1);    % Simple 1-step buffer for the 10ms delay

% Fuel exhaustion parameters
fuel_exhausted = false;

%% Simulation Loop
for k = 1:N_steps-1
    t_curr = time_vector(k);

    % --- Navigation ---
    x0 = navigation_module(t_curr, x0);

    % --- Guidance ---
    [cmd_n, cmd_w, cmd_u] = guidance_module(t_curr, x0, global_parameters, mission_parameters, is6DOF);
    
    % --- Control ---
    [throttles, engine_on_timers, command_buffer, fuel_exhausted] = control_module(t_curr, dt, x0, cmd_n, cmd_w, cmd_u, engine_on_timers, command_buffer, fuel_exhausted, hovercraft_parameters, is6DOF);

    % --- Plant ---
    x0 = plant_module(dt, x0, throttles, global_parameters, hovercraft_parameters, is6DOF);

    % --- Save history ---
    x_history(k+1, :) = x0';

    % Touchdown/arrival check
    isTouchDown = x0(3) <= mission_parameters.mTargetPosition(3);
    if isTouchDown
        x_history = x_history(1:k+1, :);
        time_vector = time_vector(1:k+1);
        disp('Touchdown detected.');
        break;
    end
end
disp('Simulation complete.');

end