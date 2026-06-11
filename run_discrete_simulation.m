function [time_vector, x_history] = run_discrete_simulation(x_curr, global_parameters, mission_parameters, hovercraft_parameters)
%% Discrete Flight Computer Setup

dt = 1 / mission_parameters.hzControllerRate; % controller period [s] (e.g. 0.02 s = 50 Hz)
t_end = mission_parameters.sFinalSimulationTime;
time_vector = 0:dt:t_end;
N_steps = length(time_vector);

% Storage arrays
x_history = zeros(N_steps, length(x_curr));
x_history(1, :) = x_curr';

%% Engine State Trackers for PWM/MIB
engine_on_timers = zeros(6,1);   % Tracks how long an engine has been ON
command_buffer = zeros(6, 1);    % Simple 1-step buffer for the 10ms delay

% ODE options are constant — create once outside the loop to avoid
% rebuilding the struct on every one of the iterations.
ode_options = odeset('RelTol', 1e-4, 'AbsTol', 1e-4);

%% The Discrete Simulation Loop
for k = 1:N_steps-1
    t_curr = time_vector(k);
    t_next = time_vector(k+1);

    % --- A. GNC ALGORITHMS (The Brain) ---
    [cmd_x, ~, cmd_z] = calculate_guidance_law(t_curr, x_curr, global_parameters, mission_parameters);
    [desired_Fz, desired_My] = run_attitude_controller(x_curr, cmd_x, cmd_z, hovercraft_parameters);

    % Map desired Fz and My to the 4 symmetric channels
    ideal_throttles = allocate_controls(desired_Fz, desired_My, hovercraft_parameters.B_pinv);

    % --- B. PWM & MIB HARDWARE LOGIC (The Electronics) ---
    [cmd_throttles, engine_on_timers] = calculate_throttles_command(dt, ideal_throttles, engine_on_timers, hovercraft_parameters);

    % 2. Enforce 10ms Transport Delay
    % (Since dt=20ms, a simple 1-step buffer perfectly mimics ~10-20ms delay)
    delayed_throttles = command_buffer;
    command_buffer = cmd_throttles;

    % --- C. THE CONTINUOUS PLANT (The Physics) ---
    % We simulate the physics for ONLY this specific dt (20ms)
    [~, x_step] = ode15s(@(t, x) calculate_dynamics(t, x, delayed_throttles, global_parameters, hovercraft_parameters), ...
        [t_curr, t_next], x_curr, ode_options);

    % Extract final state of this step to use as initial state for next step
    x_curr = x_step(end, :)';
    x_history(k+1, :) = x_curr';

    % Touchdown check
    if x_curr(3) >= mission_parameters.mTargetPosition(3)
        x_history = x_history(1:k+1, :);
        time_vector = time_vector(1:k+1);
        disp('Touchdown detected!');
        break;
    end
end

end