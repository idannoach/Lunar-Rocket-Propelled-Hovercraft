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

% Engine time constant — must match calculate_engine_dynamics.m
tau = (hovercraft_parameters.msecResponseTo90PctThrustTime / 1000) / 2.3;

% 4 RK4 sub-steps per controller period (5 ms each at 50 Hz)
N_sub = 4;
h = dt / N_sub;

%% The Discrete Simulation Loop
for k = 1:N_steps-1
    t_curr = time_vector(k);

    % --- A. GNC ALGORITHMS (The Brain) ---
    [cmd_x, ~, cmd_z] = calculate_guidance_law(t_curr, x_curr, global_parameters, mission_parameters);
    [desired_Fz, desired_My] = run_attitude_controller(x_curr, cmd_x, cmd_z, hovercraft_parameters);

    % Map desired Fz and My to the 4 symmetric channels
    ideal_throttles = allocate_controls(desired_Fz, desired_My, hovercraft_parameters.B_pinv);

    % --- B. PWM & MIB HARDWARE LOGIC (The Electronics) ---
    [cmd_throttles, engine_on_timers] = calculate_throttles_command(dt, ideal_throttles, engine_on_timers, hovercraft_parameters);

    % Enforce 10ms Transport Delay
    % (Since dt=20ms, a simple 1-step buffer perfectly mimics ~10-20ms delay)
    delayed_throttles = command_buffer;
    command_buffer = cmd_throttles;

    % --- C. THE CONTINUOUS PLANT (The Physics) ---
    x_rb             = x_curr(1:13);
    throttles_0      = x_curr(14:19);
    delta_throttles  = throttles_0 - delayed_throttles; % decaying transient

    % RK4 for the 13 rigid-body states.
    % The engine throttle is not a state here — it is evaluated analytically
    % at each stage time using the exact first-order-lag solution, so the
    % stiff engine eigenvalues never enter the integrator.
    for i = 1:N_sub
        t0  = (i-1) * h;
        th0 = delayed_throttles + delta_throttles * exp(-t0        / tau);
        thm = delayed_throttles + delta_throttles * exp(-(t0+h/2)  / tau);
        th1 = delayed_throttles + delta_throttles * exp(-(t0+h)    / tau);

        k1 = calculate_rigid_body_dynamics(x_rb,           th0, global_parameters, hovercraft_parameters);
        k2 = calculate_rigid_body_dynamics(x_rb + h/2*k1,  thm, global_parameters, hovercraft_parameters);
        k3 = calculate_rigid_body_dynamics(x_rb + h/2*k2,  thm, global_parameters, hovercraft_parameters);
        k4 = calculate_rigid_body_dynamics(x_rb + h*k3,    th1, global_parameters, hovercraft_parameters);
        x_rb = x_rb + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    end

    % Exact analytical update for engine lag states over the full dt
    throttles_end = delayed_throttles + delta_throttles * exp(-dt / tau);

    x_curr = [x_rb; throttles_end];
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