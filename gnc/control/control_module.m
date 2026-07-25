function [throttles, engine_on_timers, command_buffer, fuel_exhausted] = control_module(t_curr, dt, x0, cmd_n, cmd_w, cmd_u, engine_on_timers, command_buffer, fuel_exhausted, hovercraft_parameters)

% Run attitude PD controller
[desired_Fz, desired_My] = run_attitude_controller(x0, cmd_n, cmd_u, hovercraft_parameters);

% Map desired Fz and My to the 4 symmetric channels
ideal_throttles = allocate_controls(desired_Fz, desired_My, hovercraft_parameters.B_pinv);

% Fuel exhaustion guard: zero all throttle commands once propellant is depleted
m_dry = hovercraft_parameters.kgTotalMass - hovercraft_parameters.kgFuelMass;
if x0(13) <= m_dry
    ideal_throttles = zeros(6, 1);
    if ~fuel_exhausted
        fuel_exhausted = true;
        warning('run_simulation:FuelExhausted', ...
            'Fuel exhausted at t = %.2f s (mass %.3f kg <= dry mass %.3f kg). Engines disabled.', ...
            t_curr, x0(13), m_dry);
    end
end

% PWM & MIB HARDWARE LOGIC
[cmd_throttles, engine_on_timers] = calculate_throttles_command(dt, ideal_throttles, engine_on_timers, hovercraft_parameters);

% Enforce 10ms Transport Delay
% (Since dt=20ms, a simple 1-step buffer perfectly mimics ~10-20ms delay)
throttles = command_buffer;
command_buffer = cmd_throttles;

end