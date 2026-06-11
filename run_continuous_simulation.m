function [t_out, x_out] = run_continuous_simulation(x0, global_parameters, mission_parameters, hovercraft_parameters)
% Run continuous control simulation using ode15s

tspan = [0, mission_parameters.sFinalSimulationTime];

% Extract the target Z depth
target_Z = mission_parameters.mTargetPosition(3);

% Configure ODE options to include the event
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-6, ...
                 'Events', @(t, x) touchdown_event(t, x, target_Z));

% Call ode45
[t_out, x_out] = ode15s(@(t, x) closed_loop_system(t, x, global_parameters, hovercraft_parameters, mission_parameters), tspan, x0, options);

end