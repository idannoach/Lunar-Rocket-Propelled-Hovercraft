function [t_out, x_out] = run_simulation(mission_parameters, hovercraft_parameters)

%% Load Global Parameters
global_parameters = GP();

%% Pack the initial state vector and define Simulation Time Span
x0 = pack_initial_state_vector(mission_parameters, hovercraft_parameters);

if strcmp(mission_parameters.ieSimulationType, 'continuous')
    disp('Starting Continuous Simulation...');
    [t_out, x_out] = run_continuous_simulation(x0, global_parameters, mission_parameters, hovercraft_parameters);
elseif strcmp(mission_parameters.ieSimulationType, 'discrete')
    disp('Starting Discrete Simulation...');
    [t_out, x_out] = run_discrete_simulation(x0, global_parameters, mission_parameters, hovercraft_parameters);
else
    error('No matching simulation type in the config file');
end
disp('Simulation Complete!');

end