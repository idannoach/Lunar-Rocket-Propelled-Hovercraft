function main()
% Main script to initialize and run the 6-DOF simulation
startup();

%% Read Hovercraft Parameters
hovercraft_parameters = read_from_json('hovercraft.json');

% Calculate and add moments of inertia poly constants
hovercraft_parameters = calculate_moments_of_inertia_poly(hovercraft_parameters);

% Calculate the Control Allocation Matrix
hovercraft_parameters = calculate_allocation_matrices(hovercraft_parameters);

%% Read Mission Parameters
mission_parameters = read_from_json('mission.json');

%% Run simulation - towards crater
[t_out, x_out] = run_simulation(mission_parameters, hovercraft_parameters);

%% Run simulation - back from crater
% mission_parameters.mTargetPosition = [0, 0, 0];
% [t_out, x_out] = run_simulation(mission_parameters, hovercraft_parameters);

%% Log
log_results(t_out, x_out, mission_parameters, 'logs');

%% Visualize
visualization(hovercraft_parameters, t_out, x_out);

end

