function [global_parameters, hovercraft_parameters, mission_parameters] = startup()
clc; close all;

%% Load root folder
rootDir = fileparts(mfilename('fullpath'));
addpath(genpath(rootDir));

disp('Hovercraft Simulation paths loaded successfully');

%% Load global parameters
try 
    global_parameters = GP();
catch
    error('Hovercraft parameters did not load successfully');
end

disp('Global parameters loaded successfully');

%% Load hovercraft parameters
try
    hovercraft_parameters = read_from_json('hovercraft.json');
    hovercraft_parameters = calculate_moments_of_inertia_poly(hovercraft_parameters);
    hovercraft_parameters = calculate_allocation_matrices(hovercraft_parameters);
catch
    error('Hovercraft parameters did not load successfully');
end

disp('Hovercraft parameters loaded successfully');

%% Load mission parameters
try
    mission_parameters = read_from_json('mission.json');
catch
    error('Mission parameters did not load successfully');
end

disp('Mission parameters loaded successfully');

end