function startup()
clc; close all;

% Get the directory of this script (the root folder)
rootDir = fileparts(mfilename('fullpath'));

% Add the root and all subdirectories to the path
addpath(genpath(rootDir));

disp('Hovercraft Simulation paths loaded successfully!');
end