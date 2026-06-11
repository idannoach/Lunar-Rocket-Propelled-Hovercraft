function log_results(t_out, x_out, mission_parameters, log_folder)
% log_results - Generates a timestamped text file containing terminal
%               performance metrics for the simulation run.
%
% Inputs:
%   t_out              - Time history array
%   x_out              - State history matrix
%   mission_parameters - Struct containing the target waypoint
%   log_folder         - (Optional) Directory to save the log. Defaults to current directory.

%% 1. Handle File Paths & Naming
if nargin < 4 || isempty(log_folder)
    log_folder = pwd;
end

% Create the folder if it doesn't exist
if ~exist(log_folder, 'dir')
    mkdir(log_folder);
end

% Generate a unique timestamped filename
timestamp = datestr(now, 'yyyy-mm-dd_HH-MM-SS');
filename = fullfile(log_folder, sprintf('sim_run_%s.log', timestamp));

%% 2. Extract Terminal Data
t_final = t_out(end);
pos_f   = x_out(end, 1:3);
vel_f   = x_out(end, 4:6);
euler_f = rad2deg(x_out(end, 7:9));
rates_f = x_out(end, 10:12);

mass_initial = x_out(1, 13);
mass_final   = x_out(end, 13);
fuel_burned  = mass_initial - mass_final;
impact_speed = norm(vel_f);

%% 3. Calculate Performance Metrics
% Ensure target_pos is a row vector for subtraction
target_pos = mission_parameters.mTargetPosition;
if iscolumn(target_pos)
    target_pos = target_pos';
end

% Euclidean distance between final position and target waypoint
miss_distance = norm(pos_f - target_pos);

%% 4. Write to File
fid = fopen(filename, 'w');
if fid == -1
    error('log_results:CannotOpenFile', 'Could not open log file for writing at %s', filename);
end

% Header
fprintf(fid, '======================================================\n');
fprintf(fid, '          6-DOF HOVERCRAFT SIMULATION LOG\n');
fprintf(fid, '======================================================\n');
fprintf(fid, 'Timestamp:          %s\n\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

% Mass & Time
fprintf(fid, '--- SYSTEM METRICS ---\n');
fprintf(fid, 'Flight Time:        %.3f sec\n', t_final);
fprintf(fid, 'Fuel Burned:        %.3f kg\n', fuel_burned);
fprintf(fid, 'Remaining Mass:     %.3f kg\n\n', mass_final);

% Navigation
fprintf(fid, '--- NAVIGATION PERFORMANCE ---\n');
fprintf(fid, 'Target Position:    [%8.2f, %8.2f, %8.2f] m\n', target_pos(1), target_pos(2), target_pos(3));
fprintf(fid, 'Final Position:     [%8.2f, %8.2f, %8.2f] m\n', pos_f(1), pos_f(2), pos_f(3));
fprintf(fid, 'Miss Distance:      %.3f m\n\n', miss_distance);

% Kinematics
fprintf(fid, '--- TERMINAL KINEMATICS ---\n');
fprintf(fid, 'Impact Speed:       %.3f m/s\n', impact_speed);
fprintf(fid, 'Terminal Velocity:  [%6.3f, %6.3f, %6.3f] m/s (Body u,v,w)\n', vel_f(1), vel_f(2), vel_f(3));
fprintf(fid, 'Terminal Attitude:  Roll: %5.2f°, Pitch: %5.2f°, Yaw: %5.2f°\n', euler_f(1), euler_f(2), euler_f(3));
fprintf(fid, 'Terminal Rates:     [%6.3f, %6.3f, %6.3f] rad/s (Body p,q,r)\n', rates_f(1), rates_f(2), rates_f(3));

% Close the file safely
fclose(fid);

% Notify the user in the command window
fprintf('Run metrics successfully logged to: %s\n', filename);

end