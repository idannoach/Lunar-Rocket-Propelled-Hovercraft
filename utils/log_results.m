function log_results(t_out, x_out, mission_parameters, log_folder, mission_label, notes)
% log_results - Generates a timestamped text file containing terminal
%               performance metrics for a simulation/mission run.
%
% Inputs:
%   t_out              - Time history array
%   x_out              - State/trajectory history matrix. Either a full
%                         vehicle history (13 or 19 columns: pos(1:3),
%                         BODY-frame vel(4:6), Euler angles(7:9),
%                         rates(10:12), mass(13), [engine throttles(14:19)])
%                         or a point-mass history (exactly 7 columns:
%                         pos(1:3), NWU-frame vel(4:6), mass(7) - see
%                         opt_control/main_lq_analytic_descent.m /
%                         opt_control/main_falcon_descent.m) - detected via
%                         size(x_out, 2) so both this project's closed-loop
%                         6-DOF runs and its point-mass benchmark missions
%                         can be logged with the same function.
%   mission_parameters - Struct containing the target waypoint
%   log_folder         - (Optional) Directory to save the log. Defaults to current directory.
%   mission_label       - (Optional) Short identifier for this run (e.g.
%                         "Mission 1 - LQ (no intermediate point)"),
%                         included in both the filename and the report
%                         header so multiple missions logged in the same
%                         second (see main.m, which now logs all 6
%                         missions per run) don't collide/overwrite each
%                         other's files.
%   notes               - (Optional) Free-text line appended near the top
%                         of the report (e.g. a FALCON.m solver
%                         convergence warning) - see
%                         opt_control/main_falcon_descent.m's
%                         result.converged/.exit_status.

%% 1. Handle File Paths & Naming
if nargin < 4 || isempty(log_folder)
    log_folder = pwd;
end
if nargin < 5
    mission_label = '';
end
if nargin < 6
    notes = '';
end

% Create the folder if it doesn't exist
if ~exist(log_folder, 'dir')
    mkdir(log_folder);
end

% Generate a unique timestamped filename, slugging in the mission label (if
% given) so missions logged within the same second don't overwrite each other.
timestamp = datestr(now, 'HH-MM-SS_dd-mm-yyyy');
if isempty(mission_label)
    filename = fullfile(log_folder, sprintf('sim_run_%s.log', timestamp));
else
    slug = regexprep(lower(mission_label), '[^a-z0-9]+', '_');
    slug = regexprep(slug, '(^_+|_+$)', '');
    filename = fullfile(log_folder, sprintf('sim_run_%s_%s.log', slug, timestamp));
end

%% 2. Extract Terminal Data
is_point_mass = size(x_out, 2) == 7;

t_final = t_out(end);
pos_f   = x_out(end, 1:3);
vel_f   = x_out(end, 4:6); % NWU frame if point-mass, Body frame otherwise

if is_point_mass
    mass_col = 7;
else
    mass_col = 13;
    euler_f = rad2deg(x_out(end, 7:9));
    rates_f = x_out(end, 10:12);
end

mass_initial = x_out(1, mass_col);
mass_final   = x_out(end, mass_col);
fuel_burned  = mass_initial - mass_final;
impact_speed = norm(vel_f);

%% 3. Calculate Performance Metrics
target_pos = mission_parameters.mTargetPosition;
if iscolumn(target_pos)
    target_pos = target_pos';
end

miss_distance = calculate_miss_distance(x_out, target_pos);

has_intermediate_point = isfield(mission_parameters, 'bUseIntermediatePoint') && mission_parameters.bUseIntermediatePoint ...
    && isfield(mission_parameters, 'gammaIntermediatePositionWeights') && any(mission_parameters.gammaIntermediatePositionWeights ~= 0);

%% 4. Write to File
fid = fopen(filename, 'w');
if fid == -1
    error('log_results:CannotOpenFile', 'Could not open log file for writing at %s', filename);
end

% Header
fprintf(fid, '======================================================\n');
if is_point_mass
    fprintf(fid, '          POINT-MASS MISSION LOG\n');
else
    fprintf(fid, '          6-DOF HOVERCRAFT SIMULATION LOG\n');
end
if ~isempty(mission_label)
    fprintf(fid, '          %s\n', mission_label);
end
fprintf(fid, '======================================================\n');
fprintf(fid, 'Timestamp:          %s\n', datestr(now, 'HH:MM:SS dd-mm-yyyy'));
if ~isempty(notes)
    fprintf(fid, 'Notes:              %s\n', notes);
end
fprintf(fid, '\n');

% Mass & Time
fprintf(fid, '--- SYSTEM METRICS ---\n');
fprintf(fid, 'Flight Time:        %.3f sec\n', t_final);
fprintf(fid, 'Fuel Burned:        %.3f kg\n', fuel_burned);
fprintf(fid, 'Remaining Mass:     %.3f kg\n\n', mass_final);

% Navigation
fprintf(fid, '--- NAVIGATION PERFORMANCE ---\n');
fprintf(fid, 'Target Position:    [%8.2f, %8.2f, %8.2f] m\n', target_pos(1), target_pos(2), target_pos(3));
fprintf(fid, 'Final Position:     [%8.2f, %8.2f, %8.2f] m\n', pos_f(1), pos_f(2), pos_f(3));
fprintf(fid, 'Miss Distance:      %.3f m\n', miss_distance);
if has_intermediate_point
    ip = mission_parameters.mIntermediatePointPosition;
    fprintf(fid, 'Intermediate Point: [%8.2f, %8.2f, %8.2f] m at t1 = %.2f s (mission setpoint - not necessarily honored by this specific trajectory)\n\n', ...
        ip(1), ip(2), ip(3), mission_parameters.sIntermediatePointTime);
else
    fprintf(fid, 'Intermediate Point: not used\n\n');
end

% Kinematics
fprintf(fid, '--- TERMINAL KINEMATICS ---\n');
fprintf(fid, 'Impact Speed:       %.3f m/s\n', impact_speed);
if is_point_mass
    fprintf(fid, 'Terminal Velocity:  [%6.3f, %6.3f, %6.3f] m/s (NWU X,Y,Z)\n', vel_f(1), vel_f(2), vel_f(3));
    fprintf(fid, 'Terminal Attitude:  N/A (point-mass model - no attitude/rate states)\n');
else
    fprintf(fid, 'Terminal Velocity:  [%6.3f, %6.3f, %6.3f] m/s (Body u,v,w)\n', vel_f(1), vel_f(2), vel_f(3));
    fprintf(fid, 'Terminal Attitude:  Roll: %5.2f°, Pitch: %5.2f°, Yaw: %5.2f°\n', euler_f(1), euler_f(2), euler_f(3));
    fprintf(fid, 'Terminal Rates:     [%6.3f, %6.3f, %6.3f] rad/s (Body p,q,r)\n', rates_f(1), rates_f(2), rates_f(3));
end

% Close the file safely
fclose(fid);

% Notify the user in the command window
fprintf('Run metrics successfully logged to: %s\n', filename);

end
