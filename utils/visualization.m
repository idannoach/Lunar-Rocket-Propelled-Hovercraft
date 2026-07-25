function visualization(hovercraft_parameters, mission_parameters, t_out, x_out, falcon_result, lq_analytic_result, phase_label)
% visualization - Generates 3D spatial trajectory, State Variable time
%                 histories, Terminal Performance Metrics, and Actuator states.
%
%   visualization(hovercraft_parameters, mission_parameters, t_out, x_out, falcon_result, lq_analytic_result, phase_label)
%   optionally overlays up to two point-mass benchmark trajectories on the
%   3D trajectory, terminal metrics, and the position/speed/mass telemetry
%   panels:
%     falcon_result       - FALCON.m minimum-fuel benchmark (see
%                            opt_control/main_falcon_descent.m)
%     lq_analytic_result  - exact closed-form solution to the paper's own
%                            LQ optimal control problem (see
%                            opt_control/main_lq_analytic_descent.m)
%   Both are [X Y Z Vx Vy Vz m] trajectories (NWU inertial frame), so
%   neither can be overlaid on the body-frame u/v/w velocity components or
%   the attitude/actuator panels.
%
%   phase_label - (optional, default 'Descent') short phase name used in
%   the 3D plot title and start/end markers - e.g. 'Return Ascent' for
%   main.m's Phase 3 leg (paper.tex), where the vehicle starts at the
%   crater floor and arrives back at the rim, not the reverse.

if nargin < 7 || isempty(phase_label)
    phase_label = 'Descent';
end
is_return_leg = strcmpi(phase_label, 'Return Ascent');

has_falcon = nargin >= 5 && ~isempty(falcon_result);
has_lq_analytic = nargin >= 6 && ~isempty(lq_analytic_result);

% A FALCON.m result with converged==false (see
% opt_control/main_falcon_descent.m) is IPOPT's last iterate, not a valid
% minimum-fuel solution - it's still plotted (for transparency), but
% labeled/styled as unreliable rather than presented as a real benchmark.
falcon_converged = has_falcon && (~isfield(falcon_result, 'converged') || falcon_result.converged);
if has_falcon && ~falcon_converged
    fprintf('WARNING: FALCON.m Min-Fuel benchmark did NOT converge (exit status: %s) - treat this overlay as unreliable, not a valid benchmark.\n', ...
        falcon_result.exit_status);
    falcon_label       = 'FALCON.m Min-Fuel Optimal [NOT CONVERGED]';
    falcon_label_short = 'FALCON.m Min-Fuel [NOT CONVERGED]';
    falcon_style       = {':', 'Color', [0.85 0 0]};
else
    falcon_label       = 'FALCON.m Min-Fuel Optimal';
    falcon_label_short = 'FALCON.m Min-Fuel';
    falcon_style       = {'m--'};
end

%% 1. Extract parameters
t = t_out;
pos = x_out(:, 1:3);
x = pos(:, 1); y = pos(:, 2); z = pos(:, 3);

vel = x_out(:, 4:6);
u = vel(:, 1); v = vel(:, 2); w = vel(:, 3);

euler = rad2deg(x_out(:, 7:9));
phi = euler(:, 1); theta = euler(:, 2); psi = euler(:, 3);

rates = rad2deg(x_out(:, 10:12));
p = rates(:, 1); q = rates(:, 2); r = rates(:, 3);

mass = x_out(:, 13);

% Check if Actuator Dynamics (19 states) are included
has_actuators = size(x_out, 2) >= 19;
if has_actuators
    throttles = x_out(:, 14:19);
end

% Derived Metrics
velocity = sqrt(u.^2 + v.^2 + w.^2);
range_from_start = sqrt( (x - x(1)).^2 + (y - y(1)).^2 + (z - z(1)).^2 );

if has_falcon
    t_f = falcon_result.t;
    pos_f_traj = falcon_result.x(:, 1:3);
    xf = pos_f_traj(:, 1); yf = pos_f_traj(:, 2); zf = pos_f_traj(:, 3);
    mass_f = falcon_result.x(:, 7);
    velocity_f = sqrt(sum(falcon_result.x(:, 4:6).^2, 2));
    range_from_start_f = sqrt( (xf - xf(1)).^2 + (yf - yf(1)).^2 + (zf - zf(1)).^2 );
    miss_distance_f = norm(pos_f_traj(end, :) - mission_parameters.mTargetPosition(:)');
end

if has_lq_analytic
    t_a = lq_analytic_result.t;
    pos_a_traj = lq_analytic_result.x(:, 1:3);
    xa = pos_a_traj(:, 1); ya = pos_a_traj(:, 2); za = pos_a_traj(:, 3);
    mass_a = lq_analytic_result.x(:, 7);
    velocity_a = sqrt(sum(lq_analytic_result.x(:, 4:6).^2, 2));
    range_from_start_a = sqrt( (xa - xa(1)).^2 + (ya - ya(1)).^2 + (za - za(1)).^2 );
    miss_distance_a = norm(pos_a_traj(end, :) - mission_parameters.mTargetPosition(:)');
end

%% 2. Calculate Terminal Performance Metrics
t_final = t(end);
pos_f = pos(end, :);
vel_f = vel(end, :);

% Convert terminal attitude to degrees for intuitive reading
euler_f_deg = euler(end, :);

% Magnitudes
impact_velocity = norm(vel_f);
fuel_burned = mass(1) - mass(end);

% Format the metrics text
metrics_text = {
    '\bf--- TERMINAL METRICS ---',
    sprintf('Flight Time: \\rm%.1f sec', t_final),
    sprintf('\\bfFinal Position (X, Y, Z): \\rm[%.1f, %.1f, %.1f] m', pos_f(1), pos_f(2), pos_f(3)),
    sprintf('\\bfImpact Velocity: \\rm%.2f m/s', impact_velocity),
    sprintf('\\bfTerminal Attitude (R, P, Y): \\rm[%.1f^\\circ, %.1f^\\circ, %.1f^\\circ]', euler_f_deg(1), euler_f_deg(2), euler_f_deg(3)),
    sprintf('\\bfFuel Burned: \\rm%.2f kg', fuel_burned)
    };

if has_falcon
    if falcon_converged
        falcon_header = '\bf--- FALCON.m MIN-FUEL BENCHMARK ---';
    else
        falcon_header = '\bf--- FALCON.m MIN-FUEL BENCHMARK (NOT CONVERGED - UNRELIABLE) ---';
    end
    metrics_text = [metrics_text; {
        falcon_header,
        sprintf('Flight Time: \\rm%.1f sec', t_f(end)),
        sprintf('\\bfFuel Used: \\rm%.2f kg', falcon_result.fuel_used),
        sprintf('\\bfMiss Distance: \\rm%.2f m', miss_distance_f)
        }];
end

if has_lq_analytic
    metrics_text = [metrics_text; {
        '\bf--- LQ-OPTIMAL (ANALYTIC) BENCHMARK ---',
        sprintf('Flight Time: \\rm%.1f sec', t_a(end)),
        sprintf('\\bfFuel Used: \\rm%.2f kg', lq_analytic_result.fuel_used),
        sprintf('\\bfMiss Distance: \\rm%.2f m', miss_distance_a)
        }];
end

%% 3. FIGURE 1: 3D Spatial Trajectory & Metrics
figure('Name', 'Hovercraft 3D Trajectory', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.5 0.6]);

% Plot the continuous path
plot3(x, y, z, 'b-', 'LineWidth', 2, 'DisplayName', 'LQ/ZEM-ZEV Guidance');
hold on; grid on;

% Highlight start and end points (labeled per phase - the Return Ascent
% leg starts at the crater floor and arrives back at the rim, not the
% reverse - see phase_label above)
if is_return_leg
    end_label = 'Arrival (Rim)';
else
    end_label = 'Touchdown';
end
plot3(x(1), y(1), z(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start');
plot3(x(end), y(end), z(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', end_label);

% Overlay the FALCON.m minimum-fuel benchmark trajectory, if provided
if has_falcon
    plot3(xf, yf, zf, falcon_style{:}, 'LineWidth', 2, 'DisplayName', falcon_label);
    if falcon_converged
        plot3(xf(end), yf(end), zf(end), 'ms', 'MarkerFaceColor', 'm', 'MarkerSize', 8, 'DisplayName', 'FALCON.m Touchdown');
    end
end

% Overlay the exact closed-form LQ-optimal (Nataf & Shaferman) benchmark, if provided
if has_lq_analytic
    orange = [0.8500 0.3250 0.0980];
    plot3(xa, ya, za, '-.', 'Color', orange, 'LineWidth', 2, 'DisplayName', 'LQ-Optimal (Analytic)');
    plot3(xa(end), ya(end), za(end), 'd', 'Color', orange, 'MarkerFaceColor', orange, 'MarkerSize', 8, 'DisplayName', 'LQ-Optimal Touchdown');
end

% Highlight the intermediate waypoint, if one was used (Sec IV.B guidance law)
has_intermediate_point = isfield(mission_parameters, 'bUseIntermediatePoint') && mission_parameters.bUseIntermediatePoint ...
    && isfield(mission_parameters, 'gammaIntermediatePositionWeights') && any(mission_parameters.gammaIntermediatePositionWeights ~= 0);
if has_intermediate_point
    ip = mission_parameters.mIntermediatePointPosition;
    plot3(ip(1), ip(2), ip(3), 'ys', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'DisplayName', 'Intermediate Point');
end

% Draw the approach cone (Sec IV.B): apex at the target, widening to
% radius mApproachConeRadius at height mApproachConeHeight above it for a
% descent (NWU Up) - or BELOW it for main.m's Return Ascent leg
% (gnc/select_intermediate_point.m mirrors the cone to the vehicle's
% approach side, since it arrives at the target from below, not above).
% Rather than re-deriving that direction here, just extend the cone
% toward whichever side the actual computed waypoint landed on.
has_approach_cone = isfield(mission_parameters, 'bUseIntermediatePoint') && mission_parameters.bUseIntermediatePoint ...
    && isfield(mission_parameters, 'mApproachConeRadius') && isfield(mission_parameters, 'mApproachConeHeight');
if has_approach_cone
    target = mission_parameters.mTargetPosition(:)';
    Rmax = mission_parameters.mApproachConeRadius;
    h = mission_parameters.mApproachConeHeight;

    cone_z_sign = sign(mission_parameters.mIntermediatePointPosition(3) - target(3));
    if cone_z_sign == 0
        cone_z_sign = 1;
    end

    theta_cone = linspace(0, 2*pi, 40);
    z_local = [0; cone_z_sign * h];
    r_local = [0; Rmax];

    [Theta, Zl] = meshgrid(theta_cone, z_local);
    R = repmat(r_local, 1, numel(theta_cone));

    Xc = target(1) + R .* cos(Theta);
    Yc = target(2) + R .* sin(Theta);
    Zc = target(3) + Zl;

    surf(Xc, Yc, Zc, 'FaceColor', 'c', 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'DisplayName', 'Approach Cone');
end

xlabel('North (X) [m]'); ylabel('West (Y) [m]'); zlabel('Up (Z) [m]');
title(sprintf('Hovercraft %s Trajectory', phase_label));
legend('Location', 'northeast');
view(3);

% Overlay the Terminal Metrics Textbox
annotation('textbox', [0.02, 0.75, 0.35, 0.2], ...
    'String', metrics_text, ...
    'EdgeColor', 'k', ...
    'LineWidth', 1, ...
    'BackgroundColor', [0.95 0.95 0.95], ...
    'FaceAlpha', 0.8, ...
    'FitBoxToText', 'on', ...
    'Interpreter', 'tex', ...
    'FontSize', 10);

%% 4. FIGURE 2: Translational Telemetry (Position & Velocity)
figure('Name', 'Translational Telemetry', 'Color', 'w', 'Units', 'normalized', 'Position', [0.15 0.15 0.7 0.6]);

trans_vars = {x, y, z, range_from_start, u, v, w, velocity};
trans_names = {'x (North) [m]', 'y (West) [m]', 'z (Up) [m]', 'Range from Start [m]', ...
    'u (body-x) [m/s]', 'v (body-y) [m/s]', 'w (body-z) [m/s]', 'Total Velocity [m/s]'};

% Both benchmarks' point-mass states are NWU-inertial-frame, so they only
% overlay onto the frame-invariant panels (position, range, total speed),
% not the body-frame u/v/w velocity components (indices 5-7).
if has_falcon
    trans_vars_falcon = {xf, yf, zf, range_from_start_f, [], [], [], velocity_f};
end
if has_lq_analytic
    trans_vars_lq_analytic = {xa, ya, za, range_from_start_a, [], [], [], velocity_a};
end
orange = [0.8500 0.3250 0.0980];

if ~verLessThan('matlab','9.1')
    tlo1 = tiledlayout(2, 4, 'Padding', 'compact', 'TileSpacing', 'compact');
    for i = 1:8
        ax = nexttile(tlo1);
        plot(ax, t, trans_vars{i}, 'b-', 'LineWidth', 1.2, 'DisplayName', 'LQ/ZEM-ZEV Guidance');
        hold(ax, 'on'); grid on; xlabel('Time [s]'); ylabel(trans_names{i});
        title(trans_names{i}, 'Interpreter', 'none');
        if has_falcon && ~isempty(trans_vars_falcon{i})
            plot(ax, t_f, trans_vars_falcon{i}, falcon_style{:}, 'LineWidth', 1.2, 'DisplayName', falcon_label_short);
        end
        if has_lq_analytic && ~isempty(trans_vars_lq_analytic{i})
            plot(ax, t_a, trans_vars_lq_analytic{i}, '-.', 'Color', orange, 'LineWidth', 1.2, 'DisplayName', 'LQ-Optimal (Analytic)');
        end
        if i == 1 && (has_falcon || has_lq_analytic)
            legend(ax, 'Location', 'best');
        end
    end
else
    for i = 1:8
        subplot(2, 4, i);
        plot(t, trans_vars{i}, 'b-', 'LineWidth', 1.2);
        hold on; grid on; xlabel('Time [s]'); ylabel(trans_names{i});
        title(trans_names{i}, 'Interpreter', 'none');
        if has_falcon && ~isempty(trans_vars_falcon{i})
            plot(t_f, trans_vars_falcon{i}, falcon_style{:}, 'LineWidth', 1.2);
        end
        if has_lq_analytic && ~isempty(trans_vars_lq_analytic{i})
            plot(t_a, trans_vars_lq_analytic{i}, '-.', 'Color', orange, 'LineWidth', 1.2);
        end
    end
end

%% 5. FIGURE 3: Rotational Telemetry & System Mass
figure('Name', 'Rotational Telemetry & System', 'Color', 'w', 'Units', 'normalized', 'Position', [0.2 0.2 0.6 0.7]);

rot_vars = {phi, theta, psi, p, q, r, mass};
rot_names = {'\phi (Roll) [deg]', '\theta (Pitch) [deg]', '\psi (Yaw) [deg]', ...
    'p (Roll Rate) [deg/s]', 'q (Pitch Rate) [deg/s]', 'r (Yaw Rate) [deg/s]', ...
    'System Mass [kg]'};

if ~verLessThan('matlab','9.1')
    tlo2 = tiledlayout(3, 3, 'Padding', 'compact', 'TileSpacing', 'compact');

    % Plot Attitude and Rates (Tiles 1-6)
    for i = 1:6
        nexttile(tlo2);
        plot(t, rot_vars{i}, 'b-', 'LineWidth', 1.2);
        grid on; xlabel('Time [s]'); ylabel(rot_names{i});
        title(rot_names{i}, 'Interpreter', 'tex');
    end

    % Plot Mass spanning the bottom row (Tile 8 conceptually)
    ax_mass = nexttile(tlo2, 8);
    plot(ax_mass, t, rot_vars{7}, 'r-', 'LineWidth', 1.5, 'DisplayName', 'LQ/ZEM-ZEV Guidance');
    hold(ax_mass, 'on'); grid on; xlabel('Time [s]'); ylabel(rot_names{7});
    title(rot_names{7}, 'Interpreter', 'tex');
    if has_falcon
        plot(ax_mass, t_f, mass_f, falcon_style{:}, 'LineWidth', 1.5, 'DisplayName', falcon_label_short);
    end
    if has_lq_analytic
        plot(ax_mass, t_a, mass_a, '-.', 'Color', orange, 'LineWidth', 1.5, 'DisplayName', 'LQ-Optimal (Analytic)');
    end
    if has_falcon || has_lq_analytic
        legend(ax_mass, 'Location', 'best');
    end

else
    % Fallback for older MATLAB versions
    for i = 1:6
        subplot(3, 3, i);
        plot(t, rot_vars{i}, 'b-', 'LineWidth', 1.2);
        grid on; xlabel('Time [s]'); ylabel(rot_names{i});
        title(rot_names{i}, 'Interpreter', 'tex');
    end

    subplot(3, 3, 8);
    plot(t, rot_vars{7}, 'r-', 'LineWidth', 1.5);
    hold on; grid on; xlabel('Time [s]'); ylabel(rot_names{7});
    title(rot_names{7}, 'Interpreter', 'tex');
    if has_falcon
        plot(t_f, mass_f, falcon_style{:}, 'LineWidth', 1.5);
    end
    if has_lq_analytic
        plot(t_a, mass_a, '-.', 'Color', orange, 'LineWidth', 1.5);
    end
end

%% 6. FIGURE 4: Actuator Telemetry (Engine Throttles)
if has_actuators
    figure('Name', 'Actuator Telemetry (Engine Throttles)', 'Color', 'w', 'Units', 'normalized', 'Position', [0.25 0.25 0.6 0.5]);
    
    th_names = {'Engine 1 (Front)', 'Engine 2 (Front-Left)', 'Engine 3 (Back-Left)', ...
                'Engine 4 (Back)', 'Engine 5 (Back-Right)', 'Engine 6 (Front-Right)'};
                
    if ~verLessThan('matlab','9.1')
        tlo3 = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
        for i = 1:6
            nexttile(tlo3);
            % Convert throttle 0.0-1.0 to Percentage 0-100%
            plot(t, throttles(:, i) * 100, 'm-', 'LineWidth', 1.5); 
            hold on; grid on;
            
            % Plot physical hardware limits
            plot([t(1), t(end)], [hovercraft_parameters.pctThrottleRange(1), hovercraft_parameters.pctThrottleRange(1)], 'r--', 'LineWidth', 1);
            plot([t(1), t(end)], [hovercraft_parameters.pctThrottleRange(2), hovercraft_parameters.pctThrottleRange(2)], 'r--', 'LineWidth', 1);
            
            ylim([0 hovercraft_parameters.pctThrottleRange(2)+10]); % Give a little margin above 100% for readability
            xlabel('Time [s]'); ylabel('Throttle [%]');
            title(th_names{i}, 'Interpreter', 'none');
        end
    else
        % Fallback for older MATLAB versions
        for i = 1:6
            subplot(2, 3, i);
            plot(t, throttles(:, i) * 100, 'm-', 'LineWidth', 1.5);
            hold on; grid on;
            
            plot([t(1), t(end)], [hovercraft_parameters.pctThrottleRange(1), hovercraft_parameters.pctThrottleRange(1)], 'r--', 'LineWidth', 1);
            plot([t(1), t(end)], [hovercraft_parameters.pctThrottleRange(2), hovercraft_parameters.pctThrottleRange(2)], 'r--', 'LineWidth', 1);
            
            ylim([0 110]); 
            xlabel('Time [s]'); ylabel('Throttle [%]');
            title(th_names{i}, 'Interpreter', 'none');
        end
    end
end

end