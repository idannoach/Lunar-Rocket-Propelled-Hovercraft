function visualization(hovercraft_parameters, t_out, x_out)
% visualization - Generates 3D spatial trajectory, State Variable time
%                 histories, Terminal Performance Metrics, and Actuator states.

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

%% 3. FIGURE 1: 3D Spatial Trajectory & Metrics
figure('Name', 'Hovercraft 3D Trajectory', 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.5 0.6]);

% Plot the continuous path
plot3(x, y, z, 'b-', 'LineWidth', 2);
hold on; grid on;

% Highlight start and end points
plot3(x(1), y(1), z(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start (Rim)');
plot3(x(end), y(end), z(end), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8, 'DisplayName', 'Touchdown');

set(gca, 'ZDir', 'reverse'); % NED Z is Down
xlabel('North (X) [m]'); ylabel('East (Y) [m]'); zlabel('Down (Z) [m]');
title('Hovercraft Descent Trajectory');
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
trans_names = {'x (North) [m]', 'y (East) [m]', 'z (Down) [m]', 'Range from Start [m]', ...
    'u (body-x) [m/s]', 'v (body-y) [m/s]', 'w (body-z) [m/s]', 'Total Velocity [m/s]'};

if ~verLessThan('matlab','9.1')
    tlo1 = tiledlayout(2, 4, 'Padding', 'compact', 'TileSpacing', 'compact');
    for i = 1:8
        nexttile(tlo1);
        plot(t, trans_vars{i}, 'b-', 'LineWidth', 1.2);
        grid on; xlabel('Time [s]'); ylabel(trans_names{i});
        title(trans_names{i}, 'Interpreter', 'none');
    end
else
    for i = 1:8
        subplot(2, 4, i);
        plot(t, trans_vars{i}, 'b-', 'LineWidth', 1.2);
        grid on; xlabel('Time [s]'); ylabel(trans_names{i});
        title(trans_names{i}, 'Interpreter', 'none');
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
    nexttile(tlo2, 8);
    plot(t, rot_vars{7}, 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time [s]'); ylabel(rot_names{7});
    title(rot_names{7}, 'Interpreter', 'tex');

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
    grid on; xlabel('Time [s]'); ylabel(rot_names{7});
    title(rot_names{7}, 'Interpreter', 'tex');
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