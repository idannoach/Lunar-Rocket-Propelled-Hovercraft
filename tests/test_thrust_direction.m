% visualize_hovercraft_engines.m
clear; clc; close all;

% --- Vehicle Parameters ---
r = 0.6;           % Engine radial distance [m]
z_eng = 0.25;      % Engine height along Z-axis [m]
cg_z = 0.22;       % Center of Gravity height [m] (from your table)
tilt_angle = deg2rad(15);

% --- Setup Figure ---
figure('Name', 'Hovercraft Thrust Vectors', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal;
view(3); % Set to 3D isometric view

% --- Plot Reference Geometry ---
% Plot Center of Gravity (C.G)
plot3(0, 0, cg_z, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'C.G (z=0.22m)');

% Plot a dashed circle to represent the hovercraft frame at engine height
theta_circle = linspace(0, 2*pi, 100);
plot3(r*cos(theta_circle), r*sin(theta_circle), z_eng*ones(1,100), ...
    'k--', 'LineWidth', 1, 'HandleVisibility', 'off');

% --- Plot Engines and Thrust Vectors ---
v_scale = 0.3; % Visual scaling factor for the quiver arrows

for i = 1:6
    % Calculate position for engine i
    theta_rad = deg2rad((i-1) * 60);
    pos = [r * cos(theta_rad), r * sin(theta_rad), z_eng];

    % Get thrust direction
    dx = -sin(tilt_angle) * cos(theta_rad);
    dy = -sin(tilt_angle) * sin(theta_rad);
    dz = cos(tilt_angle);

    %% Create the column vector
    t_dir = [dx; dy; dz];

    % Plot engine location
    if i == 1
        scatter3(pos(1), pos(2), pos(3), 70, 'filled', 'MarkerFaceColor', 'b', 'DisplayName', 'Engines');
    else
        scatter3(pos(1), pos(2), pos(3), 70, 'filled', 'MarkerFaceColor', 'b', 'HandleVisibility', 'off');
    end

    % Plot thrust vector using quiver3
    q = quiver3(pos(1), pos(2), pos(3), ...
        t_dir(1)*v_scale, t_dir(2)*v_scale, t_dir(3)*v_scale, ...
        'off', 'LineWidth', 2, 'Color', 'm', 'MaxHeadSize', 0.5);

    % Add text label next to each engine
    text(pos(1)*1.2, pos(2)*1.2, pos(3), sprintf('E%d', i), 'FontSize', 10, 'FontWeight', 'bold');

    if i == 1
        q.DisplayName = 'Thrust Vector (15^\circ Inward)';
    else
        q.HandleVisibility = 'off'; % Prevent cluttering the legend
    end
end

% --- Formatting ---
xlabel('X [m] (Forward)', 'FontWeight', 'bold');
ylabel('Y [m] (Right)', 'FontWeight', 'bold');
zlabel('Z [m] (Up)', 'FontWeight', 'bold');
title('6-DOF Lunar Hovercraft: Engine Positions & Thrust Vectors', 'FontSize', 14);
legend('Location', 'northeast', 'FontSize', 11);

% Set axes limits to frame the vehicle nicely
xlim([-0.8 0.8]); 
ylim([-0.8 0.8]); 
zlim([0 0.7]);
set(gca, 'CameraPosition', [3.5, 3.5, 2.5]); % Better default camera angle
hold off;