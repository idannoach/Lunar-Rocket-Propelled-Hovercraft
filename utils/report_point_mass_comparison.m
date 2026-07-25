function report_point_mass_comparison(mission_parameters, result_a, label_a, result_b, label_b, fig_name)
% report_point_mass_comparison - Prints a terminal-performance comparison
%   table and plots a 3D-trajectory + telemetry comparison for two
%   point-mass mission results (e.g. this project's closed-form LQ-optimal
%   guidance law vs. a FALCON.m minimum-fuel benchmark, both solved for the
%   same initial/target conditions). Used by main.m for the point-mass
%   mission pairs.
%
%   Unlike utils/visualization.m (built around one primary 6-DOF x_out with
%   up to two point-mass overlays), both trajectories here are pure
%   point-mass histories with no attitude/actuator states, so they get
%   their own, simpler comparison instead of being forced into that format.
%
%   A result with a `converged` field set to false (see
%   opt_control/main_falcon_descent.m - IPOPT can run out of iterations
%   without converging) is NOT a valid trajectory, just the solver's last
%   iterate. Rather than silently plotting/reporting it as if it were a
%   legitimate benchmark, it is labeled "[NOT CONVERGED]" in the legend,
%   terminal table, and metrics box, drawn with a distinct warning style
%   (dotted red), and flagged with a console warning. Results without a
%   `converged` field (e.g. the closed-form LQ-analytic solution, which is
%   exact by construction) are always treated as converged.
%
%   Inputs:
%     mission_parameters   - mission struct (target position, approach
%                             cone/intermediate point, if used)
%     result_a, result_b    - point-mass result structs (see
%                             opt_control/main_lq_analytic_descent.m /
%                             opt_control/main_falcon_descent.m): fields t,
%                             x [Nx7: X Y Z Vx Vy Vz m] NWU, u [Nx3: Tx Ty
%                             Tz thrust force, N], tf, fuel_used, and
%                             optionally converged/exit_status
%     label_a, label_b      - legend labels for the two trajectories
%     fig_name              - figure name/title prefix

target = mission_parameters.mTargetPosition(:)';

miss_a = norm(result_a.x(end, 1:3) - target);
miss_b = norm(result_b.x(end, 1:3) - target);

converged_a = local_is_converged(result_a);
converged_b = local_is_converged(result_b);

label_a_disp = local_decorate_label(label_a, converged_a);
label_b_disp = local_decorate_label(label_b, converged_b);

if ~converged_a
    fprintf('WARNING: "%s" did NOT converge (exit status: %s) - treat this trajectory as unreliable, not a valid benchmark.\n', ...
        label_a, result_a.exit_status);
end
if ~converged_b
    fprintf('WARNING: "%s" did NOT converge (exit status: %s) - treat this trajectory as unreliable, not a valid benchmark.\n', ...
        label_b, result_b.exit_status);
end

%% Terminal comparison table
fprintf('\n--- Terminal comparison: %s vs. %s ---\n', label_a_disp, label_b_disp);
fprintf('%-22s %24s %24s\n', '', label_a_disp, label_b_disp);
fprintf('%-22s %24.2f %24.2f\n', 'Flight time [s]', result_a.t(end), result_b.t(end));
fprintf('%-22s %24.2f %24.2f\n', 'Fuel used [kg]', result_a.fuel_used, result_b.fuel_used);
fprintf('%-22s %24.2f %24.2f\n', 'Final mass [kg]', result_a.x(end, 7), result_b.x(end, 7));
fprintf('%-22s %24.3f %24.3f\n', 'Miss distance [m]', miss_a, miss_b);

%% Figure 1: 3D trajectory + terminal metrics
figure('Name', [fig_name ' - 3D Trajectory'], 'Color', 'w', 'Units', 'normalized', 'Position', [0.1 0.1 0.5 0.6]);

local_plot_trajectory(result_a.x(:, 1), result_a.x(:, 2), result_a.x(:, 3), label_a_disp, converged_a, 'b-');
hold on; grid on;
local_plot_trajectory(result_b.x(:, 1), result_b.x(:, 2), result_b.x(:, 3), label_b_disp, converged_b, 'm--');
plot3(result_a.x(1, 1), result_a.x(1, 2), result_a.x(1, 3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8, 'DisplayName', 'Start');
plot3(target(1), target(2), target(3), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 10, 'DisplayName', 'Target');

has_intermediate_point = isfield(mission_parameters, 'bUseIntermediatePoint') && mission_parameters.bUseIntermediatePoint ...
    && isfield(mission_parameters, 'gammaIntermediatePositionWeights') && any(mission_parameters.gammaIntermediatePositionWeights ~= 0);
if has_intermediate_point
    ip = mission_parameters.mIntermediatePointPosition;
    plot3(ip(1), ip(2), ip(3), 'ys', 'MarkerFaceColor', 'y', 'MarkerSize', 10, 'DisplayName', 'Intermediate Point');
end

xlabel('North (X) [m]'); ylabel('West (Y) [m]'); zlabel('Up (Z) [m]');
title([fig_name ': Point-Mass Trajectory Comparison']);
legend('Location', 'best');
view(3);

metrics_text = {
    '\bf--- TERMINAL METRICS ---'
    sprintf('\\bf%s: \\rmt_f=%.1f s, fuel=%.2f kg, miss=%.2f m', label_a_disp, result_a.t(end), result_a.fuel_used, miss_a)
    sprintf('\\bf%s: \\rmt_f=%.1f s, fuel=%.2f kg, miss=%.2f m', label_b_disp, result_b.t(end), result_b.fuel_used, miss_b)
    };
annotation('textbox', [0.02, 0.78, 0.4, 0.17], ...
    'String', metrics_text, 'EdgeColor', 'k', 'LineWidth', 1, ...
    'BackgroundColor', [0.95 0.95 0.95], 'FaceAlpha', 0.8, ...
    'FitBoxToText', 'on', 'Interpreter', 'tex', 'FontSize', 10);

%% Figure 2: Telemetry (downrange, altitude, speed, mass, thrust magnitude)
figure('Name', [fig_name ' - Telemetry'], 'Color', 'w', 'Units', 'normalized', 'Position', [0.15 0.15 0.7 0.55]);

speed_a = sqrt(sum(result_a.x(:, 4:6).^2, 2));
speed_b = sqrt(sum(result_b.x(:, 4:6).^2, 2));
thrust_mag_a = sqrt(sum(result_a.u.^2, 2));
thrust_mag_b = sqrt(sum(result_b.u.^2, 2));

vars_a = {result_a.x(:, 1), result_a.x(:, 3), speed_a, result_a.x(:, 7), thrust_mag_a};
vars_b = {result_b.x(:, 1), result_b.x(:, 3), speed_b, result_b.x(:, 7), thrust_mag_b};
names  = {'X (North) [m]', 'Z (Up) [m]', 'Speed [m/s]', 'Mass [kg]', '|T| [N]'};

if ~verLessThan('matlab', '9.1')
    tlo = tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
    for i = 1:5
        ax = nexttile(tlo);
        local_plot_series(ax, result_a.t, vars_a{i}, label_a_disp, converged_a, 'b-');
        hold(ax, 'on'); grid on; xlabel('Time [s]'); ylabel(names{i});
        title(names{i}, 'Interpreter', 'none');
        local_plot_series(ax, result_b.t, vars_b{i}, label_b_disp, converged_b, 'm--');
        if i == 1
            legend(ax, 'Location', 'best');
        end
    end
else
    for i = 1:5
        subplot(2, 3, i);
        local_plot_series(gca, result_a.t, vars_a{i}, label_a_disp, converged_a, 'b-');
        hold on; grid on; xlabel('Time [s]'); ylabel(names{i});
        title(names{i}, 'Interpreter', 'none');
        local_plot_series(gca, result_b.t, vars_b{i}, label_b_disp, converged_b, 'm--');
    end
end

end

function tf = local_is_converged(result)
% A result with no `converged` field (e.g. the closed-form LQ-analytic
% solution) is exact by construction and always treated as converged.
tf = ~isfield(result, 'converged') || result.converged;
end

function label = local_decorate_label(label, converged)
if ~converged
    label = [label ' [NOT CONVERGED]'];
end
end

function local_plot_trajectory(x, y, z, label, converged, ok_style)
if converged
    plot3(x, y, z, ok_style, 'LineWidth', 2, 'DisplayName', label);
else
    plot3(x, y, z, ':', 'Color', [0.85 0 0], 'LineWidth', 2.5, 'DisplayName', label);
end
end

function local_plot_series(ax, t, y, label, converged, ok_style)
if converged
    plot(ax, t, y, ok_style, 'LineWidth', 1.2, 'DisplayName', label);
else
    plot(ax, t, y, ':', 'Color', [0.85 0 0], 'LineWidth', 1.8, 'DisplayName', label);
end
end
