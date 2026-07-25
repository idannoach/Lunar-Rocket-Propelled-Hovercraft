function main()

[global_parameters, hovercraft_parameters, mission_parameters] = startup();

%% ===================================================================================
%% Missions 1-2: basic point-mass LQ/ZEM-ZEV law (min. fuel) (no intermediate point) vs. FALCON.m
%% ===================================================================================
if mission_parameters.isRunBenchmark(1)

    fprintf('\n============================================================\n');
    fprintf('MISSIONS 1-2: Basic point-mass LQ law (no intermediate point)\n');
    fprintf('============================================================\n');

    % Copy of parameters
    point_mass_mission_parameters = mission_parameters;
    point_mass_hovercraft_parameters = hovercraft_parameters;

    % Override parameters
    point_mass_mission_parameters.bUseIntermediatePoint = false;
    point_mass_mission_parameters.mInitialPosition   = [152400; 30480; 15240];
    point_mass_mission_parameters.mpsInitialVelocity = [-914.4; 0; -150];
    point_mass_mission_parameters.sFinalSimulationTime = 340;
    point_mass_mission_parameters.mTargetPosition = [0; 0; 0];
    
    % Run simulation
    [basic_point_mass_t_out, basic_point_mass_x_out, point_mass_mission_parameters, ~] = solve_mission(global_parameters, point_mass_mission_parameters, point_mass_hovercraft_parameters, false);

    % Run FALCON
    basic_point_mass_falcon_out  = run_falcon(global_parameters, point_mass_mission_parameters, point_mass_hovercraft_parameters);
    
    % Report
    report_point_mass_comparison(point_mass_mission_parameters, basic_point_mass_x_out, 'LQ no intermediate point', ...
        basic_point_mass_falcon_out, 'FALCON no intermediate point', 'Missions 1-2');

    % Log
    log_results(basic_point_mass_t_out, basic_point_mass_x_out, point_mass_mission_parameters, 'logs', 'Mission 1 - LQ no intermediate point');
    log_results(basic_point_mass_falcon_out.t, basic_point_mass_falcon_out.x, point_mass_mission_parameters, 'logs', 'Mission 2 - FALCON no intermediate point', ...
        local_falcon_note(basic_point_mass_falcon_out));

end

%% ====================================================================
%% Missions 3-4: point-mass LQ law (min. fuel) (WITH intermediate point) vs. FALCON.m
%% ====================================================================

if mission_parameters.isRunBenchmark(2)

    fprintf('\n============================================================\n');
    fprintf('MISSIONS 3-4: Point-mass LQ law with an intermediate point\n');
    fprintf('============================================================\n');
    
    % Override parameters
    point_mass_mission_parameters.bUseIntermediatePoint = true;
    point_mass_mission_parameters = select_intermediate_point(global_parameters, point_mass_mission_parameters);
    
    % Run simulation
    [basic_point_mass_t_out, basic_point_mass_x_out, point_mass_mission_parameters, ~] = solve_mission(global_parameters, point_mass_mission_parameters, point_mass_hovercraft_parameters, false);

    % Run FALCON
    basic_point_mass_falcon_out = run_falcon(point_mass_mission_parameters, point_mass_hovercraft_parameters);
    
    % Report
    report_point_mass_comparison(point_mass_mission_parameters, basic_point_mass_x_out, 'LQ with intermediate point', ...
        basic_point_mass_falcon_out, 'FALCON with an intermediate point', 'Missions 3-4');
    
    % Log
    log_results(basic_point_mass_t_out, basic_point_mass_x_out, point_mass_mission_parameters, 'logs', 'Mission 3 - LQ with intermediate point');
    log_results(basic_point_mass_falcon_out.t, basic_point_mass_falcon_out.x, point_mass_mission_parameters, 'logs', 'Mission 4 - FALCON with intermediate point', ...
        local_falcon_note(falcon_ip));
end

%% ====================================================================
%% Missions 5-6: this project's own final mission (config/mission.json),
%% full 6-DOF closed-loop LQ/ZEM-ZEV guidance (t_f-converged) vs. FALCON.m
%% ====================================================================

if isRunFullSim

    fprintf('\n============================================================\n');
    fprintf('MISSIONS 5-6: Final project mission (full 6-DOF)\n');
    fprintf('============================================================\n');
    
    [t_out, x_out, mission_parameters, hovercraft_parameters] = solve_mission(global_parameters, mission_parameters, hovercraft_parameters, true);
    
    lq_project.t = t_out;
    lq_project.x = x_out;
    lq_project.fuel_used = x_out(1, 13) - x_out(end, 13);
    
    if mission_parameters.isRunBenchmark(3)
        falcon_out = run_falcon(mission_parameters, hovercraft_parameters, [0.95, 1.05]);
        
        falcon_project_label = 'FALCON min-fuel';
        if isfield(falcon_out, 'converged') && ~falcon_out.converged
            fprintf('WARNING: Mission 6 FALCON.m benchmark did NOT converge (exit status: %s) - treat it as unreliable, not a valid benchmark.\n', ...
                falcon_out.exit_status);
            falcon_project_label = 'FALCON min-fuel [NOT CONVERGED]';
        end
        
        fprintf('\n--- Terminal comparison: full 6-DOF LQ/ZEM-ZEV guidance vs. %s ---\n', falcon_project_label);
        fprintf('%-22s %18s %24s\n', '', 'LQ guidance', falcon_project_label);
        fprintf('%-22s %18.2f %24.2f\n', 'Flight time [s]', lq_project.t(end), falcon_out.tf);
        fprintf('%-22s %18.2f %24.2f\n', 'Fuel used [kg]', lq_project.fuel_used, falcon_out.fuel_used);
        fprintf('%-22s %18.3f %24.3f\n', 'Miss distance [m]', ...
            calculate_miss_distance(x_out, mission_parameters.mTargetPosition), ...
            norm(falcon_out.x(end, 1:3) - mission_parameters.mTargetPosition(:)'));
        
        log_results(falcon_out.t, falcon_out.x, mission_parameters, 'logs', 'Mission 6 - FALCON min-fuel (final project mission)', ...
            local_falcon_note(falcon_out));
        visualization(hovercraft_parameters, mission_parameters, t_out, x_out, falcon_out);
    end
    log_results(t_out, x_out, mission_parameters, 'logs', 'Mission 5 - Final project mission (full 6-DOF)');
    visualization(hovercraft_parameters, mission_parameters, t_out, x_out);
end

end