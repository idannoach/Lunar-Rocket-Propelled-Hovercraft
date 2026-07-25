function [t_out, x_out, mission_parameters, hovercraft_parameters] = solve_mission(global_parameters, mission_parameters, hovercraft_parameters, is6DOF)

isUseIntermediatePoint = mission_parameters.bUseIntermediatePoint;

if isUseIntermediatePoint
    baseline_gamma_weights = mission_parameters.gammaIntermediatePositionWeights;
    baseline_cone_radius   = mission_parameters.mApproachConeRadius;
    baseline_cone_height   = mission_parameters.mApproachConeHeight;
    mission_parameters = select_intermediate_point(global_parameters, mission_parameters);
end

tol_pos  = mission_parameters.mPositionErrorTolerance;
tol_vel  = mission_parameters.mpsVelocityErrorTolerance;
tol_pitch = mission_parameters.radPitchErrorTolerance;
step     = mission_parameters.sTimeStepIncrement;
min_step = mission_parameters.sMinTimeStepIncrement;
max_iter = mission_parameters.iMaxTfIterations;
direction = 1;

t_f = mission_parameters.sFinalSimulationTime;
[t_out, x_out, ] = run_phase1_simulation(global_parameters, mission_parameters, hovercraft_parameters, is6DOF);
pos_error = calculate_miss_distance(x_out, mission_parameters.mTargetPosition);
vel_error = calculate_miss_velocity(x_out);
pitch_error = calculate_miss_pitch(x_out);
score = pos_error / tol_pos + vel_error / tol_vel + pitch_error/tol_pitch;
fprintf('t_f iteration 0: t_f = %.3f s, position error = %.3f m, velocity error = %.3f m/s, pitch error = %.3f deg\n', t_f, pos_error, vel_error, rad2deg(pitch_error));

iter = 0;
while (pos_error >= tol_pos || vel_error >= tol_vel || pitch_error >= tol_pitch) && iter < max_iter && step >= min_step
    iter = iter + 1;

    t_f_candidate = t_f + direction * step;
    mission_parameters.sFinalSimulationTime = t_f_candidate;

    if isUseIntermediatePoint
        mission_parameters.gammaIntermediatePositionWeights = baseline_gamma_weights;
        mission_parameters.mApproachConeRadius = baseline_cone_radius;
        mission_parameters.mApproachConeHeight = baseline_cone_height;
        mission_parameters = select_intermediate_point(global_parameters, mission_parameters);
    end

    [t_out_candidate, x_out_candidate] = run_phase1_simulation(global_parameters, mission_parameters, hovercraft_parameters, is6DOF);

    pos_error_candidate = calculate_miss_distance(x_out_candidate, mission_parameters.mTargetPosition);
    vel_error_candidate = calculate_miss_velocity(x_out_candidate);
    pitch_error_candidate = calculate_miss_pitch(x_out_candidate);
    score_candidate = pos_error_candidate / tol_pos + vel_error_candidate / tol_vel + pitch_error_candidate / tol_pitch;

    fprintf('t_f iteration %d: t_f = %.3f s, position error = %.3f m, velocity error = %.3f m/s, pitch error = %.3f deg\n', ...
        iter, t_f_candidate, pos_error_candidate, vel_error_candidate, rad2deg(pitch_error_candidate));

    if score_candidate < score
        t_f = t_f_candidate;
        pos_error = pos_error_candidate;
        vel_error = vel_error_candidate;
        pitch_error = pitch_error_candidate;
        score = score_candidate;
        t_out = t_out_candidate;
        x_out = x_out_candidate;
    else
        direction = -direction;
        step = step / 2;
    end
end

mission_parameters.sFinalSimulationTime = t_f;

if pos_error < tol_pos && vel_error < tol_vel && pitch_error < tol_pitch
    fprintf('t_f converged after %d iterations: t_f = %.3f s, position error = %.3f m, velocity error = %.3f m/s, pitch error = %.3f deg\n', ...
        iter, t_f, pos_error, vel_error, rad2deg(pitch_error));
elseif step < min_step
    warning('solve_mission:TfStepStalled', ...
        't_f search stalled (step size %.4g below minimum %.4g) before reaching tolerance. Using best result: t_f = %.3f s, position error = %.3f m, velocity error = %.3f m/s, pitch error = %.3f deg', ...
        step, min_step, t_f, pos_error, vel_error, rad2deg(pitch_error));
else
    warning('solve_mission:TfIterationLimit', ...
        't_f iteration did not converge within %d iterations. Using best result: t_f = %.3f s, position error = %.3f m, velocity error = %.3f m/s, pitch error = %.3f deg', ...
        max_iter, t_f, pos_error, vel_error, rad2deg(pitch_error));
end

end
