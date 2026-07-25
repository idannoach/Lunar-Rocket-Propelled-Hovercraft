function falcon_out = run_falcon(global_parameters, mission_parameters, hovercraft_parameters, tf_margin_bounds)
% run_falcon - Thin wrapper main.m calls to solve the FALCON.m point-mass
%              min-fuel powered-descent benchmark. See
%              opt_control/main_falcon_descent.m for the problem
%              formulation.

if nargin < 4
    tf_margin_bounds = [1, 1];
end

falcon_out = main_falcon_descent(global_parameters, mission_parameters, hovercraft_parameters, tf_margin_bounds);

end
