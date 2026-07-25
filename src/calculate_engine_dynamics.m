function throttles_dot = calculate_engine_dynamics(cmd_throttles, actual_throttles, HP)
% Calculate Engine Lag

% tau = time to 90% / 2.3
tau = (HP.msecResponseTo90PctThrustTime / 1000) / 2.3;

% First order lag derivative: rate of change of the throttles
throttles_dot = (cmd_throttles - actual_throttles) / tau;

end