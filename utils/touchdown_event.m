function [value, isterminal, direction] = touchdown_event(t, x, target_Z)
% touchdown_event - ODE45 event function to stop simulation upon impact.
%
% Inputs:
%   target_Z - The exact depth of the crater floor (e.g., 4000 m).
%              Remember NED Z is positive downwards.

current_Z = x(3);

% 1. The Value: What we want to cross zero.
% When current_Z equals target_Z, value becomes 0.
value = current_Z - target_Z;

% 2. Is Terminal: Does crossing zero stop the integration?
% 1 = Yes (Stop), 0 = No (Keep going, just log the event)
isterminal = 1;

% 3. Direction: Does it matter which way it crosses zero?
% 0 = All crossings, 1 = Increasing only, -1 = Decreasing only.
% Since Z is increasing as we descend into the crater, we use 1.
direction = 1;

end