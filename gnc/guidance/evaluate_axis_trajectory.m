function [r, V, u] = evaluate_axis_trajectory(traj, t_query)
% evaluate_axis_trajectory - Exact (closed-form polynomial) evaluation of a
%                             single-axis trajectory produced by
%                             gnc/propagate_axis_closed_form.m at arbitrary
%                             query times.
%
% Inputs:
%   traj     - struct returned by propagate_axis_closed_form
%   t_query  - vector of times [s] in [0, traj.tf] to evaluate at
%
% Outputs (same size as t_query):
%   r, V, u  - position, velocity, and thrust-acceleration command for
%              this axis at each query time

r = zeros(size(t_query));
V = zeros(size(t_query));
u = zeros(size(t_query));

for k = 1:numel(t_query)
    t = t_query(k);
    if t <= traj.t1
        s = traj.seg(1);
    else
        s = traj.seg(2);
    end

    tau = t - s.ta;
    tau = max(0, min(tau, s.tb - s.ta)); % clamp defensively to the segment domain

    r(k) = s.ra + s.Va * tau + s.a1 * tau^2 / 2 + s.a2 * tau^3 / 3;
    V(k) = s.Va + s.a1 * tau + s.a2 * tau^2;
    u(k) = s.c0 + s.c1 * (s.ta + tau);
end

end
