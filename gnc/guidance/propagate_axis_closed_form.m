function traj = propagate_axis_closed_form(r0, V0, g, alpha, beta, gamma, rc, tf, t1)
% propagate_axis_closed_form - Exact (non-numerical) point-mass trajectory
%                               shape for one decoupled axis under the
%                               optimal LQ guidance law, Nataf & Shaferman
%                               (2024), Sec IV.
%
% Along the optimal trajectory, the terminal Zero-Effort-Miss/Velocity and
% the Zero-Effort-Intermediate-Miss are CONSTANTS of the motion (this is
% what the paper's Z-transform, Eq 10-14, is designed to produce). This
% function solves the per-axis 3x3 system ONCE, using the initial state
% (t=0), to obtain those constants (via gnc/calculate_axis_control.m), and
% then uses the fact that the resulting u*(t) (Eq 35) is piecewise-affine
% in t (breakpoint at t1) to derive the exact closed-form quadratic V(t)
% and cubic r(t) polynomials by direct analytic integration - reproducing
% the paper's Eq 58/60 structure without hand-transcribing that algebra.
%
% This is used for OFFLINE trajectory shaping (intermediate-point
% selection, Sec IV.B, see gnc/select_intermediate_point.m) - NOT by the
% real-time feedback law, which re-solves gnc/calculate_axis_control.m
% every controller tick against the true evolving state.
%
% Inputs (all scalar, one axis, paper's frame convention):
%   r0, V0  - initial position/velocity for this axis
%   g       - gravitational acceleration for this axis (constant)
%   alpha, beta, gamma - terminal position / terminal velocity /
%             intermediate-point weights for this axis (gamma=0 disables
%             the intermediate point on this axis)
%   rc      - intermediate point position for this axis
%   tf, t1  - final time and intermediate-point time [s], 0 <= t1 <= tf
%
% Output: traj, a struct with fields:
%   r_t1, V_t1  - exact state at t1
%   r_tf, V_tf  - exact state at tf
%   u_sq_integral - exact integral_0^tf u(t)^2 dt (closed-form, no
%                   numerical-integration error)
%   max_r       - exact maximum of r(t) over [0, tf]
%   seg         - 1x2 struct array with the per-segment polynomial
%                 coefficients, for use by evaluate_axis_trajectory.m

t1 = min(max(t1, 0), tf); % defensive clamp

%% Solve the 3x3 system once, at t=0 (tgo_f=tf, tgo_1=t1)
Zbar_fr0 = r0 + tf * V0 + 0.5 * g * tf^2;
Zbar_fV0 = V0 + g * tf;
Zbar_10  = r0 + t1 * V0 + 0.5 * g * t1^2;
DeltaZbar_10 = Zbar_10 - rc;

[~, f] = calculate_axis_control(tf, t1, tf, t1, alpha, beta, gamma, Zbar_fr0, Zbar_fV0, DeltaZbar_10);

Z0  = [alpha * Zbar_fr0; beta * Zbar_fV0; gamma * DeltaZbar_10]; % weighted (Eq 47)
Ztf = f * Z0; % [Z_fr(tf); Z_fV(tf); DeltaZ1(t1)], WEIGHTED constants (Eq 22a/46)

%% Piecewise-affine control law u(t) = c0 + c1*t (Eq 35, using the weighted
% terminal constants above directly - these are the SAME constants for the
% whole flight, since they are conserved along the optimal trajectory).
% Eq 35: u(t) = -tgo_f(t)*alpha*Z_fr(tf) - beta*Z_fV(tf) - tgo_1(t)*gamma*DeltaZ1(t1),
% where Z_fr(tf)=Ztf(1) etc. are already weighted - do NOT multiply by
% alpha/beta/gamma again here (that would double-weight them).
% Segment 1: [0, t1] - terminal and intermediate-point terms both active
c1_seg1 = alpha * Ztf(1) + gamma * Ztf(3);
c0_seg1 = -tf * alpha * Ztf(1) - beta * Ztf(2) - t1 * gamma * Ztf(3);

% Segment 2: (t1, tf] - intermediate-point term switches off (Eq 15/36)
c1_seg2 = alpha * Ztf(1);
c0_seg2 = -tf * alpha * Ztf(1) - beta * Ztf(2);

seg(1) = local_build_segment(0, t1, c0_seg1, c1_seg1, r0, V0, g);
seg(2) = local_build_segment(t1, tf, c0_seg2, c1_seg2, seg(1).r_end, seg(1).V_end, g);

traj = struct( ...
    'r0', r0, 'V0', V0, 'g', g, 'alpha', alpha, 'beta', beta, 'gamma', gamma, ...
    'rc', rc, 'tf', tf, 't1', t1, ...
    'seg', seg, ...
    'r_t1', seg(1).r_end, 'V_t1', seg(1).V_end, ...
    'r_tf', seg(2).r_end, 'V_tf', seg(2).V_end, ...
    'u_sq_integral', seg(1).u_sq_integral + seg(2).u_sq_integral, ...
    'max_r', max([seg(1).extrema_r, seg(2).extrema_r]));

end

function s = local_build_segment(ta, tb, c0, c1, ra, Va, g)
% Analytic evaluation of one affine-control segment: u(t) = c0 + c1*t for
% t in [ta,tb], given the state (ra,Va) at t=ta. Returns the exact state
% at tb (quadratic V, cubic r) and the exact extrema of r(t) on [ta,tb].
dur = tb - ta;

a1 = g + c0 + c1 * ta; % V(t) = Va + a1*tau + a2*tau^2, tau = t - ta
a2 = c1 / 2;

s.ta = ta; s.tb = tb; s.c0 = c0; s.c1 = c1;
s.ra = ra; s.Va = Va; s.a1 = a1; s.a2 = a2;

s.V_end = Va + a1 * dur + a2 * dur^2;
s.r_end = ra + Va * dur + a1 * dur^2 / 2 + a2 * dur^3 / 3;

% Exact integral of u(t)^2 over [ta,tb], u(t) = c0 + c1*t
s.u_sq_integral = (c0^2 * tb + c0 * c1 * tb^2 + c1^2 * tb^3 / 3) ...
                 - (c0^2 * ta + c0 * c1 * ta^2 + c1^2 * ta^3 / 3);

% Extrema of r(t): dr/dt = V(t) = Va + a1*tau + a2*tau^2 = 0
extrema_r = [ra, s.r_end]; % segment endpoints always included
if dur > 0
    if a2 ~= 0
        tau_roots = roots([a2, a1, Va]);
    elseif a1 ~= 0
        tau_roots = -Va / a1;
    else
        tau_roots = [];
    end
    for k = 1:numel(tau_roots)
        tau_r = tau_roots(k);
        if isreal(tau_r) && tau_r > 0 && tau_r < dur
            r_at = ra + Va * tau_r + a1 * tau_r^2 / 2 + a2 * tau_r^3 / 3;
            extrema_r(end+1) = r_at; %#ok<AGROW>
        end
    end
end
s.extrema_r = extrema_r;

end
