function mission_parameters = select_intermediate_point(global_parameters, mission_parameters)
% select_intermediate_point - Optimal intermediate-point selection, Sec
%                              IV.B of Nataf & Shaferman (2024), "Optimal
%                              Linear Quadratic Powered Descent With An
%                              Optimally Selected Intermediate Point,"
%                              AIAA SciTech 2024.
%
% Automatically chooses an intermediate waypoint (position + time) so the
% optimal LQ guidance trajectory passes through a specified approach cone
% above the target and avoids ground collision, while minimizing the cost
% function. Operates entirely on the idealized point-mass model (matching
% the paper's own methodology, see Sec V) via
% gnc/propagate_axis_closed_form.m - independent of the vehicle's full
% 6-DOF dynamics.
%
% If the trajectory without an intermediate point already satisfies the
% approach-cone requirement (Step 1), this returns mission_parameters
% with gammaIntermediatePositionWeights forced to zero and no further
% work is done.
%
% Input/Output: mission_parameters struct (read fields: mTargetPosition,
% mInitialPosition, mpsInitialVelocity, sFinalSimulationTime,
% alpha/beta/gammaFinalPositionWeights, mApproachConeRadius/Height,
% mConeHeightIncrement, iMaxConeHeightIterations. Written fields:
% gammaIntermediatePositionWeights, mIntermediatePointPosition,
% sIntermediatePointTime, mApproachConeRadius, mApproachConeHeight).

target_nwu = mission_parameters.mTargetPosition(:);
r0 = flip_z_axis(mission_parameters.mInitialPosition(:) - target_nwu);
V0 = flip_z_axis(mission_parameters.mpsInitialVelocity(:));
g  = flip_z_axis([0; 0; -global_parameters.g_lunar]);

% Z-axis canonicalization: every collision/cone check below (idx = find(
% r_samp(:,3) >= -h, ...), collided_before = ... >= 0, max_r_before_t1 >=
% 0) implicitly assumes the trajectory STARTS above the target in the
% paper's Z-down frame (r0(3) < 0) and its paper-Z increases toward 0 - a
% descent. main.m's Return Ascent leg (paper.tex Phase 3) reverses the
% boundary conditions, so r0(3) > 0 (starts BELOW the target) and paper-Z
% DECREASES toward 0 instead - every one of those checks is then either
% trivially true from t=0 (Step 3's collision check) or looking for the
% wrong crossing (Step 1), and select_intermediate_point never converges -
% empirically, h inflates to its iteration cap on every single t_f
% candidate instead.
%
% Fix: flip the sign of the Z-COMPONENT of r0, V0, AND g together
% (z_mirror) before feeding them to propagate_axis_closed_form for axis 3
% specifically. This is a valid coordinate transform of the scalar
% dynamics r'' = u + g (flipping r, V, g by the same constant just flips
% the resulting u and r(t) by that same constant too), so it doesn't
% change the underlying axis-control math at all - it just makes an
% ascent's Z-trajectory look like a canonical descent (starts negative,
% rises to 0) to every check below, which can then stay written exactly
% as before. X/Y (i=1,2) have no such ambiguity (flip_z_axis never
% touches them) and are untouched. Only the very end (Step 4's final
% mIntermediatePointPosition) needs to un-mirror back to the real sign.
z_mirror = -sign(r0(3));
if z_mirror == 0
    z_mirror = 1; % degenerate case: initial altitude == target altitude
end

alpha_w = mission_parameters.alphaFinalPositionWeights(:);
beta_w  = mission_parameters.betaFinalVelocityWeights(:);
gamma_w = mission_parameters.gammaIntermediatePositionWeights(:);

tf = mission_parameters.sFinalSimulationTime;

Rmax0 = mission_parameters.mApproachConeRadius;
h0    = mission_parameters.mApproachConeHeight;
Rmax  = Rmax0;
h     = h0;

h_increment = mission_parameters.mConeHeightIncrement;
max_iter    = mission_parameters.iMaxConeHeightIterations;

N_samples = 2000;
t_samples = linspace(0, tf, N_samples);

cone_pass = false;
collision = true;
t1 = tf;

for iter = 0:max_iter
    %% Step 1 - trajectory without an intermediate point (gamma=0); check
    % whether it already passes through the approach cone without
    % colliding with the ground.
    r_samp = zeros(N_samples, 3);
    for i = 1:3
        if i == 3
            traj0_i = propagate_axis_closed_form(z_mirror*r0(i), z_mirror*V0(i), z_mirror*g(i), alpha_w(i), beta_w(i), 0, 0, tf, tf);
        else
            traj0_i = propagate_axis_closed_form(r0(i), V0(i), g(i), alpha_w(i), beta_w(i), 0, 0, tf, tf);
        end
        r_samp(:, i) = evaluate_axis_trajectory(traj0_i, t_samples)';
    end

    idx = find(r_samp(:, 3) >= -h, 1, 'first');
    if isempty(idx)
        cone_pass = false;
    else
        if idx > 1
            xy_at_h = interp1(r_samp(idx-1:idx, 3), r_samp(idx-1:idx, 1:2), -h);
        else
            xy_at_h = r_samp(idx, 1:2);
        end
        collided_before = any(r_samp(1:max(idx-1,1), 3) >= 0);
        cone_pass = (xy_at_h(1)^2 + xy_at_h(2)^2 <= Rmax^2) && ~collided_before;
    end

    if cone_pass
        fprintf('select_intermediate_point: trajectory without an intermediate point already satisfies the approach cone - no waypoint needed.\n');
        mission_parameters.gammaIntermediatePositionWeights = [0; 0; 0];
        return;
    end

    %% Step 2 - select t1 (Eq 57): last time ||r(t)|| equals the cone's slant distance
    Rslant = sqrt(Rmax^2 + h^2);
    rnorm = vecnorm(r_samp, 2, 2);
    sign_diff = diff(sign(rnorm - Rslant));
    crossing_idx = find(sign_diff ~= 0, 1, 'last');

    if isempty(crossing_idx)
        warning('select_intermediate_point:NoConeCrossing', ...
            'No-intermediate-point trajectory never crosses the cone slant distance (%.1f m); using tf/2 as a fallback t1.', Rslant);
        t1 = tf / 2;
    else
        t1 = interp1(rnorm(crossing_idx:crossing_idx+1), t_samples(crossing_idx:crossing_idx+1), Rslant);
    end

    %% Step 3 - ground collision check (z-axis only; independent of rc_x, rc_y)
    % Restricted to [0, t1] (before the intermediate point), matching the
    % paper's own worked example (Table 1's r_max_z(0,t1) definition) -
    % NOT the full [0,tf] domain, which would always include the intended
    % touchdown point r_z(tf)~=0 and false-positive on any converged
    % trajectory.
    traj_z = propagate_axis_closed_form(z_mirror*r0(3), z_mirror*V0(3), z_mirror*g(3), alpha_w(3), beta_w(3), gamma_w(3), -h, tf, t1);
    max_r_before_t1 = max(traj_z.seg(1).extrema_r);
    collision = max_r_before_t1 >= 0;

    if ~collision
        break;
    end

    if iter == max_iter
        warning('select_intermediate_point:HeightIterationLimit', ...
            'Cone-height adjustment did not clear the ground collision within %d iterations. Proceeding with the last candidate: h=%.1f m, Rmax=%.1f m.', ...
            max_iter, h, Rmax);
        break;
    end

    h = h + h_increment;
    Rmax = Rmax0 / h0 * h;
end

%% Step 4 - optimal (rc_x, rc_y), Eq 70-78
% J_j(rc_j) is exactly quadratic in rc_j (sum/integral of squares of
% affine functions of rc_j), so 3 sample points determine it exactly.
A = zeros(2, 1);
B = zeros(2, 1);
for j = 1:2
    rc_samples = [-Rmax, 0, Rmax];
    J_samples = zeros(1, 3);
    for k = 1:3
        traj_j = propagate_axis_closed_form(r0(j), V0(j), g(j), alpha_w(j), beta_w(j), gamma_w(j), rc_samples(k), tf, t1);
        J_samples(k) = 0.5 * alpha_w(j) * traj_j.r_tf^2 ...
                     + 0.5 * beta_w(j)  * traj_j.V_tf^2 ...
                     + 0.5 * gamma_w(j) * (traj_j.r_t1 - rc_samples(k))^2 ...
                     + 0.5 * traj_j.u_sq_integral;
    end
    p = polyfit(rc_samples, J_samples, 2); % exact, not a lossy fit (J is exactly quadratic)
    A(j) = p(1);
    B(j) = p(2);
end

rc_unconstrained = [-B(1) / (2 * A(1)); -B(2) / (2 * A(2))];

if sum(rc_unconstrained.^2) <= Rmax^2
    rc_xy = rc_unconstrained;
else
    % Constrained minimum on the circle boundary via Lagrange multipliers
    % (Eq 74-78). A(j) > 0 always (Eq 73a), so for lambda >= 0 the
    % curvature A(j)+lambda stays positive throughout, and h_lambda(lambda)
    % decreases monotonically from h_lambda(0) > 0 (since the unconstrained
    % point lies outside the circle) toward -Rmax^2 as lambda -> inf, so
    % exactly one root exists in [0, inf).
    h_lambda = @(lam) (-B(1) / (2 * (A(1) + lam)))^2 + (-B(2) / (2 * (A(2) + lam)))^2 - Rmax^2;

    lam_lo = 0;
    lam_hi = max(1, max(A)); % start above 0, grow until sign change
    while h_lambda(lam_hi) > 0
        lam_hi = lam_hi * 10;
    end
    lambda_star = fzero(h_lambda, [lam_lo, lam_hi]);
    rc_xy = [-B(1) / (2 * (A(1) + lambda_star)); -B(2) / (2 * (A(2) + lambda_star))];
end

%% Finalize
% -h is the canonical (mirrored-if-ascending) cone-apex offset used by the
% checks above; un-mirror it back to the real paper-frame sign before
% converting to NWU, so the waypoint lands on the correct side of the
% target (above it for a descent, below it for an ascent - see z_mirror
% above).
mission_parameters.mIntermediatePointPosition = target_nwu + flip_z_axis([rc_xy(1); rc_xy(2); z_mirror*(-h)]);
mission_parameters.sIntermediatePointTime = t1;
mission_parameters.mApproachConeHeight = h;
mission_parameters.mApproachConeRadius = Rmax;

fprintf('select_intermediate_point: t1=%.2f s, intermediate point (NWU)=[%.1f, %.1f, %.1f] m, Rmax=%.1f m, h=%.1f m.\n', ...
    t1, mission_parameters.mIntermediatePointPosition(1), mission_parameters.mIntermediatePointPosition(2), ...
    mission_parameters.mIntermediatePointPosition(3), Rmax, h);

end
