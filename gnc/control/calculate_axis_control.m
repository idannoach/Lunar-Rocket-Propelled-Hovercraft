function [u, f] = calculate_axis_control(tgo_f, tgo_1, tf, t1, alpha, beta, gamma, Zbar_fr, Zbar_fV, DeltaZbar_1)
% calculate_axis_control - Optimal LQ control for a single decoupled axis,
%                           from Nataf & Shaferman (2024), "Optimal Linear
%                           Quadratic Powered Descent With An Optimally
%                           Selected Intermediate Point," AIAA SciTech.
%
% Because the terminal-miss (M_frr), terminal-velocity (M_fVV), and
% intermediate-point-miss (M_1rr) weighting matrices are all diagonal
% (Eq 7-8 of the paper), the 3-axis problem decouples into three
% independent scalar two-point boundary value problems. This function
% solves ONE such axis.
%
% Inputs (paper-frame, i.e. this axis's own units/sign convention - the
% caller is responsible for any NWU<->paper frame conversion):
%   tgo_f       - time-to-go to the final time tf   [s], tgo_f = max(tf-t,0)
%   tgo_1       - time-to-go to the intermediate time t1 [s], tgo_1 = max(t1-t,0)
%                 (zero for t > t1, matching the paper's step function 1(tgo_1))
%   tf, t1      - final time and intermediate-point time [s]
%   alpha       - terminal position (miss) weight for this axis
%   beta        - terminal velocity weight for this axis
%   gamma       - intermediate-point position weight for this axis (0 = no
%                 intermediate point on this axis; law degrades automatically)
%   Zbar_fr     - Zero-Effort-Miss for this axis at the current time (Eq 48a)
%   Zbar_fV     - Zero-Effort-Velocity for this axis at the current time (Eq 48b)
%   DeltaZbar_1 - Zero-Effort-Intermediate-Miss minus the intermediate point
%                 position for this axis at the current time (Eq 24, 48c)
%
% Outputs:
%   u - optimal thrust-acceleration command for this axis (Eq 49). This is
%       the COMPLETE thrust-acceleration command - the Zbar terms already
%       account for gravity drift, so no separate gravity compensation is
%       needed by the caller.
%   f - the 3x3 inverse matrix from Eq 43/44 (returned so offline
%       trajectory-shaping code can reuse it without re-deriving it)

%% Build the symmetric 3x3 system (Eq 41-42) and invert it directly.
% This is mathematically identical to the paper's expanded closed-form
% f_ij/Lambda_i rational expressions (Eq 44-45), but far less error-prone
% to implement than transcribing that algebra by hand. Proposition 1 in
% the paper (S = I + P, P positive semi-definite) guarantees S is always
% invertible, so no singularity handling is required.
psi_rr = 1 + alpha^2 * tgo_f^3 / 3;
psi_rV = alpha * beta * tgo_f^2 / 2;
psi_VV = 1 + beta^2 * tgo_f;

Xi_rr = alpha * gamma * (tgo_1^3 / 3 + tgo_1^2 / 2 * (tf - t1));
Xi_rV = beta * gamma * tgo_1^2 / 2;

Omega_rr = 1 + gamma^2 * tgo_1^3 / 3;

S = [psi_rr, psi_rV, Xi_rr;
     psi_rV, psi_VV, Xi_rV;
     Xi_rr,  Xi_rV,  Omega_rr];

f = inv(S);

%% Optimal control law (Eq 49), collapsed to scalars since diagonal.
gate = tgo_1 > 0; % step function 1(tgo_1): intermediate-point term is only active before t1

gain_r  = tgo_f * alpha * f(1,1) + beta * f(2,1) + tgo_1 * gamma * f(3,1);
gain_V  = tgo_f * alpha * f(1,2) + beta * f(2,2) + tgo_1 * gamma * f(3,2);
gain_1  = tgo_f * alpha * f(1,3) + beta * f(2,3) + tgo_1 * gamma * f(3,3);

u = -gain_r * alpha * Zbar_fr ...
    -gain_V * beta  * Zbar_fV ...
    -gain_1 * gamma * DeltaZbar_1 * gate;

end
