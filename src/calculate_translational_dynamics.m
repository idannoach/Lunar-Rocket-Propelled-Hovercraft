function vel_dot = calculate_translational_dynamics(Forces_body, mass, R_b2n, GP, r, p, q, u, v, w)
% calculate_translational_dynamics - Computes the translational acceleration
%                                    of the vehicle in the Body frame.
%
% This function applies Newton's Second Law in a rotating reference frame,
% accounting for gravitational forces and the Coriolis effect induced by
% the vehicle's angular rates.
%
% Inputs:
%   Forces_body - 3x1 vector of total thrust forces in Body frame [Fx; Fy; Fz] [N]
%   mass        - Instantaneous vehicle mass [kg]
%   R_b2n       - 3x3 Direction Cosine Matrix from Body to NED frame
%   GP          - Global Parameters struct containing gravity constants
%   r, p, q     - Body angular rates (yaw, roll, pitch) [rad/s]
%   u, v, w     - Body linear velocities (forward, right, down) [m/s]
%
% Outputs:
%   vel_dot     - 3x1 vector of linear accelerations in Body frame [m/s^2]

% Define gravity vector in the local NED frame (Z points down)
g_ned = [0; 0; GP.g_lunar];

% Rotate the gravity vector into the vehicle's Body frame
R_n2b = R_b2n'; % The transpose of an orthogonal rotation matrix is its inverse
g_body = R_n2b * g_ned;

% Calculate the Coriolis acceleration term: cross(omega, velocity)
cross_omega_v = [q*w - r*v;
    r*u - p*w;
    p*v - q*u];

% Calculate final acceleration: a = F/m + g - (omega x v)
vel_dot = (Forces_body / mass) + g_body - cross_omega_v;

end