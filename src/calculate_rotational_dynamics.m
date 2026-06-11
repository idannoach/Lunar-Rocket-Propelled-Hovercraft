function rates_dot = calculate_rotational_dynamics(Moments_body, mass, HP, r, p, q)
% calculate_rotational_dynamics - Computes the angular acceleration of the
%                                 vehicle in the Body frame.
%
% This function solves Euler's equations of motion for a rigid body,
% updating the inertia tensor dynamically based on the current fuel mass.
%
% Inputs:
%   Moments_body - 3x1 vector of control moments in Body frame [Mx; My; Mz] [Nm]
%   mass         - Instantaneous vehicle mass [kg]
%   HP           - Hovercraft Parameters struct containing inertia polynomials
%   r, p, q      - Body angular rates (yaw, roll, pitch) [rad/s]
%
% Outputs:
%   rates_dot    - 3x1 vector of angular accelerations [p_dot; q_dot; r_dot] [rad/s^2]

% Calculate the instantaneous principal moments of inertia based on current mass
[Ixx, Iyy, Izz] = calculate_moments_of_inertia(mass, HP);

% Construct the 3x3 diagonal inertia tensor matrix
I = diag([Ixx, Iyy, Izz]);

% Calculate the gyroscopic coupling term: cross(omega, I * omega)
cross_omega_H = [q*(Izz*r) - r*(Iyy*q);
    r*(Ixx*p) - p*(Izz*r);
    p*(Iyy*q) - q*(Ixx*p)];

% Solve for angular acceleration: alpha = inv(I) * (Moments - omega x H)
rates_dot = I \ (Moments_body - cross_omega_H);

end