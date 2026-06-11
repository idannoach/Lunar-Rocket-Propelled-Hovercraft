function euler_dot = calculate_euler_kinematics(rates, phi, theta)
% calculate_euler_kinematics - Transforms Body frame angular rates into
%                              Euler angle rates.
%
% This kinematic differential equation maps the physical rotation rates
% (p, q, r) to the rate of change of the spatial orientation angles.
%
% Inputs:
%   rates - 3x1 vector of Body angular rates [p; q; r] [rad/s]
%   phi   - Current Roll angle [rad]
%   theta - Current Pitch angle [rad]
%
% Outputs:
%   euler_dot - 3x1 vector of Euler angle rates [phi_dot; theta_dot; psi_dot] [rad/s]

% Construct the kinematic transformation matrix for Z-Y-X rotation sequence
euler_kinematics_matrix = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi),           -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% Multiply matrix by body rates to get Euler angle derivatives
euler_dot = euler_kinematics_matrix * rates;

end