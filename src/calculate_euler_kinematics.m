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

% Guard against the Euler-angle gimbal-lock singularity at theta = ±90°.
% tan(theta) and 1/cos(theta) both blow up there, producing Inf/NaN in the
% ODE state.  The attitude controller limits commanded pitch to ±30°, but
% this guard protects against unexpected initial conditions or large disturbances.
if abs(cos(theta)) < 1e-6
    error('calculate_euler_kinematics:Singularity', ...
        ['Gimbal-lock singularity: pitch theta = %.4f rad (%.1f deg) is at or near ' ...
         '+/-90 deg.  Consider switching to a quaternion attitude representation.'], ...
        theta, rad2deg(theta));
end

% Construct the kinematic transformation matrix for Z-Y-X rotation sequence
euler_kinematics_matrix = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
    0, cos(phi),           -sin(phi);
    0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% Multiply matrix by body rates to get Euler angle derivatives
euler_dot = euler_kinematics_matrix * rates;

end