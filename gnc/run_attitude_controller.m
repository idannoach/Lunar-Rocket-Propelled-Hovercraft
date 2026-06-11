function [desired_Fz, desired_My] = run_attitude_controller(x_curr, cmd_x, cmd_z, hovercraft_parameters)

%% ATTITUDE CONTROL (Inner Loop)
% Extract variables from the state vector
mass  = x_curr(13);
theta = x_curr(8);
q     = x_curr(11);

% --- 1. Total Thrust Vector Magnitude ---
% When pitched, you lose vertical lift. We must command the hypotenuse
% of the required X and Z accelerations to maintain altitude.
% (Negative because Body Z points down, so we need negative Fz to pull up)
desired_Fz = -mass * norm([cmd_x, cmd_z]);

% --- 2. Pitch Angle Command & Limits ---
desired_theta = atan2(-cmd_x, -cmd_z);

% Limit the pitch angle to +/- 30 degrees to prevent flipping over
% and completely losing vertical lift authority.
max_pitch = deg2rad(30);
desired_theta = max(-max_pitch, min(max_pitch, desired_theta));

% --- 3. Dynamic Inversion PD Controller ---
% Instead of raw Nm gains, we tune for Angular Acceleration (alpha)
% using natural frequency (omega_n) and damping ratio (zeta).
omega_n = 3.0;  % Bandwidth [rad/s] - How aggressively it tracks the angle
zeta    = 0.85; % Damping - < 1.0 is slightly underdamped for fast response

% Calculate standard linear gains
Kp = omega_n^2;
Kd = 2 * zeta * omega_n;

% Calculate errors (Wrapping theta error to [-pi, pi] for mathematical stability)
theta_error = desired_theta - theta;
theta_error = atan2(sin(theta_error), cos(theta_error));
q_error     = 0 - q;

% Commanded angular acceleration [rad/s^2]
alpha_y_cmd = Kp * theta_error + Kd * q_error;

% Dynamic Inversion: Scale by instantaneous Inertia (M = I * alpha)
[~, Iyy, ~] = calculate_moments_of_inertia(mass, hovercraft_parameters);
desired_My = Iyy * alpha_y_cmd;


end