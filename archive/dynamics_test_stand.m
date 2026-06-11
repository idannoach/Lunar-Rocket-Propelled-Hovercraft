function [dxdt, parasitic_loads] = dynamics_test_stand(t, x, throttles, GP, HP)
% 3-DOF Kinematics with 6-DOF Engine Inputs
%
% This simulates the vehicle constrained to the X-Z plane, but fully 
% calculates the 3D forces and moments from all 6 engines.
%
% Inputs:
%   x         - 7x1 State Vector [X; Z; u; w; theta; q; mass]
%   throttles - 6x1 vector of throttle commands [0.4 to 1.0] for each engine
%
% Outputs:
%   dxdt            - State derivatives for the 3-DOF integrator
%   parasitic_loads - [Fy; Mx; Mz] The out-of-plane forces/moments generated 
%                     by the engines that are physically constrained by the rig.

%% Unpack State Vector
X = x(1); Z = x(2);
u = x(3); w = x(4);
theta = x(5);
q = x(6);
mass = x(7);

%% Calculate 3D Engine Forces & Moments
Forces_body  = zeros(3,1);
Moments_body = zeros(3,1);
mdot_total   = 0;

% Center of Gravity offset (Engines are at z = 0.25, CG is at z = 0.22)
% Assuming CG x and y are exactly 0.
cg_vec = [0; 0; HP.mCG(3)]; 

for i = 1:HP.nEngines
    % Calculate engine position relative to the origin
    theta_rad = deg2rad((i-1) * HP.degEngineRadialAngle);
    pos_engine = [HP.mEngineRadialPos * cos(theta_rad); 
        HP.mEngineRadialPos * sin(theta_rad); 
        HP.mEngineAxialPos];

    % Vector from C.G. to Engine (Moment Arm)
    r_arm = pos_engine - cg_vec;

    % Calculate Thrust Vector for this specific engine
    mdot_i = throttles(i) * HP.gpsecMaxPropFlowRate / 1000; % Convert g/s to kg/s
    T_vec = calculate_thrust_vec(HP.newtonMaxThrust, HP.secIsp, mdot_i, GP.g0, ...
        theta_rad, deg2rad(HP.degThrustRadialAngle));

    % Sum the forces and moments
    Forces_body = Forces_body + T_vec;
    Moments_body = Moments_body + cross(r_arm, T_vec);
    mdot_total = mdot_total + mdot_i;
end

%% Extract Longitudinal Controls & Parasitic Loads
% The rig allows these to affect the vehicle:
Fx_body = Forces_body(1);
Fz_body = Forces_body(3);
My_body = Moments_body(2);

% The rig absorbs these (we output them for analysis):
parasitic_loads = [Forces_body(2); Moments_body(1); Moments_body(3)]; % [Fy, Mx, Mz]

%% 3-DOF KINEMATICS & DYNAMICS (X-Z Plane Only)
% 2D Rotation Matrix (Body to NED)
R_b2n_2x2 = [cos(theta),  sin(theta);
    -sin(theta),  cos(theta)];

pos_dot = R_b2n_2x2 * [u; w];
theta_dot = q;

% Gravity in Body frame
g_ned = [0; GP.g_lunar];
g_body = R_b2n_2x2' * g_ned;

% Translational Dynamics (F = m*a)
u_dot = (Fx_body / mass) + g_body(1) - (q * w);
w_dot = (Fz_body / mass) + g_body(2) + (q * u);

% Rotational Dynamics (M = I*alpha)
Iyy = HP.Iyy_poly(1) * mass + HP.Iyy_poly(2);
q_dot = My_body / Iyy;

% Mass Dynamics
mass_dot = -mdot_total;

%% Pack Derivative Vector
dxdt = [pos_dot(1); pos_dot(2); u_dot; w_dot; theta_dot; q_dot; mass_dot];
end