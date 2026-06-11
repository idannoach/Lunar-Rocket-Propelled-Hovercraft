function thrust_vec = calculate_thrust_vec(newtonMaxThrust, secIsp, kgpsecMassFlowRate, g, ...
    radEngineTheta, radEngineTiltAngle)
% calculate_thrust_vec - Computes the 3D thrust vector for a single rocket engine
%
% This function calculates the thrust vector based on the engine's mass flow
% rate and specific impulse. It applies a physical saturation limit to ensure
% the commanded thrust does not exceed the engine's maximum capability.
%
% Assumptions: 
% (1) The thrust force pushes the hovercraft *upwards*, hence the Z
%       component is negative.
% (2) The X and Y components point *inwards* towards the center of mass
%       (Canting in). If the engines point outwards, remove the minus sign
%       from the first two lines).
%
% Inputs:
%   newtonMaxThrust    - Maximum physically possible thrust of the engine [N]
%   secIsp             - Specific impulse of the engine [sec]
%   kgpsecMassFlowRate - Current mass flow rate commanded by the throttle [kg/s]
%   g                  - Standard Earth gravity for Isp conversion (9.80665 m/s^2)
%   radEngineTheta     - Azimuth angle of the engine around the Z-axis [rad]
%   radEngineTiltAngle - Cant angle of the engine relative to the Z-axis [rad]
%
% Outputs:
%   thrust_vec         - A 3-element column vector [Tx; Ty; Tz] representing
%                        the thrust force acting on the vehicle body [N].
%
%% Calculate the components of the direction vector (unit vector)
dx = -sin(radEngineTiltAngle) * cos(radEngineTheta);
dy = -sin(radEngineTiltAngle) * sin(radEngineTheta);
dz = -cos(radEngineTiltAngle);

%% Create the column vector
thrust_dir = [dx; dy; dz];

%% Calculate the theoretical thrust vector
% Based on the rocket equation: Thrust = Isp * dm/dt * g0
thrust_vec = secIsp * kgpsecMassFlowRate * g * thrust_dir;

%% Apply physical thrust saturation
% If the commanded mass flow rate results in a thrust higher than the
% engine's maximum limit, scale the vector magnitude down to the maximum
% while preserving the directional geometry.
if norm(thrust_vec) >= newtonMaxThrust
    thrust_vec = (thrust_vec / norm(thrust_vec)) * newtonMaxThrust;
end

end