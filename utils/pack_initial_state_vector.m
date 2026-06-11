function x0 = pack_initial_state_vector(mission_parameters, hovercraft_parameters)

pos_0   = mission_parameters.mInitialPosition;      % Initial position (NED) [m]
vel_0   = mission_parameters.mpsInitialVelocity;    % Initial velocity (Body frame) [m/s]
euler_0 = mission_parameters.radEulerAngles;        % Initial attitude [phi; theta; psi] [rad]
rates_0 = mission_parameters.radpsecRates;          % Initial angular rates [p; q; r] [rad/s]
mass_0  = hovercraft_parameters.kgTotalMass;        % Initial mass [kg]
throttles_0 = zeros(6, 1);                          % Engines

% Pack the state vector
x0 = [pos_0; vel_0; euler_0; rates_0; mass_0; throttles_0];

end