function x0 = pack_initial_state_vector_point_mass(mission_parameters, hovercraft_parameters)
% pack_initial_state_vector_point_mass - 7-state point-mass initial state
%                                         [pos(3) NWU; vel(3) NWU; mass].
%
% Unlike pack_initial_state_vector.m, velocity is NWU directly rather than
% body-frame: there are no attitude states in the point-mass model, so
% body and NWU frames coincide and no rotation is needed or possible.

pos_0  = mission_parameters.mInitialPosition;      % Initial position (NWU) [m]
vel_0  = mission_parameters.mpsInitialVelocity;    % Initial velocity (NWU) [m/s]
mass_0 = hovercraft_parameters.kgTotalMass;        % Initial mass [kg]

x0 = [pos_0; vel_0; mass_0];

end
