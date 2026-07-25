function x0 = plant_module_point_mass(dt, x0, T_nwu, global_parameters, hovercraft_parameters)
% plant_module_point_mass - Integrates the 7-state point-mass dynamics
%                            over one controller period under a constant
%                            (zero-order-hold) commanded thrust vector.
%
% No engine lag/PWM/MIB/transport-delay modeling here - those are
% hardware-specific to the 6-DOF actuator chain (see plant_module.m) and
% have no equivalent in a net-thrust-vector point-mass model.

N_sub = 4;
h = dt / N_sub;

for i = 1:N_sub
    k1 = calculate_point_mass_dynamics(x0,          T_nwu, global_parameters, hovercraft_parameters);
    k2 = calculate_point_mass_dynamics(x0 + h/2*k1,  T_nwu, global_parameters, hovercraft_parameters);
    k3 = calculate_point_mass_dynamics(x0 + h/2*k2,  T_nwu, global_parameters, hovercraft_parameters);
    k4 = calculate_point_mass_dynamics(x0 + h*k3,    T_nwu, global_parameters, hovercraft_parameters);
    x0 = x0 + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

% Clamp mass at dry mass to prevent numerical overshoot below physical minimum
m_dry = hovercraft_parameters.kgTotalMass - hovercraft_parameters.kgFuelMass;
x0(7) = max(m_dry, x0(7));

end
