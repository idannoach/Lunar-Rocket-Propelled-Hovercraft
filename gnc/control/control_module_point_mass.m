function [T_nwu, fuel_exhausted] = control_module_point_mass(t_curr, x0, cmd_n, cmd_w, cmd_u, hovercraft_parameters, fuel_exhausted)
% control_module_point_mass - Converts the guidance acceleration command
%                              directly into a net NWU thrust vector.
%
% No attitude control, control allocation, PWM/MIB, or transport delay:
% the point-mass model has no attitude or individual-engine states to
% drive. Thrust is left unsaturated (unlike control_module.m's throttle
% clamp) to match the closed-form LQ law's own assumption of infinite
% control authority - see
% gnc/guidance/optimal_LQ_guidance_with_intermediate_point.m.

mass = x0(7);

% Fuel exhaustion guard: zero thrust once propellant is depleted
m_dry = hovercraft_parameters.kgTotalMass - hovercraft_parameters.kgFuelMass;
if mass <= m_dry
    T_nwu = zeros(3, 1);
    if ~fuel_exhausted
        fuel_exhausted = true;
        warning('run_simulation:FuelExhausted', ...
            'Fuel exhausted at t = %.2f s (mass %.3f kg <= dry mass %.3f kg). Thrust disabled.', ...
            t_curr, mass, m_dry);
    end
else
    cmd_accel_nwu = [cmd_n; cmd_w; cmd_u];
    T_nwu = mass * cmd_accel_nwu;
end

end
