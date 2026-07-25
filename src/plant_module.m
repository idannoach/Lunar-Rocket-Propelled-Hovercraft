function x0 = plant_module(dt, x0, throttles, global_parameters, hovercraft_parameters, is6DOF)

% Engine time constant
tau = (hovercraft_parameters.msecResponseTo90PctThrustTime / 1000) / 2.3;

% 4 RK4 sub-steps per controller period
N_sub = 4;
h = dt / N_sub;

% The Physics
x_rb             = x0(1:13);
throttles_0      = x0(14:19);
delta_throttles  = throttles_0 - throttles; % decaying transient

% RK4 for the 13 rigid-body states.
% The engine throttle is not a state here — it is evaluated analytically
% at each stage time using the exact first-order-lag solution, so the
% stiff engine eigenvalues never enter the integrator.
for i = 1:N_sub
    t0  = (i-1) * h;
    th0 = throttles + delta_throttles * exp(-t0        / tau);
    thm = throttles + delta_throttles * exp(-(t0+h/2)  / tau);
    th1 = throttles + delta_throttles * exp(-(t0+h)    / tau);

    k1 = calculate_rigid_body_dynamics(x_rb,           th0, global_parameters, hovercraft_parameters);
    k2 = calculate_rigid_body_dynamics(x_rb + h/2*k1,  thm, global_parameters, hovercraft_parameters);
    k3 = calculate_rigid_body_dynamics(x_rb + h/2*k2,  thm, global_parameters, hovercraft_parameters);
    k4 = calculate_rigid_body_dynamics(x_rb + h*k3,    th1, global_parameters, hovercraft_parameters);
    x_rb = x_rb + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

% Clamp mass at dry mass to prevent numerical overshoot below physical minimum
m_dry = hovercraft_parameters.kgTotalMass - hovercraft_parameters.kgFuelMass;
x_rb(13) = max(m_dry, x_rb(13));

% Exact analytical update for engine lag states over the full dt
throttles_end = throttles + delta_throttles * exp(-dt / tau);

x0 = [x_rb; throttles_end];

end