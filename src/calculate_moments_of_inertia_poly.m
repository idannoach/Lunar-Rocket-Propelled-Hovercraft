function hovercraft_parameters = calculate_moments_of_inertia_poly(hovercraft_parameters)
% calculate_moments_of_inertia_poly - Calculates the linear polynomial
% coefficients for the moments of inertia as a function of vehicle mass.
%
% Input:
%   hovercraft_parameters - A struct containing the hovercraft parameters.
%
% Outputs:
%   hovercraft_parameters - hovercraft_parameters containing Ixx_poly, 
%                           Iyy_poly, Izz_poly [slope, intercept]

%% Extract mass boundary conditions
m_full = hovercraft_parameters.kgTotalMass;
m_empty = hovercraft_parameters.kgTotalMass - hovercraft_parameters.kgFuelMass;

% Mass difference for the slope calculation
delta_m = m_full - m_empty;

%% Calculate Polynomial for Ixx (Roll)
slope_x = (hovercraft_parameters.kgsqmIxx.max - hovercraft_parameters.kgsqmIxx.min) / delta_m;
intercept_x = hovercraft_parameters.kgsqmIxx.max - slope_x * m_full;
Ixx_poly = [slope_x, intercept_x];

%% Calculate Polynomial for Iyy (Pitch)
slope_y = (hovercraft_parameters.kgsqmIyy.max - hovercraft_parameters.kgsqmIyy.min) / delta_m;
intercept_y = hovercraft_parameters.kgsqmIyy.max - slope_y * m_full;
Iyy_poly = [slope_y, intercept_y];

%% Calculate Polynomial for Izz (Yaw)
slope_z = (hovercraft_parameters.kgsqmIzz.max - hovercraft_parameters.kgsqmIzz.min) / delta_m;
intercept_z = hovercraft_parameters.kgsqmIzz.max - slope_z * m_full;
Izz_poly = [slope_z, intercept_z];

%% Add to hovercraft parameters
hovercraft_parameters.Ixx_poly = Ixx_poly;
hovercraft_parameters.Iyy_poly = Iyy_poly;
hovercraft_parameters.Izz_poly = Izz_poly;

end