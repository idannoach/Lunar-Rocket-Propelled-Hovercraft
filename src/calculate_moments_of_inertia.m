function [Ixx, Iyy, Izz] = calculate_moments_of_inertia(kgMass, HP)
% calculate_moments_of_inertia - Computes the instantaneous moment of
%                                inertia tensor for the hovercraft.
%
% This function calculates the 3x3 inertia matrix based on the vehicle's
% current mass. It accounts for the mass depletion as fuel is consumed
% by using the linear polynomial relationships defined by the vehicle's
% empty and full states.
%
% Assumptions:
% (1) We assume the body axes (X, Y, Z) align with the vehicle's principal
%   axes of symmetry. Because of the symmetric placement of the 6 engines
%   and fuel tanks, the products of inertia (Ixy, Ixz, Iyz) are assumed
%   to be exactly zero, resulting in a strictly diagonal matrix.
%
% Inputs:
%   kgMass - The current instantaneous mass of the hovercraft [kg]
%   HP     - A struct containing the hovercraft parameters
%
% Output:
%   [Ixx, Iyy, Izz] - The moments of inertia [kg*m^2] at the given mass.

%% Evaluate the polynomials (I_current = slope * mass + intercept)
% to find the principal moments of inertia for the current time step.
Ixx = HP.Ixx_poly(1) * kgMass + HP.Ixx_poly(2);
Iyy = HP.Iyy_poly(1) * kgMass + HP.Iyy_poly(2);
Izz = HP.Izz_poly(1) * kgMass + HP.Izz_poly(2);

end