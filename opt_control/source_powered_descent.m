function [states_dot] = source_powered_descent(states, controls)
% source_powered_descent - FALCON.m dynamics model: 7-state point-mass
%                           powered descent in the NWU inertial frame.
%
% States:   [X; Y; Z; Vx; Vy; Vz; m]   (NWU position/velocity, mass)
% Controls: [Tx; Ty; Tz]               (net NWU thrust vector, unbounded)
%
% This mirrors src/calculate_point_mass_dynamics.m exactly, so the
% closed-loop point-mass simulation (is6DOF=false) and this open-loop
% FALCON.m benchmark solve the same equations of motion. FALCON.m dynamics
% functions must have the fixed (states, controls) signature - no extra
% struct arguments - so the constants below are hardcoded and must be kept
% in sync with config/GP.m (g_lunar) and config/hovercraft.json
% (secIsp, g0) if those ever change.

Vx = states(4); Vy = states(5); Vz = states(6);
m = states(7);

Tx = controls(1); Ty = controls(2); Tz = controls(3);

g_lunar = 1.625;   % config/GP.m: g_lunar
Isp     = 285;     % config/hovercraft.json: secIsp
g0      = 9.80665; % config/hovercraft.json / config/GP.m: g0 (Isp -> mdot)

T_mag = sqrt(Tx^2 + Ty^2 + Tz^2 + 1e-9); % epsilon avoids a non-smooth sqrt(0)
mdot  = -T_mag / (Isp * g0);

Xdot  = Vx;  Ydot  = Vy;  Zdot  = Vz;
Vxdot = Tx / m;
Vydot = Ty / m;
Vzdot = Tz / m - g_lunar; % Z is up in NWU, gravity acts -Z

states_dot = [Xdot; Ydot; Zdot; Vxdot; Vydot; Vzdot; mdot];

end
