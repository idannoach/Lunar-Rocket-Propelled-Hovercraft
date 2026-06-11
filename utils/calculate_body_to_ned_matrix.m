function R_b2n = calculate_body_to_ned_matrix(phi, theta, psi)
% calculate_body_to_ned_matrix - Computes the transformation matrix from 
%                                the Body frame to the local NED frame.
%
% This function uses the standard aerospace Z-Y-X Euler angle sequence:
% 1. Yaw (psi) around the Z-axis
% 2. Pitch (theta) around the Y-axis
% 3. Roll (phi) around the X-axis
%
% Inputs:
%   phi   - Roll angle [radians]
%   theta - Pitch angle [radians]
%   psi   - Yaw (Heading) angle [radians]
%
% Output:
%   R_b2n - 3x3 Direction Cosine Matrix (DCM) transforming a vector 
%           from the Body frame to the NED frame: V_ned = R_b2n * V_body

% Pre-compute sine and cosine of the angles for efficiency
c_phi = cos(phi);
s_phi = sin(phi);

c_theta = cos(theta);
s_theta = sin(theta);

c_psi = cos(psi);
s_psi = sin(psi);

% Compute the elements of the Body-to-NED transformation matrix
% Derived from: R_b2n = R_z(psi) * R_y(theta) * R_x(phi)

R11 = c_theta * c_psi;
R12 = s_phi * s_theta * c_psi - c_phi * s_psi;
R13 = c_phi * s_theta * c_psi + s_phi * s_psi;

R21 = c_theta * s_psi;
R22 = s_phi * s_theta * s_psi + c_phi * c_psi;
R23 = c_phi * s_theta * s_psi - s_phi * c_psi;

R31 = -s_theta;
R32 = s_phi * c_theta;
R33 = c_phi * c_theta;

% Assemble the 3x3 matrix
R_b2n = [R11, R12, R13;
    R21, R22, R23;
    R31, R32, R33];
end