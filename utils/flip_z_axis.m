function v_flipped = flip_z_axis(v)
% flip_z_axis - Converts a 3-vector between this project's NWU (Z positive
%               up) frame and the guidance-law reference paper's frame
%               (Z positive down). Flips the sign of the Z-component only;
%               X (North) and Y (West) are unchanged since they already
%               align with the paper's downrange/crossrange axes.
%
% Involutory: flip_z_axis(flip_z_axis(v)) == v. Used in both directions
% (NWU -> paper and paper -> NWU) for velocity/acceleration/gravity
% vectors. For positions, translate to be target-relative first.

v_flipped = [v(1); v(2); -v(3)];

end