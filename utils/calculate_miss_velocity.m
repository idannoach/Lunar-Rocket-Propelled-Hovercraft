function miss_velocity = calculate_miss_velocity(x_out)
% calculate_miss_velocity - Speed magnitude of the final velocity in a
%                            state history. Frame-agnostic (norm is
%                            invariant under rotation), so this works for
%                            both NWU-frame point-mass histories and
%                            body-frame 6-DOF histories alike.

vel_f = x_out(end, 4:6);
miss_velocity = norm(vel_f);

end
