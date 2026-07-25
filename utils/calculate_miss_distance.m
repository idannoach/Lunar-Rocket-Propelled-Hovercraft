function miss_distance = calculate_miss_distance(x_out, target_pos)
% calculate_miss_distance - Euclidean distance between the final position
%                            in a state history and a target waypoint.

pos_f = x_out(end, 1:3);

if iscolumn(target_pos)
    target_pos = target_pos';
end

miss_distance = norm(pos_f - target_pos);

end
