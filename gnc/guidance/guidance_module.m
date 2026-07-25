function [cmd_accel_n, cmd_accel_w, cmd_accel_u] = guidance_module(t, current_state, global_parameters, mission_parameters, is6DOF)

cmd_accel_nwu = optimal_LQ_guidance_with_intermediate_point(t, current_state, global_parameters, mission_parameters, is6DOF);

cmd_accel_n = cmd_accel_nwu(1);
cmd_accel_w = cmd_accel_nwu(2);
cmd_accel_u = cmd_accel_nwu(3);

end

