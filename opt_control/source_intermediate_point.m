function constraints = source_intermediate_point(states, controls)
% source_intermediate_point - FALCON.m point constraint forcing the
%                              trajectory's NWU position to lie within a
%                              small tolerance box of the intermediate
%                              waypoint selected by
%                              gnc/guidance/select_intermediate_point.m,
%                              evaluated at that waypoint's normalized
%                              time (see main_falcon_descent.m). Only
%                              added when mission_parameters.bUseIntermediatePoint
%                              is true.

constraints = states(1:3);

end
