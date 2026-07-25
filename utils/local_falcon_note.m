function note = local_falcon_note(falcon_result)
% Free-text convergence note passed through to log_results so a
% non-converged FALCON.m benchmark (see opt_control/main_falcon_descent.m)
% is flagged in its log file too, not just on-screen/in the plots.
if isfield(falcon_result, 'converged') && ~falcon_result.converged
    note = sprintf('FALCON.m/IPOPT did NOT converge (exit status: %s) - this is the solver''s last iterate, NOT a valid minimum-fuel solution.', ...
        falcon_result.exit_status);
else
    note = '';
end

end 