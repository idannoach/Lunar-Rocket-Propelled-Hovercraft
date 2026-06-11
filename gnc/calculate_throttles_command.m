function [cmd_throttles, engine_on_timers] = calculate_throttles_command(dt, ideal_throttles, engine_on_timers, hovercraft_parameters)

minThrottle = hovercraft_parameters.pctThrottleRange(1)/100;
maxThrottle = hovercraft_parameters.pctThrottleRange(2)/100;

% The shortest possible time the engine can fire
secMinDt = hovercraft_parameters.newtonsecMinImpulseBit / hovercraft_parameters.newtonMaxThrust;

pwmDeadband = hovercraft_parameters.pctPwmDeadband / 100;

cmd_throttles = zeros(6,1);

for i = 1:6
    if engine_on_timers(i) > 0
        % REGIME 1: MIB Pulse Active
        % The engine MUST stay on for secMinDt. Allow it to throttle up if 
        % needed, but lock the minimum to the 40% hardware limit.
        cmd_throttles(i) = max(minThrottle, min(maxThrottle, ideal_throttles(i))); 

        engine_on_timers(i) = engine_on_timers(i) - dt;
        if engine_on_timers(i) <= 0
            engine_on_timers(i) = 0; % Pulse finished
        end
    else
        % Engine is free to switch states
        if ideal_throttles(i) >= minThrottle
            % REGIME 2: Continuous Throttling
            cmd_throttles(i) = min(maxThrottle, ideal_throttles(i));

        elseif ideal_throttles(i) > pwmDeadband
            % REGIME 3: PWM Pulse Triggered 
            % Fire at 100% to satisfy the 5 Ns MIB in 250ms.
            cmd_throttles(i) = 1.0; 
            engine_on_timers(i) = secMinDt - dt;

        else
            % REGIME 4: Engine OFF
            cmd_throttles(i) = 0.0; 
        end
    end
end

end