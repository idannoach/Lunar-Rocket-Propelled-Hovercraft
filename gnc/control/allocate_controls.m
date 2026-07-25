function throttles = allocate_controls(desired_Fz, desired_My, B_pinv_sym)
    % Map desired Fz and My to the 4 symmetric channels
    nu = [desired_Fz; desired_My];
    sym_throttles_ideal = B_pinv_sym * nu;

    % Unpack the 4 channels back to the 6 physical engines
    % Mapping: 1=Front, 2=FL, 3=BL, 4=Back, 5=BR, 6=FR
    throttles = zeros(6, 1);
    
    throttles(1) = sym_throttles_ideal(1); % Engine 1: Front
    throttles(2) = sym_throttles_ideal(2); % Engine 2: Front-Left (Ch 2)
    throttles(3) = sym_throttles_ideal(3); % Engine 3: Back-Left (Ch 3)
    throttles(4) = sym_throttles_ideal(4); % Engine 4: Back
    throttles(5) = sym_throttles_ideal(3); % Engine 5: Back-Right (Ch 3)
    throttles(6) = sym_throttles_ideal(2); % Engine 6: Front-Right (Ch 2)

    % Clamp to [0, 1]: prevents negative throttles (which invert thrust direction
    % and cause mass to increase in the engine dynamics model) and over-unity
    % commands that bypass the physical saturation logic.
    throttles = max(0, min(1, throttles));
end