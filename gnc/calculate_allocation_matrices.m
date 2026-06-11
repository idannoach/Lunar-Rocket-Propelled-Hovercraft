function hovercraft_parameters = calculate_allocation_matrices(hovercraft_parameters)
    % Constants
    newtonMaxThrust = hovercraft_parameters.newtonMaxThrust;
    radTiltAngle = deg2rad(hovercraft_parameters.degThrustRadialAngle);
    delta_z = hovercraft_parameters.mEngineAxialPos - hovercraft_parameters.mCG(3); 
    R = hovercraft_parameters.mEngineRadialPos;

    % Coefficients
    Cz = newtonMaxThrust * -cos(radTiltAngle);
    C_arm = R * cos(radTiltAngle) - delta_z * sin(radTiltAngle);
    Cm = newtonMaxThrust * C_arm;

    % We map 4 symmetric control channels strictly to [Fz; My]
    % Ch 1: Engine 1 (Front, +X)
    % Ch 2: Engines 2 & 6 (Front-Left & Front-Right)
    % Ch 3: Engines 3 & 5 (Back-Left & Back-Right)
    % Ch 4: Engine 4 (Back, -X)
    B_sym = zeros(2, 4);

    % Channel 1 (Engine 1: Front)
    B_sym(1, 1) = Cz;
    B_sym(2, 1) = Cm * cos(0); 
    
    % Channel 2 (Engines 2 & 6: Front-Left & Front-Right)
    B_sym(1, 2) = 2 * Cz;
    B_sym(2, 2) = Cm * cos(deg2rad(60)) + Cm * cos(deg2rad(300)); 
    
    % Channel 3 (Engines 3 & 5: Back-Left & Back-Right)
    B_sym(1, 3) = 2 * Cz;
    B_sym(2, 3) = Cm * cos(deg2rad(120)) + Cm * cos(deg2rad(240)); 
    
    % Channel 4 (Engine 4: Back)
    B_sym(1, 4) = Cz;
    B_sym(2, 4) = Cm * cos(deg2rad(180)); 

    % Calculate the Pseudo-Inverse (Rank 2 is perfectly stable)
    hovercraft_parameters.B_pinv = pinv(B_sym);

end