function [ I ] = rocketTrajectoryMomentOfInertia(Cgrocket, propellantMass)

    %All masses in kg
    m = [2.132 3.547 1.324 0.127 8.64 propellantMass];
    %[nose cone, recovery bay, main bay, boatail, fin can, propellant]
    
    %All Cg in m and from tip of rocket
    Cgs = [0.536 0.948 1.61 2.61 2.08 2.04];
    %[nose cone, recovery bay, main bay, boatail, fin can, propellant]
    
    r = -1 * Cgs + Cgrocket;
    
    I = sum(m .* (r .^ 2));
end