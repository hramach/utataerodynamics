function [ P, T, rho ] = standardAtmosphere( h )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
Tb0 = 288.15;           %Standard temperature at sea level (K)
Lb0 = -0.0065;          %Standard temperature lapse rate (K/m)
rho0 = 1.225;

Tb1 = 216.65;           %Standard temperature at 11000m & 20000m (K)

Lb2 = 0.003;
Tb2 = 141.94;

P = barometricPressure(h);

if h <= 11000
    T = Tb0 + Lb0*h; 
elseif h <= 25000 && h > 11000
    T = Tb1;
else
    T = Tb2 + Lb2*h;
end

rho = P/(287.058*T);


end

