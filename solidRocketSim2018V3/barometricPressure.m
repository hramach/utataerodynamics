function [P] = barometricPressure( h )
%barometricPressure: Calculates pressure as a function of altitude
%using standard atmosphere.
%   From 0-11000m uses equation P = Pb0 * (Tb0 / (Tb0 + Lb0 * h))^(g0/(R*Lb0))
%   with Pb0, Tb0 & Lb0.
%   From 11000m-25000m uses equation P = Pb1 * exp(-g0 * (h - hb1) / (R * Tb1))
%   with Pb1, Tb1, & Lb1 = 0.
%   Above 25000m uses equation P = Pb2 * (Tb2 / (Tb2 + Lb2 * h))^(g0/(R*Lb2))
%   with Pb2, Tb2 & Lb2.

Pb0 = 101325;           %Static pressure at sea level (Pa)
Tb0 = 288.15;           %Standard temperature at sea level (K)
Lb0 = -0.0065;          %Standard temperature lapse rate (K/m)

Pb1 = 22632.10;         %Static pressure at 11000m (Pa)
Tb1 = 216.65;           %Standard temperature at 11000m (K)
hb1 = 11000;

Pb2 = 2488.63;           %Static pressure at 25000m (Pa)
Tb2 = 216.65;
Lb2 = 0.003;

g0 = 9.80665;           %Gravitational acceleration
R = 287.0531;           %Universal gas constant

if h <= 11000
    P = Pb0 * (Tb0 / (Tb0 + Lb0 * h))^(g0/(R*Lb0));
elseif h <= 25000 && h > 11000
    P = Pb1 * exp(-g0 * (h - hb1) / (R * Tb1));
else
    P = Pb2 * (Tb2 / (Tb2 + Lb2 * h))^(g0/(R*Lb2));
end

end

