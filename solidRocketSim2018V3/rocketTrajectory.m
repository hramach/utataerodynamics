function [] = rocketTrajectory( engineType,Pitch0,mp,h0)
%rocketTrajectory Calculates and displays the trajectory and other
%variables during a solid rocket flight.
%   Input: engineType (string): 'L1300', 'M1800', 'N1000','O3400','N2500'
%          Pitch0: launch angle from vertical (deg)
%          mp: payload and structural mass (kg)
%          h0: initial altitude;
%   Ouput: Graphs of variables

clc
close all

[mo, mf, tb, f] = engineParameters(engineType) ;

DeltaT = 0.1;           %Time increment after powered flight
tsim = 1000;

g = 9.80665;            %Gravitational constant
A = 0.015;              %Frontal area
dryMass = 30.65;        %Changed from 13.9 based on mass budget, 2018-12-01
payloadMass = 0;
fullMass = mo + payloadMass + dryMass;
gamma = 1.4;
R = 287.0531;

%Allocate arrays
t = zeros(1,tsim);
tq = 0:0.01:tb;

Fx = zeros(1,tsim);     %total force in x direction
Fy = zeros(1,tsim);     %total force in y direction
Ft = zeros(1,tsim);     %thrust

W = zeros(1,tsim);      %gravitational force
Fd = zeros(1,tsim);     %drag force
Cd = zeros(1,tsim);     %Drag coefficient

Pitch = zeros(1,tsim);  %Pitch angle
FPTheta = zeros(1,tsim);  %Flight path angle
Mass = zeros(1,tsim);   %Vehicle mass (kg)

Inertia = zeros(1, tsim); %Moment of inertia (kg m^2)
Cp = zeros(1, tsim);   %Center of pressure (m from tip)
Cg = zeros(1, tsim);   %Center of gravity (m from tip)

Moment = zeros(1, tsim); %Overall moment on rocket (N m)

Ax = zeros(1,tsim);     %acceleration in x direction
Ay = zeros(1,tsim);     %acceleration in y direction
Atot = zeros(1,tsim);   %Total Acceleration
Alpha = zeros(1,tsim);  %Angular acceleration

Vx = zeros(1,tsim);     %velocity in x direction
Vy = zeros(1,tsim);     %velocity in y direction
Vtot = zeros(1,tsim);   %Total speed
Omega = zeros(1, tsim); %Angular velocity

x = zeros(1,tsim);      %position in x direction
y = zeros(1,tsim);      %position in y direction

Mach = zeros(1,tsim);   %Flight Mach number
Q = zeros(1,tsim);      %Dynamic pressure
P = zeros(1,tsim);      %Pressure
T = zeros(1,tsim);      %Temperature
Rho = zeros(1,tsim);    %Density

%Initial Conditions
engineMode = 1;         %Engine is initially ON
Pitch(1) = 90 - Pitch0;
AofA(1) = 90 - Pitch0;
Vx(1) = 0;
Vy(1) = 0;
Omega(1) = 0;
x(1) = 0;
y(1) = h0;
[ P(1), T(1), Rho(1) ] = standardAtmosphere( h0 );
Mass(1) = fullMass;

%Thrust Interpolation
fInterp = interp1(f(1,:),f(2,:),tq);

for i = 2:(tb/DeltaT)+1
       t(i) = t(i-1)+ DeltaT;
       Mass(i) = (-mf/tb)*t(i) + Mass(1);
       
       %Mach Number and Dynamic Pressure Calculation
       [ P(i-1), T(i-1), Rho(i-1) ] = standardAtmosphere( y(i-1) );
       Atot(i-1) = sqrt(Ax(i-1)^2+Ay(i-1)^2);
       Vtot(i-1) = sqrt(Vx(i-1)^2+Vy(i-1)^2);
       Mach(i-1) = Vtot(i-1)/sqrt(gamma*R*T(i-1));
       Q(i-1) = 0.5*Rho(i-1)*Vtot(i-1)^2;
       
       %Force Calculation
       Cd(i-1) = rocketTrajectoryDragCoefficient(y(i-1),Mach(i-1),engineMode);
       % Cd(i-1) = rocketTrajectoryDragCoefficient(Mach(i-1));
       Fd(i) = 0.5*Cd(i-1)*Rho(i-1)*A*(Vx(i-1)^2+Vy(i-1)^2);
       Ft(i) = interp1(tq,fInterp,t(i),'pchip');
       Fx(i) = Ft(i)*cosd(Pitch(i-1)) - Fd(i)*cosd(FPTheta(i-1)); 
       W(i) = Mass(i)*g;
       Fy(i) = Ft(i)*sind(Pitch(i-1)) - Fd(i)*sind(FPTheta(i-1)) - W(i);
       
       %Moment Calculation
       Cg(i-1) = rocketTrajectoryCenterGravity(Mach(i-1)) / 100;
       Cp(i-1) = rocketTrajectoryCenterPressure(Mach(i-1)) / 100;
       Inertia(i) = rocketTrajectoryMomentOfInertia(Cg(i-1), fullMass - Mass(i));
       Moment(i) = -Fd(i) * (Cg(i-1) - Cp(i-1));
       
       if Fy(i) <= 0 && i < 10
           Fy(i) = 0;
           Fx(i) = 0;
       end
       
       %Acceleration Calculation
       Ax(i) = Fx(i)/Mass(i);
       Ay(i) = Fy(i)/Mass(i);
       Alpha(i) = Moment(i) / Inertia(i);
       
       %Velocity Calculation
       Vx(i) = Vx(i-1)+Ax(i)*(t(i)-t(i-1));
       Vy(i) = Vy(i-1)+Ay(i)*(t(i)-t(i-1));
       Omega(i) = Omega(i-1) + Alpha(i) * (t(i) - t(i-1));
       
       %Position Calculation
       x(i) = x(i-1)+Vx(i)*(t(i)-t(i-1));
       y(i) = y(i-1)+Vy(i)*(t(i)-t(i-1));
       Pitch(i) = Pitch(i-1) + Omega(i)*(t(i) - t(i-1));
       
       %Flight Path Angle
       if (Fy(i) <= 0 && i < 10) || y(i) < 1
           FPTheta(i) = FPTheta(i-1);
       else
           FPTheta(i) =  atand(Vy(i)/Vx(i));
       end
end

iBurntOut = i;
i = (tb/DeltaT)+1;
engineMode = 0; %Engine turns off after burn

while y(i) > 0.01
    i = i+1;
    Mass(i) =  Mass((tb/DeltaT)+1);
    t(i) = t(i-1)+ DeltaT;
    
    %Mach Number and Dynamic Pressure Calculation
    [ P(i-1), T(i-1), Rho(i-1) ] = standardAtmosphere( y(i-1) );
    Vtot(i-1) = sqrt(Vx(i-1)^2+Vy(i-1)^2);
    Atot(i-1) = -sqrt(Ax(i-1)^2+Ay(i-1)^2);
    Mach(i-1) = Vtot(i-1)/sqrt(gamma*R*T(i-1));
    Q(i-1) = 0.5*Rho(i-1)*Vtot(i-1)^2;
       
    %Force Calculation
    Cd(i-1) = rocketTrajectoryDragCoefficient(y(i-1),Mach(i-1),engineMode);
    % Cd(i-1) = rocketTrajectoryDragCoefficient(Mach(i-1));
    Fd(i) = 0.5*Cd(i-1)*Rho(i-1)*A*(Vx(i-1)^2+Vy(i-1)^2);
    Ft(i) = interp1(tq,fInterp,t(i),'pchip');
    Fx(i) = Ft(i)*cosd(Pitch(i-1)) - Fd(i)*cosd(FPTheta(i-1)); 
    W(i) = Mass(i)*g;
    Fy(i) = Ft(i)*sind(Pitch(i-1)) - Fd(i)*sind(FPTheta(i-1)) - W(i);
    
    %Moment Calculation
    Cg(i-1) = rocketTrajectoryCenterGravity(Mach(i-1)) / 100;
    Cp(i-1) = rocketTrajectoryCenterPressure(Mach(i-1)) / 100;
    Inertia(i) = rocketTrajectoryMomentOfInertia(Cg(i-1), fullMass - Mass(i));
    Moment(i) = -Fd(i) * (Cg(i-1) - Cp(i-1));
    
    %Acceleration Calculation
    Ax(i) = Fx(i)/Mass(i);
    Ay(i) = Fy(i)/Mass(i);
    Alpha(i) = Moment(i) / Inertia(i);
    
    %Velocity Calculation
    Vx(i) = Vx(i-1)+Ax(i)*(t(i)-t(i-1));
    Vy(i) = Vy(i-1)+Ay(i)*(t(i)-t(i-1));
    Omega(i) = Omega(i-1) + Alpha(i) * (t(i) - t(i-1));
       
    %Position Calculation
    x(i) = x(i-1)+Vx(i)*(t(i)-t(i-1));
    y(i) = y(i-1)+Vy(i)*(t(i)-t(i-1));
    Pitch(i) = Pitch(i-1) + Omega(i)*(t(i) - t(i-1));
       
    %Flight Path Angle
    FPTheta(i) =  atand(Vy(i)/Vx(i)); 
end

figure('units','normalized','outerposition',[0 0 1 1]) % Maximize plot window

% Figure 1
subplot(3,3,1)
plot(t(1:i),x(1:i),t(1:i),y(1:i),'linewidth',2); 
hold on;
plot(t(iBurntOut), y(iBurntOut), 'r*');
grid on
%xlim([0 max(x)+50]);
ylim([0 max(y)+100]);
legend('Downrange Distance','Altitude')
ylabel({'Distance (m)'});
xlabel({'Time(s)'});
title({'Trajectory'});

% Figure 2
subplot(3,3,2)
plot(t(1:i),Vtot(1:i),'linewidth',2);
hold on;
plot(t(iBurntOut), Vtot(iBurntOut), 'r*');
grid on
xlabel({'Time (s)'});
ylabel({'V (m/s)'});
title({'Axial Velocity'});

% Figure 3
subplot(3,3,3)
plot(t(1:i),Atot(1:i),'linewidth',2);
hold on;
plot(t(iBurntOut), Atot(iBurntOut), 'r*');
grid on
xlabel({'Time (s)'});
ylabel({'A (m/s^2)'});
title({'Axial Acceleration'});

% Figure 4
subplot(3,3,4)
plot(t(1:i-1),Cd(1:i-1),'linewidth',2);
hold on;
plot(t(iBurntOut), Cd(iBurntOut), 'r*');
grid on
xlabel({'Time (s)'});
ylabel({'Cd'});
title({'Drag Coefficient vs. Time'});

% Figure 5
subplot(3,3,5)
plot(t(1:i),Mach(1:i),'linewidth',2);
hold on;
plot(t(iBurntOut), Mach(iBurntOut), 'r*');
grid on
xlabel({'Time (s)'});
ylabel({'Mach Number'});
title({'Mach Number vs. Time'});

% Figure 6
subplot(3,3,6)
plot(Mach(1:i-1),Cd(1:i-1),'linewidth',2);
hold on;
plot(Mach(iBurntOut), Cd(iBurntOut), 'r*');
grid on
xlabel({'Mach Number'});
ylabel({'Cd'});
title({'Drag Coefficient vs. Mach Number'});


% Figure 7
subplot(3,3,7)
plot(t(1:i),Fd(1:i),'linewidth',2);
hold on;
plot(t(iBurntOut), Fd(iBurntOut), 'r*');
grid on
xlabel({'Time (s)'});
ylabel({'Drag (N)'});
title({'Drag Force'});

% Figure 8
subplot(3,3,8)
plot(t(1:i),Ft(1:i),'linewidth',2);
hold on;
plot(t(iBurntOut), Ft(iBurntOut), 'r*');
grid on
xlim([0 tb]);
xlabel({'Time (s)'});
ylabel({'Thrust (N)'});
title({'Thrust'});

% Figure 9
subplot(3,3,9)
plot(t(1:i),Q(1:i),'linewidth',2);
hold on;
plot(t(iBurntOut), Q(iBurntOut), 'r*');
grid on
xlabel({'Time (s)'});
ylabel({'Dynamic Pressure (Pa)'});
title({'Dynamic Pressure'});

end