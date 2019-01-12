function [ Cp ] = rocketTrajectoryCenterPressure(M)

x = load('data.mat', 'solidRocketSimCDh01');
mach = x.solidRocketSimCDh01{:,1};
[mach, index] = unique(mach);
Cppts = x.solidRocketSimCDh01{:,3}(index);
points = [mach Cppts];
points(1,1) = 0;
points(1,2) = 0;
Cp = interp1(points(:,1), points(:,2), M);
end
