function [ Cg ] = rocketTrajectoryCenterGravity(M)

x = load('data.mat', 'solidRocketSimCDh01');
mach = x.solidRocketSimCDh01{:,1};
[mach, index] = unique(mach);
Cgpts = x.solidRocketSimCDh01{:,4}(index);
points = [mach Cgpts];
points(1,1) = 0;
points(1,2) = 0;
Cg = interp1(points(:,1), points(:,2), M);
end
