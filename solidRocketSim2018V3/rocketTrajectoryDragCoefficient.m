function [ Cd ] = rocketTrajectoryDragCoefficient(h, M, engineMode)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

x = load('data.mat', 'solidRocketSimCDh01');
mach = x.solidRocketSimCDh01{:, 1};
[mach, index] = unique(mach);
Cdpts = x.solidRocketSimCDh01{:, 2}(index);
points = [mach Cdpts];
points(1,1) = 0;
points(1,2) = 0;
Cd = interp1(points(:, 1), points(:, 2), M);

%  x = load('data.mat','solidRocketSimCDh0','solidRocketSimCDh18'); 
%  if engineMode == 1
%       Cd0 = interp1(x.solidRocketSimCDh0{:,1},x.solidRocketSimCDh0{:,3},M);
%       Cd18 = interp1(x.solidRocketSimCDh18{:,1},x.solidRocketSimCDh18{:,3},M);
%       
%       Cd = interp1([0;18288],[Cd0;Cd18],h);
%   else if engineMode == 0
%       Cd0 = interp1(x.solidRocketSimCDh0{:,1},x.solidRocketSimCDh0{:,2},M);
%       Cd18 = interp1(x.solidRocketSimCDh18{:,1},x.solidRocketSimCDh18{:,2},M);   
%       
%       Cd = interp1([0;18288],[Cd0;Cd18],h);
% end

% x = load('data.mat','solidRocketSimCDh01');
%  
% if engineMode == 1
%      Cd0 = interp1(x.solidRocketSimCDh01{:,1},x.solidRocketSimCDh01{:,2},M);
%      Cd18 = interp1(x.solidRocketSimCDh01{:,1},x.solidRocketSimCDh01{:,2},M);
%      
%      Cd = interp1([0;18288],[Cd0;Cd18],h);
%  else if engineMode == 0
%      Cd0 = interp1(x.solidRocketSimCDh01{:,1},x.solidRocketSimCDh01{:,2},M);
%      Cd18 = interp1(x.solidRocketSimCDh01{:,1},x.solidRocketSimCDh01{:,2},M);   
%      
%      Cd = interp1([0;18288],[Cd0;Cd18],h);
% end

end

