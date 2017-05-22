%% Run EKF prediction step based on ROS commands
% Comments

% Running the plotting functions in Matlab to show the robot will not work !!
% The reason is the delay during the plotting which misses the ros messages 
% being sent. 


%% prep workspace
clear variables
close all
clc

%% initial position of the vehicle
%init posterior and covariance
xEst = [0, 0, 0]';
covEst = diag([0.01, 0.01, 0.001]);

%control for the velocity model
v = 0.0;
w = 0.0;

%sample time
dt = 0.1;

% alpha 1 -> v based on v
% alpha 2 -> v based on v
% alpha 3 -> w based on v
% alpha 4 -> w based on w
alpha = [10, 0.1, 0.1, 1];



if(robotics.ros.internal.Global.isNodeActive)
    sprintf('ROS master initialized')
else
    rosinit;
    sprintf('Initializing ROS master')
end

N = robotics.ros.Node('matlabNode');
sub = rossubscriber('/cmd_vel_mux/input/teleop');

nSteps = 1000;
v=0;
w=0;

for i = 1:nSteps
    
    msg = receive(sub);
    v = msg.Linear.X;
    w = msg.Angular.Z;
    [xPred, covPred] = ekfPredict([v,w],xEst, covEst, alpha, dt);
    
    xEst = xPred
    n = norm(covPred, inf);
    covEst = covPred
end