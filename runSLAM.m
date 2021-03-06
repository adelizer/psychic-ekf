%% Main file to run EKF SLAM
% Author: adel
%   ______   _  __  ______      _____   _                   __  __
%  |  ____| | |/ / |  ____|    / ____| | |          /\     |  \/  |
%  | |__    | ' /  | |__      | (___   | |         /  \    | \  / |
%  |  __|   |  <   |  __|      \___ \  | |        / /\ \   | |\/| |
%  | |____  | . \  | |         ____) | | |____   / ____ \  | |  | |
%  |______| |_|\_\ |_|        |_____/  |______| /_/    \_\ |_|  |_|
%
%% comments
% Building the main file structure [DONE]

% Verifying the velocity motion model. Comparing to the odom. motion model
% in Robotics package in MatLAB [DONE ?]

% The odometery motion model needs as input the pervious pose(s) and the
% current odometry which is the perviouse odom added to the different
% between two poses !!?

% Added the new feature handler [DONE]

% Implement EKF update [DONE]

% Handle unordered landmarks views [DONE]

% Handle multiple landmark detections [DONE]
%% prep workspace
clear variables
close all
clc

%% Initialize world and sensor
global landFeatures;
global laserSensorSettings;
global mappedLandFeatures;

% global to be used in gui callbacks
global v; %linear vel
global w; %angular vel

%laser sensor settings
laserSensorSettings.Bearing = 40; % Degrees
laserSensorSettings.Range = 4; % Meters

%Length of the experiment
nSteps = 1000;

%Number of landfeatures to add to map
nLandFeatures = 8;

%init landmarks
landFeatures = zeros(2,1,nLandFeatures);
landFeatures(:,:,1)=[2 2]';
landFeatures(:,:,2)=[-2 2]';
landFeatures(:,:,3)=[-2 -2]';
landFeatures(:,:,4)=[2 -2]';
landFeatures(:,:,5)=[2.5 0]';
landFeatures(:,:,6)=[-0.5 -2.5]';
landFeatures(:,:,7)=[0.5 2.5]';
landFeatures(:,:,8)=[-2.5 0.5]';
% landFeatures(:,:,4)=[1.5 2.5]';



% tempHandle = figure('name','Dummy');
drawRobotHandle = figure('WindowKeyPressFcn',@figureKeyPressFcn);
load('controls.mat')

%% Initialize variables
%what the robot has seen
mappedLandFeatures = NaN*zeros(nLandFeatures,2);

%initial position of the vehicle
xTrue = [0, 0, 0]';

%init posterior and covariance
xEst = xTrue;
covEst = diag([0.01, 0.01, 0.001]);

%control for the velocity model
v = 0.02;
w = 0.02;

%sample time
dt = 1;

% alpha 1 -> v based on v
% alpha 2 -> v based on v
% alpha 3 -> w based on v
% alpha 4 -> w based on w
alpha = [10, 0.1, 0.1, 1];

Q = diag([0.1, 0.01]);
observationNoiseFactor =  0.004;
processNoiseFactor = 0.004;


%odom model tryout
motionModel = robotics.OdometryMotionModel;
motionModel.Noise = alpha;
motionModel(xTrue', xTrue');
currentOdom = xTrue';
previousPose = xTrue';
currentPose = previousPose;



% writerObj = VideoWriter('SLAM.avi'); % Name it.
% writerObj.FrameRate = 25; % How many frames per second.
% open(writerObj);

for k = 1:nSteps
    %     v = old_v(k);
    %     w = old_w(k);
    controlNoise = randn(1,2)*processNoiseFactor;
    xTrue = moveReal(xTrue, [v, w] + controlNoise, dt);
    
    [z, iFeature] = getObservation(xTrue, observationNoiseFactor);
    
    %     currentOdom = currentOdom + xTrue - previousPose;
    %     currentPose = motionModel(previousPose, currentOdom);
    %     previousPose = currentPose;
    
    [xPred, covPred] = ekfPredict([v,w], xEst, covEst, alpha, dt);
    z = nan;
    if(~isnan(z))
        for i =1:size(z,1)
            if(~isnan(mappedLandFeatures(iFeature(i),1)))
                %update               
                [xEst, covEst] = ekfUpdate(xPred, covPred, z(i,:), iFeature(i), Q);
                xPred = xEst;
                covPred = covEst;
            else
                %new landmark
                [xEst, covEst] = addNewFeature(xPred, covPred, z(i,:), iFeature(i), Q);
                xPred = xEst;
                covPred = covEst;
            end
        end
    else
        %no measurements
        xEst = xPred;
        covEst = covPred;
    end
    
    drawRobot(xTrue, xEst, covEst(1:2,1:2),1);
    
    %log data
    log.xTrue(k,:) = xTrue;
    log.xEst(k,:) = xEst(1:3);
    log.currentPose(k,:) = currentPose;
    log.u(k,:) = [v,w,0];
    
    %     frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    %     writeVideo(writerObj, frame);
    
end % end of k forLoop

% close(writerObj); % Saves the movie.

figure('name', 'Compare odom & vel')
plot(log.xTrue(:,1), log.xTrue(:,2))
hold on
plot(log.xEst(:,1), log.xEst(:,2))
hold on
%plot(log.currentPose(:,1), log.currentPose(:,2))
grid on
legend('True','Vel','Odom')

figure('name', 'Error')
plot((sqrt((log.xTrue(:,1)-log.xEst(:,1)).^2 + (log.xTrue(:,2)-log.xEst(:,2)).^2 )))
