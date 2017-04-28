function [xEst, covEst] = addNewFeature(xPred, covPred, z, iFeature, Q)
%addNewFeature(xEst, covEst) Summary of this function goes here
%   Detailed explanation goes here
global mappedLandFeatures;
%%
n = length(xPred);
x = xPred(1);
y = xPred(2);
theta = xPred(3);
range = z(1);
bearing = z(2);

% project the location of the landmark into the global frame
newX = x + range * cos(theta + bearing);
newY = y + range * sin(theta + bearing);
xEst = [xPred; newX; newY];
mappedLandFeatures(iFeature,:) = [newX, newY];

% How the covariance in the vehicle is related to the landmark 
jGx = [ 1   0   -range*sin(theta + bearing);
        0   1   range*cos(theta + bearing)];
    

% How measurement noise R_t is shaped into the (x,y) of the landmark
jGz = [ cos(theta + bearing) -range*sin(theta + bearing);
        sin(theta + bearing) range*cos(theta + bearing)];


M = [eye(n), zeros(n,2);% no use jacobian w.r.t vehicle
    jGx, zeros(2,n-3), jGz];

covEst = M * blkdiag(covPred,Q) * M';

end

