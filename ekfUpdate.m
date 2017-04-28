function [ xEst, covEst ] = ekfUpdate(xPred, covPred, z, iFeature, Q)
%ekfUpdate(xPred, covPred, z, iFeature) Summary of this function goes here
%   Detailed explanation goes here

n = length(xPred);
x = xPred(1);
y = xPred(2);
theta = xPred(3);
range = z(1);
bearing = z(2);

fX = x + range * cos(theta + bearing);
fY = y + range * sin(theta + bearing);
dist = [];
for j = 4:2:length(xPred)
    tempFeature = xPred(j:j+1);
    currDist = sqrt((tempFeature(1) - fX)^2 + (tempFeature(2) - fY)^2);
    dist = [dist, currDist];
end

idx = find(dist==min(dist));

fX = xPred(idx*2 + 2);
fY = xPred(idx*2 + 3);


sigma = [fX - x; fY - y];
q = sigma'*sigma;
z_hat = [sqrt(q); atan2(sigma(2), sigma(1)) - theta];

H = zeros(2,n);
H_1 = [-1*(fX - x)/sqrt(q), -1*(fY-y)/sqrt(q), 0; (fY-y)/sqrt(q), -1*(fX-x)/sqrt(q), -1];
H(:,1:3) = H_1;

H(:, 2+2*idx:3+2*idx) = [sqrt(q)*sigma(1)/q, sqrt(q)*sigma(2)/q; -sigma(2)/q, sigma(1)/q];

Innov = z'-z_hat;
Innov(2) = angleWrap(Innov(2));

S = H * covPred * H' + Q;
K = covPred * H' * inv(S);
xEst = xPred + K * Innov;
covEst = covPred - K * S * K';
covEst = 0.5*(covEst+covEst');

end
