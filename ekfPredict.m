function [xNew, covNew] = ekfPredict(u, xOld, covOld, alpha, dt)
%ekfPredict Summary of this function goes here
% Calculates the expected pose and covariance based on old states and
% control input
%   Detailed explanation goes here

%%
v = u(1);
w = u(2);
theta = xOld(3);

n = length(xOld);

% Maps from the low dimension space to the higher dimension space
F = [eye(3), zeros(3,length(xOld)-3)];

% alpha 1 -> v based on v
% alpha 2 -> v based on v
% alpha 3 -> w based on v
% alpha 4 -> w based on w

% Formulate the process noise
M = [alpha(1)*v^2 + alpha(2)*w^2, 0; 0, alpha(3)*v^2+alpha(4)*w^2];

if(w~=0)
    xNew = xOld + F' * [((-v/w) * sin(theta)) + ((v/w) * sin(theta + w*dt)); ...
        ((v/w) * cos(theta)) - ((v/w) * cos(theta + w*dt)); ...
        w*dt];
    
    
    % the eye(n) and the entries are the derivatives of the motion model
    % The dimension has to be adjusted hence F' ... F
    % The result is an identity probagating through all the uncertainties of
    % the landmarks.
    G = eye(n) + F' * [0,0, ((-v/w) * cos(theta)) + ((v/w) * cos(theta + w*dt)); 0,0, ((-v/w) * sin(theta)) + ((v/w) * sin(theta + w*dt)) ; 0,0,0] * F;
    
    
    
    V = [(-sin(theta) + sin(theta+w*dt))/w, (sin(theta) - sin(theta+w*dt))*v/w^2 + v*dt*cos(theta+w*dt)/w;...
        (cos(theta) - cos(theta+w*dt))/w, -1*((cos(theta) - cos(theta+w*dt))*v/w^2) + v*dt*sin(theta+w*dt)/w;...
        0, dt];
else
    xNew = xOld + F' * [v*cos(theta)*dt; ...
        v*sin(theta)*dt; ...
        0];
    
    G = eye(length(xOld)) + F' * [0,0, -v*sin(theta)*dt; 0,0, v*cos(theta)*dt ; 0,0,0] * F;
    
    % The effect of noise in the control over the states
    % similar to the state transition 
    % V shapes the direction of the uncertainty to form the ellipse
    V = [cos(theta)*dt, 0;...
        sin(theta)*dt, 0;...
        0, dt];
end

tempAngle =  angleWrap(xNew(3));
xNew(3) =tempAngle;
% V*M*V' maps the noise from the contorl inputs v,w into the states (x y theta)
% The usage of F is to map the total noise into the complete augmented with
% the feature coordinates
covNew = G * covOld * G' + F'* V*M*V' * F;

end

