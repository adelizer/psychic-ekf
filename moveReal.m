function x_new = moveReal(x_old, u, dt )
% move_real(x_old, u, dt) Summary of this function goes here
% Velocity model for motion 
%   Detailed explanation goes here


%%
v = u(1);
w = u(2);
theta = x_old(3);


if(w ~= 0)
    x_new(1) = x_old(1) + (-v/w) * sin(theta) + (v/w) * sin(theta + dt*w);
    x_new(2) = x_old(2) + (v/w)  * cos(theta) - (v/w) * cos(theta + dt*w);
    x_new(3) = x_old(3) + w*dt;
else
    x_new(1) = x_old(1) + v * cos(theta) * dt;
    x_new(2) = x_old(2) + v * sin(theta) * dt;
    x_new(3) = x_old(3);
end

tempAngle = angleWrap(x_new(3));
x_new(3) = tempAngle;

% Copy the rest of the vector 
if(length(x_old) > 3)
x_new(4:end) = x_old(4:end);
end