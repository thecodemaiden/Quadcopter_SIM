function [ new_motion ] = update_copter_motion(physical, motion )

new_motion = motion;
theta = motion.theta;
i = motion.thrust;
x = motion.pos;
dt = motion.dt;

omega = thetadot2omega(motion.thetadot, theta);
% Compute linear and angular accelerations.
a = acceleration(i, theta, motion.xdot, physical.m, motion.g, physical.k, physical.kd);
%f = ma
a = a + motion.forces/physical.m;
   
omegadot = angular_acceleration(i, omega, physical.I, physical.L, physical.b, physical.k);
omega = omega + dt * omegadot;
thetadot = omega2thetadot(omega, theta); 
theta = theta + dt * thetadot; 
new_motion.xdot=motion.xdot+dt* a;
x = x + dt*new_motion.xdot;
    
new_motion.pos = x;
new_motion.angacc = omegadot';
new_motion.theta = theta;
new_motion.thetadot = thetadot;
end

function T = thrust(inputs, k)
% Inputs are values for ?i2
T = [0; 0; k * sum(inputs)];
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coe
function tau = torques(inputs, L, b, k)
% Inputs are values for ?i2
tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end

function a = acceleration(inputs, angles, xdot, m, g, k, kd)
gravity = [0; 0; -g];
R = rotation(angles);
T = R * thrust(inputs, k);
Fd = -kd * xdot;
a = gravity + 1 / m * T + Fd; 
end
 
function omegadot = angular_acceleration(inputs, omega, I, L, b, k) 
tau = torques(inputs, L, b, k);
omegadot = I \ (tau - cross(omega, I * omega));
end

function omega = thetadot2omega(thetadot, theta)
M = [  1 0 -sin(theta(2));
       0 cos(theta(1)) cos(theta(2))*sin(theta(1));
       0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];
omega = M * thetadot;
end

function td = omega2thetadot(omega, theta)
% the relation is omega = M * thetadot
% so we need thetadot = inv(M)*omega
M = [  1  0  -sin(theta(2));
       0 cos(theta(1)) cos(theta(2))*sin(theta(1));
       0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];
td = M \ omega;
end

function Mr = rotation(theta)
Mr = zeros(3);
r = theta(1); p = theta(2); y = theta(3);
Mr(1,1) = cos(r)*cos(y)-cos(p)*sin(r)*sin(y);
Mr(1,2) = -cos(r)*sin(y)-cos(y)*cos(p)*sin(r);
Mr(1,3) = sin(r)*sin(p);

Mr(2,1) = cos(r)*cos(p)*sin(y) + cos(y)*sin(r);
Mr(2,2) = cos(r)*cos(p)*cos(y) - sin(r)*sin(y);
Mr(2,3) = -cos(r)*sin(p);

Mr(3,1) = sin(y)*sin(p);
Mr(3,2) = cos(y)*sin(p);
Mr(3,3) = cos(p);
end
