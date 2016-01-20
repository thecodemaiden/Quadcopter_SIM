function [ copter ] = make_copter()
%MAKE_COPTER Create a numerical quadcopter model
% graph_type should be "2d", "3d", or "none" (default)

copter = struct;

% Simulate some disturbance in the angular velocity.

% The magnitude of the deviation is in radians / second.

%deviation = 20; thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);


copter.m = 0.27;
copter.k = 1e-4;
copter.kd = 0.25;
copter.I = diag([5e-3, 5e-3, 10e-3]);
copter.L = 0.0485;
copter.b= 1e-7;

% make a graphical model

end
