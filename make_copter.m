function [ copter, motion ] = make_copter()
% [C, M] = MAKE_COPTER Create a numerical quadcopter model
% C contains unchaning physical constants, while M contains motion
% parameters

copter.m = 0.27; % mass in grams
copter.k = 1e-4; % thrust to ang. velocity proportionality constant
copter.kd = 0.25; % friction constant
copter.I = diag([5e-3, 5e-3, 10e-3]); % inertial moment
copter.L = 0.0485; % length of arms
copter.b = 1e-7; % drag coefficient

% variables needed to track motion

motion.pos = [0; 0; 0]; % x-y-z potions
motion.angacc = [0; 0; 0]; % angular acceleration about each axis
motion.thetadot = [0;0;0]; % angular velocity
motion.theta = [0; 0; 0]; % orientation/pose
motion.thrust = [1 1 1 1]; % thrust on each motor as fraction of hover velocity
motion.g  = 9.81; % acceleration due to gravity
motion.xdot = [0; 0; 0]; % linear velocity
motion.forces = [0; 0; 0]; % external applied force in newtons
end
