function [ copter ] = make_copter()
% [C] = MAKE_COPTER Create a numerical quadcopter model
% C.physical contains unchagning physical constants, while 
% C.motion contains motion parameters

physical.m = 0.027; % mass in kg
physical.k = 1e-4; % thrust to ang. velocity proportionality constant
physical.kd = 0.25; % friction constant
physical.I = diag([5e-3, 5e-3, 10e-3]); % inertial moment
physical.L = 0.0485; % length of arms in meters
physical.b = 1e-7; % drag coefficient
physical.zL = 0.012; % z thickness of quadcopter
% variables needed to track motion

motion.pos = [0; 0; 0]; % x-y-z potions
motion.angacc = [0; 0; 0]; % angular acceleration about each axis
motion.thetadot = [0;0;0]; % angular velocity
motion.theta = [0; 0; 0]; % orientation/pose
motion.thrust = [1 1 1 1]; % thrust on each motor as fraction of hover velocity
motion.g  = 9.81; % acceleration due to gravity
motion.xdot = [0; 0; 0]; % linear velocity
motion.forces = [0; 0; 0]; % external applied force in newtons
motion.dt = 0.005; % time step
motion.target_theta = [0;0;0];

copter.physical = physical;
copter.motion = motion;
copter.name = 'copter';
copter.color = 'r';
end
