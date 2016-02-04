function [ new_motion ] = step_motion( dt, motion_params, device_params, world_params )
%STEP_MOTION Update the motion of the psrticle by one tick
%   dt - assumed to be in seconds
%   motion_params - instantaneous values:
%       p: [x y z]  position, m
%       v: [v_x v_y v_z] velocity, m/s - yaw is derived from this
%       w: [ w_z ] the yaw, relative to mag. north, radians
%   device_params - do not modify after simulation starts:
%       p_g: (0-1) confidence in gyroscope
%       d_g: std of gyro error
%       g: [g_x g_y g_z] current gyro readings
%       d_a: std of accelerometer error (affects PID for thrust)
%       a: [a_x a_y a_z]  current accelerometer readings, N/kg
%   world_params - room boundaries, wind direction... etc
%       walls: [{[x1 y1]; [x2 y2]}, ... ] edges of room, don't change
%       corners: n x 2 matrix, [x y] (too lazy to calculate)
%       wind: [w_x w_y w_z] draft in the room

% first draft - bounce off walls, ideal motion

new_motion = motion_params;
corners = world_params.corners;
% assume old position was within bounds

new_p = new_motion.p + dt*new_motion.v;

[still_in, now_on] = inpolygon(new_p(1), new_p(2), corners(:,1), corners(:,2));
if now_on,
    % lucky - we're on the border. find the colliding wall and reflect accordingly
    disp('Boing!');
elseif ~still_in,
    % ugh... out of bounds... find the time of impact, rebound, then
    % calculate remaining motion
    disp('ARGH');
end
new_motion.p = new_p;




end

