function [forces] = handle_collisions(copter, motion, space)
% super naive - ignore angular momentum, and orientation; simply reflect
% appropriate velocity component

vel = motion.xdot;
pos = motion.pos;
L = copter.L;
m = copter.m;
thickness = copter.zr;
faces = space.faces;

forces = [0;0;0];
% are we colliding with part of the space?
% start out simple - assume all faces of the space are aligned with the
% axes, and find the plane equation
% px + qy + rz = c

f = faces{2};
ab = f(2,:) - f(1,:);
bc = f(3,:) - f(2,:);
d = f(4,:);
normal = cross(ab,bc);
% we already know how plane is aligned, but we need to get the x/y/z coord
% next step works only for planes aligned with the axes, i think
% normal = [p q r]*c
c = (normal .* d);
constraint = c./normal; 
%constraint(isnan(constraint)) = 0;

%the plane x=-10 is now represented as [-10 0 0]

% now see if we are on the same side of the wall as the center
% assume we don't go through walls
if constraint(2) < 0
    % equation is x = -a
    % super duper naive - collide if we are L away from the wall
    penetration = constraint(2) - (pos(2)-L);
    coll_vel = 2*vel(2);
    % if we are embedded in the wall, don't let velocity bounce around
    if penetration > 0 && sign(coll_vel) == -1
        % elastic collision - F = m(dv)/t
        % TODO: the dv, therefore F, will depend on penetration depth
        forces(2) = m*-coll_vel/motion.dt;
    end
end

end