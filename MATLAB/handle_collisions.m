function [forces] = handle_collisions(copter, motion, space)
% super naive - ignore angular momentum, and orientation; simply reflect
% appropriate velocity component

vel = motion.xdot;
pos = motion.pos;
L = copter.L;
m = copter.m;
thickness = copter.zL;
faces = space.faces;

bbox = [L L thickness];

forces = [0;0;0];
% are we colliding with part of the space?
% start out simple - assume all faces of the space are aligned with the
% axes, and find the plane equation
% px + qy + rz = c
for i=1:length(faces),
f = faces{i};
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

%super lazy checking here
for i=1:3
    coll_vel = 2*vel(i);
    if constraint(i) < 0
        % super duper naive - collide if we are L away from the wall
        overlap = constraint(i) - (pos(i)-bbox(i));
        % if we are embedded in the wall, don't let velocity bounce around
        if overlap > 0 && sign(coll_vel) == -1
            % elastic collision - F = m(dv)/t
            % TODO: the dv, therefore F, will depend on penetration depth
            forces(i) = m*-coll_vel/motion.dt;
        end
    elseif constraint(i) > 0
        overlap = (pos(i)+L) - constraint(i);
        if overlap > 0 && sign(coll_vel) == 1
            forces(i) = m*-coll_vel/motion.dt;
        end
    end
end
end
end
