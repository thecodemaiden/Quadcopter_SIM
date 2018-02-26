function worldInfo = step_sim(worldInfo, dt)

copterlist = worldInfo.copters;
test_room = worldInfo.room;
figureNum = worldInfo.fig;
plots = worldInfo.plots;
do3d = worldInfo.draw3d;
% TODO: always redraw, or use accumulator?

if nargin < 2
    dt = 0.05;
end
t = worldInfo.t+dt;

draw = figureNum > 0;

need_redraw = true;

for i = 1:length(copterlist)
    cpt = copterlist(i);
    cpt.motion.dt = dt;
    c = handle_copter(cpt, test_room);
    if need_redraw
        if do3d
            display_copter_3d(c,plots);
        else
            display_copter_2d(c,plots);
        end
    end
    copterlist(i) = c;
end

worldInfo.copters = copterlist;
worldInfo.room = test_room;
worldInfo.t = t;


if ~draw
    close(w);
end
end

function [updated] = handle_copter(copter, room)

updated = copter;
% Take input from our controller.
motion = copter.motion;
data = copter.data;
physical = copter.physical;
pid_state = copter.pid_state;

data.x(end+1, 1:3) = motion.pos';
data.angacc(end+1, 1:3) = motion.angacc';
data.theta(end+1, 1:3) = motion.theta';
data.v(end+1, 1:3) = motion.xdot';

motion = update_copter_motion(physical, motion);

[thrust_adj, pid_state] = pid_controller(pid_state, motion, copter.motion.target_theta);
motion.thrust = thrust_adj;

f_coll = handle_collisions(physical, motion, room);
motion.forces = f_coll;

updated.motion = motion;
updated.physical = physical;
updated.pid_state = pid_state;
updated.data = data;
end

