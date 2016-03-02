function copterlist = sim_copter(copterlist, N, dt, draw)
% copterlist should be a row vector containing copters created by 
% make_copter
if nargin < 4
    draw = true;
end

if nargin < 3
    dt = 0.05;
end

if nargin < 2
    N = 300;
end

if draw
    step_time=0.1;
else
    step_time = 0.5;
end


times = (1:N)*dt;

initlist = [];
% change/set the dt value in each copter
for i = 1:length(copterlist)
    c = copterlist(i);
    
    c.motion.dt = dt;
    c.data.t = times';
    % log variables
    c.data.x = zeros(N, 3);
    c.data.angacc = zeros(N, 3);
    c.data.theta = zeros(N,3);
    c.data.v = zeros(N,3);
    
    %pid stuff
    c.pid_state = c.physical;
    c.pid_state.Kp = 10.0; c.pid_state.Ki = 0.0; c.pid_state.Kd=0.0;
    %c.motion.thrust = c.motion.thrust*6618;    
    if i==1
        initlist = c;
    else
        initlist(i) = c;
    end
end

copterlist = initlist;

do3d = true;

test_room = make_room(0,0,0,20,20,6);

if draw
    h = figure(10);
    set(h,'KeyPressFcn',@stop_keypress);
    clf(h);
    if do3d
        hold on;
        % Create the drawn quadcopter objects
        for i = 1:length(copterlist)
            copterlist(i).drawing.model = make_quadcopter_3d;
        end
        
        plots = subplot(1,1,1);
        % Set axis scale and labels.
        axis([-15 15 -15 15 -5 5]);
        grid on;
        zlabel('Height');
        xlabel('x');
        ylabel('y');
        
        draw_room_3d(test_room,h);
    else
        plots = [subplot(3,1,1) subplot(3,1,2), subplot(3,1,3)];
        subplot(plots(1));
        axis([-10 10 -10 10]);
        subplot(plots(2));
        axis([-10 10 -3 3]);
        subplot(plots(3));
        axis([-10 10 -3 3]);
    end
    title('Quadcopter Flight Simulation');
else
    % if we're not drawing, show a progress bar
    w = waitbar(0, 'Ready...');
end

keepLooping = true;

    function stop_keypress(~,eventData)
        if strcmp(eventData.Key,'q')             %# If q key is pressed, set
            keepLooping = false;                   %#   keepLooping to false
        end
    end

accum = 0.0;

%pause(5);

for t = 1:N,
    need_redraw = false;
    if accum > step_time
        if draw
            need_redraw = true;
        else
            waitbar(t/N, w, 'Simulating...');
        end
        accum = accum - step_time;
    end
    accum = accum + dt;
    
    for i = 1:length(copterlist)
        c = handle_copter(t, copterlist(i), test_room);
        if need_redraw
            if do3d
                display_copter_3d(c,plots);
            else
                display_copter_2d(c,plots);
            end
        end
        copterlist(i) = c;
    end
    
    pause(0.01);
    if ~keepLooping
        break
    end
end
if ~draw
    close(w);
end
end

function [updated] = handle_copter(t, copter, room)

    updated = copter;
 % Take input from our controller.
    motion = copter.motion;
    data = copter.data;
    physical = copter.physical;
    pid_state = copter.pid_state;
    
    data.x(t, 1:3) = motion.pos';
    data.angacc(t, 1:3) = motion.angacc';
    data.theta(t, 1:3) = motion.theta';
    data.v(t, 1:3) = motion.xdot';
    
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

