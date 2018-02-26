function [worldInfo] = initialize_sim(copterlist, draw)

N = 0; % TODO: prealloc then resize?!
times = zeros(1,N);
figureNum = 0;
plt = 0;
initlist = [];
% change/set the dt value in each copter
for i = 1:length(copterlist)
    c = copterlist(i);
    
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
    h = figure();
    figureNum = h.Number;
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

worldInfo.copters = copterlist;
worldInfo.t = 0.0;
worldInfo.room = test_room;
worldInfo.fig = figureNum;
worldInfo.plots = plots;
worldInfo.draw3d = true;

end