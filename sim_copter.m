
function [data] = sim_copter(copter, motion, N, dt)
% Simulation times, in seconds.

if nargin < 4
    dt = 0.05;
end

if nargin < 3
    N = 300;% 1;
end

times = (1:N)*dt;
%times = (0:dt:N);

h = figure('KeyPressFcn',@stop_keypress);
clf(h);

% Create the drawn quadcopter object. Returns a handle to
% the quadcopter itself as well as the thrust-display cylinders.
[q, thrusts] = make_quadcopter_3d;
drawing.model = q;
drawing.thrusts = thrusts;
 

% save simulation values
data = struct;
data.t = times';
data.x = zeros(N, 3);
data.angacc = zeros(N, 3);

do3d = false;

if do3d
    plots = subplot(1,1,1);
    % Set axis scale and labels.
    axis([-30 30 -20 20 -3 3]);
    grid on;
    zlabel('Height');
else
    clf;
    plots = [subplot(3,1,1) subplot(3,1,2), subplot(3,1,3)];
    subplot(plots(1));
    axis([-30 30 -30 30]);
    subplot(plots(2));
    axis([-30 30 -3 3]);
    subplot(plots(3));
    axis([-30 30 -3 3]);
end
    title('Quadcopter Flight Simulation');
    
keepLooping = true;

function stop_keypress(~,eventData)
    if strcmp(eventData.Key,'q')             %# If q key is pressed, set
      keepLooping = false;                   %#   keepLooping to false
    end
end

for t = 1:N,
    % Take input from our controller.

    data.x(t, 1:3) = motion.pos';
   data.angacc(t, 1:3) = motion.angacc'; 
   if do3d
    display_copter_3d(motion, drawing, plots);
   else
    display_copter_2d(motion, plots);
   end
    motion = update_copter_motion(copter, motion, dt);
   
   if ~keepLooping
       break
   end
end
close(h);
end

