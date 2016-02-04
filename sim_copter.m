
function [data] = sim_copter(copter, motion, N, dt)
% Simulation times, in seconds.
step_time = 0.1;

if nargin < 4
    dt = 0.05;
end

if nargin < 3
    N = 300;
end

times = (1:N)*dt;

h = figure(10);
set(h,'KeyPressFcn',@stop_keypress);
clf(h);

% Create the drawn quadcopter object. Returns a handle to
% the quadcopter itself as well as the thrust-display cylinders.
drawing.model = make_quadcopter_3d; 

% save simulation values
data = struct;
data.t = times';
data.x = zeros(N, 3);
data.angacc = zeros(N, 3);
data.theta = zeros(N,3);
data.v = zeros(N,3);

do3d = true;

test_room = make_room(0,0,0,20,20,6);

if do3d
    plots = subplot(1,1,1);
    % Set axis scale and labels.
    axis([-15 15 -15 15 -5 5]);
    grid on;
    zlabel('Height');
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
    
keepLooping = true;

function stop_keypress(~,eventData)
    if strcmp(eventData.Key,'q')             %# If q key is pressed, set
      keepLooping = false;                   %#   keepLooping to false
    end
end

%r_params = [5.0, 3.0, 2.0]; 
%p_params = r_params;
%y_params = [10, 1, 0.35];
%pid_params = vertcat(r_params, p_params, y_params);
attitude_state = copter;
attitude_state.g = motion.g;
attitude_state.dt = dt;
attitude_state.Kp = 10.0; attitude_state.Ki = 3.0; attitude_state.Kd=3.0;
  
motion.thrust = [1 1 1 1]*6622;

accum = 0.0;
target_theta = [0;0.1;0];

pitch_interval = 100;

for t = 1:N,
   % Take input from our controller.
    
   data.x(t, 1:3) = motion.pos';
   data.angacc(t, 1:3) = motion.angacc'; 
   data.theta(t, 1:3) = motion.theta';
   data.v(t, 1:3) = motion.xdot';
   if accum > step_time
       if do3d
           hold off;
           display_copter_3d(motion, drawing, plots);
           hold on;
           draw_room_3d(test_room,h);
       else
           display_copter_2d(motion, plots);
       end
       accum = accum - step_time;
   end
   accum = accum + dt;
   motion = update_copter_motion(copter, motion, dt);
   if t < 100
       chosen_target = target_theta;
   else
       chosen_target = [0;0;0];
   end
   
   [thrust_adj, attitude_state] = pid_controller(attitude_state, motion, chosen_target);
    
    %motion.thrust = limit_thrust;
    %disp(['Applying thrust: ', num2str(thrust_adj)]);
    %disp(['Rotation rate: ', num2str(motion.thetadot')]);
    %disp(['Rotation: ', num2str(motion.theta')]);
    %disp(' ');
    pause(0.01);
    motion.thrust = thrust_adj;
   if ~keepLooping
       break
   end
end
%close(h);
end

%pid_r = make_pid(3.5*t_mult, 2.0*t_mult, 0.0, 0, 20.0*t_mult);
%pid_p = make_pid(3.5*t_mult, 2.0*t_mult, 0.0, 0, 20.0*t_mult);
%pid_y = make_pid(10.0*t_mult, 1.0*t_mult, 0.35*t_mult, 0, 360*t_mult); 
function [r] = make_pid(kp, ki, kd, target, limit)
    r.kp = kp;
    r.ki = ki;
    r.kd = kd;
    r.lim = abs(limit);
    r.target = target;
    r.current = 0;
    r.error = 0;
    r.integ = 0;
    r.deriv = 0;
    r.prev_error = 0;
    r.output = 0;
end

function new_p = update_pid(p, dt)
p.error = p.target - p.current;
%p.error = p.current - p.target;
p.integ = p.integ + p.error*dt;
if (p.integ > p.lim)
    p.integ = p.lim;
elseif (p.integ < -p.lim)
    p.integ = -p.lim;
end

p.deriv = (p.error - p.prev_error)/dt;

p.output = p.error*p.kp + p.integ*p.ki + p.deriv*p.kd;

p.prev_error = p.error;
new_p = p;
end

