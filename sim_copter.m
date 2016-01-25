
function [data] = sim_copter(copter, motion, N, dt)
% Simulation times, in seconds.
step_time = 0.05;

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
[q, thrusts] = make_quadcopter_3d;
drawing.model = q;
drawing.thrusts = thrusts;
 

% save simulation values
data = struct;
data.t = times';
data.x = zeros(N, 3);
data.angacc = zeros(N, 3);

do3d = true;

if do3d
    plots = subplot(1,1,1);
    % Set axis scale and labels.
    axis([-30 30 -20 20 -3 3]);
    grid on;
    zlabel('Height');
else
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

t_mult = (6620/32767); % crazyflie values are too big

%use PID values from the firmware
pid_r = make_pid(3.5*t_mult, 2.0*t_mult, 0.0, 0, 20.0*t_mult);
pid_p = make_pid(3.5*t_mult, 2.0*t_mult, 0.0, 0, 20.0*t_mult);
pid_y = make_pid(10.0*t_mult, 1.0*t_mult, 0.35*t_mult, 0, 360*t_mult); 

pid_r_rate = make_pid(70*t_mult,0,0,0,33.0*t_mult);
pid_p_rate = make_pid(70*t_mult,0,0,0,33.0*t_mult);
pid_y_rate = make_pid(70*t_mult, 16.7*t_mult, 0, 0, 500/3*t_mult);
   
t_mult = 1;
base_thrust = [1 1 1 1];

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
    
    %theta order seems to be pry, not rpy
    %update PIDs as CFlie does - first get target rates given current pose
    pid_r.current = motion.theta(2);
    pid_r = update_pid(pid_r, dt);
    pid_p.current = motion.theta(1);
    pid_p = update_pid(pid_p, dt);
    pid_y.current = motion.theta(3);
    pid_y = update_pid(pid_y, dt);
    
    %then convert rates (output of PID) to actuator output
    pid_r_rate.target = pid_r.output;
    pid_p_rate.target = pid_p.output;
    pid_y_rate.target = pid_y.output;
    
    pid_r_rate.current = motion.thetadot(2);
    pid_p_rate.current = motion.thetadot(1);
    pid_y_rate.current = motion.thetadot(3);
    
    pid_r_rate = update_pid(pid_r_rate, dt);
    pid_p_rate = update_pid(pid_p_rate, dt);
    pid_y_rate = update_pid(pid_y_rate,dt);
    
    disp(['DTheta: ', num2str(motion.thetadot')]);
    disp(['theta: ', num2str(motion.theta')]);
    % plus orientation... seems wrong...
    r = pid_r_rate.output*t_mult;
    p = pid_p_rate.output*t_mult;
    y =  pid_y_rate.output*t_mult*(2*pi)/360;
    
    % + mode
    thrust = base_thrust - [p + y, -r - y, -p + y, r - y]; 
    
    % xmode
    %r = r/2;
    %p = p/2;
   % thrust =  base_thrust + [-r + p +y, -r - p - y, r - p + y, r + p + y];
    
    
    limit_thrust = max(min(thrust,2), 0);
    motion.thrust = limit_thrust;
    disp(['Applying thrust:', num2str(limit_thrust)]);
    disp(' ');
    
    pause(step_time);
   if ~keepLooping
       break
   end
end
%close(h);
end

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

