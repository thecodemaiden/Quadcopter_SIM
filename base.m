
function [data] = base(copter, N, dt)
% Simulation times, in seconds.

if nargin < 3
    dt = 0.005;
end

if nargin < 2 || N < 50
    N = 50;
end

times = (1:N)*dt;

h = figure('KeyPressFcn',@stop_keypress);
%figure; plots = [subplot(3, 2, 1:4), subplot(3, 2, 5), subplot(3, 2, 6)];
 %   subplot(plots(1));
 plots = subplot(1,1,1);

% Initial simulation state.

x=[0;0;10];

theta = [0;0;0];

% Simulate some disturbance in the angular velocity.

% The magnitude of the deviation is in radians / second.

%deviation = 20; thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);
thetadot = [0;0;0];

Ixx = 0.1; Iyy = 0.1; Izz = 0.1;



% save simulation values
data = struct;
data.t = times';
data.x = zeros(N, 3);
data.angvel = zeros(N, 3);

    % Set axis scale and labels.
    axis([-10 30 -20 20 5 15]);
    zlabel('Height');
    title('Quadcopter Flight Simulation');

keepLooping = true;

function stop_keypress(hObject,eventData)
    if strcmp(eventData.Key,'q')             %# If q key is pressed, set
      keepLooping = false;                   %#   keepLooping to false
    end
  end
    
for t = 1:N,
    % Take input from our controller.
    %i = input('Input: ');
    i = [ 2.45 2.45 2.45 2.45];
    omega = thetadot2omega(copter.thetadot, theta);
    % Compute linear and angular accelerations.
    a = acceleration(i, theta, copter.xdot, copter.m, copter.g, copter.k, copter.kd);
    omegadot = angular_acceleration(i, omega, copter.I, copter.L, copter.b, copter.k);
    omega = omega + dt * omegadot;
    copter.thetadot = omega2thetadot(omega, theta); 
    theta = theta + dt * copter.thetadot; 
    copter.xdot=copter.xdot+dt* a;
    x=x+dt*copter.xdot;

    data.x(t, :) = x';
   data.angvel(t, :) = omegadot'; 
   displaycopter(i, x, theta, copter.drawing, plots);
   if ~keepLooping
       break
   end
end
close(h);
end


function T = thrust(inputs, k)
% Inputs are values for ?i2
T = [0; 0; k * sum(inputs)];
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coe
function tau = torques(inputs, L, b, k)
% Inputs are values for ?i2
tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end

function a = acceleration(inputs, angles, xdot, m, g, k, kd)
gravity = [0; 0; -g];
R = rotation(angles);
T = R * thrust(inputs, k);
Fd = -kd * xdot;
a = gravity + 1 / m * T + Fd; 
end
 
function omegadot = angular_acceleration(inputs, omega, I, L, b, k) 
tau = torques(inputs, L, b, k);
omegadot = I \ (tau - cross(omega, I * omega));
end

function omega = thetadot2omega(thetadot, theta)
M = [  1 0 -sin(theta(2));
       0 cos(theta(1)) cos(theta(2))*sin(theta(1));
       0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];
omega = M * thetadot;
end

function td = omega2thetadot(omega, theta)
% the relation is omega = M * thetadot
% so we need thetadot = inv(M)*omega
M = [  1  0  -sin(theta(2));
       0 cos(theta(1)) cos(theta(2))*sin(theta(1));
       0 -sin(theta(1)) cos(theta(2))*cos(theta(1))];
td = M \ omega;
end

function Mr = rotation(theta)
Mr = zeros(3);
r = theta(1); p = theta(2); y = theta(3);
Mr(1,1) = cos(r)*cos(y)-cos(p)*sin(r)*sin(y);
Mr(1,2) = -cos(p)*sin(r)-cos(r)*cos(p)*sin(y);
Mr(1,3) = sin(p)*sin(y);

Mr(2,1) = cos(p)*cos(y)*sin(r) + cos(r)*sin(y);
Mr(2,2) = cos(r)*cos(p)*cos(y) - sin(r)*sin(y);
Mr(2,3) = -cos(y)*sin(p);

Mr(3,1) = sin(r)*sin(p);
Mr(3,2) = cos(r)*sin(p);
Mr(3,3) = cos(p);
end


function displaycopter(thrustin, x, theta, drawing, plots)
    subplot(plots(1));
    hold off;
            % Compute translation to correct linear position coordinates.
    move = makehgtform('translate', x);
    
      % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
    rotate = rotation(theta);
    rotate = [rotate zeros(3, 1); zeros(1, 3) 1];
    
    
        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(drawing.model,'Matrix', move * rotate);
        
        
        scales = exp(thrustin / min(abs(thrustin)) + 5) - exp(6) +  1.5;
        for i = 1:4
            % Scale each cylinder. For negative scales, we need to flip the cylinder
            % using a rotation, because makehgtform does not understand negative scaling.
            s = scales(i);
            if s < 0
                scalez = makehgtform('yrotate', pi)  * makehgtform('scale', [1, 1, abs(s)]);
            elseif s > 0
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
            set(drawing.thrusts(i), 'Matrix', move * rotate * scalez);
        end
        drawnow;

end

