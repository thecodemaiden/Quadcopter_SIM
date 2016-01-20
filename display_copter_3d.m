function display_copter_3d(motion, drawing, plots)
thrustin = motion.thrust;
x = motion.pos;
theta = motion.theta;

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
                if s == Inf
                    s = 1;
                end
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
            set(drawing.thrusts(i), 'Matrix', move * rotate * scalez);
        end
        drawnow;

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
                if s == Inf
                    s = 1;
                end
                scalez = makehgtform('scale', [1, 1, s]);
            end

            % Scale the cylinder as appropriate, then move it to
            % be at the same place as the quadcopter propeller.
            set(drawing.thrusts(i), 'Matrix', move * rotate * scalez);
        end
        drawnow;

end

% Draw a 3D prism at (x, y, z) with width w,
% length l, and height h. Return a handle to
% the prism object.
function h = prism(x, y, z, w, l, h)
    [X, Y, Z] = prism_faces(x, y, z, w, l, h);

    faces(1, :) = [4 2 1 3];
    faces(2, :) = [4 2 1 3] + 4;
    faces(3, :) = [4 2 6 8];
    faces(4, :) = [4 2 6 8] - 1;
    faces(5, :) = [1 2 6 5];
    faces(6, :) = [1 2 6 5] + 2;

    for i = 1:size(faces, 1)
        h(i) = fill3(X(faces(i, :)), Y(faces(i, :)), Z(faces(i, :)), 'r'); hold on;
    end

    % Conjoin all prism faces into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
end

% Compute the points on the edge of a prism at
% location (x, y, z) with width w, length l, and height h.
function [X, Y, Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w];
    Y = [y y y+l y+l y y y+l y+l];
    Z = [z z+h z z+h z z+h z z+h];
end