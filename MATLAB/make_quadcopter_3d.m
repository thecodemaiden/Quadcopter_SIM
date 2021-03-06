function [h] = make_quadcopter_3d

% Draw a quadcopter. Return a handle to the quadcopter object.
    % Draw arms.
    L = 1;
    h(1) = prism(0, 0, 0, L, 0.1, 0.1);
    h(2) = prism(0, 0, 0, 0.1, L, 0.1);

    % Draw bulbs representing propellers at the end of each arm.
    [x, y, z] = sphere;
    x = 0.05 * x;
    y = 0.05 * y;
    z = 0.05 * z;
    h(3) = surf(x - L/2, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(4) = surf(x + L/2, y, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(5) = surf(x, y - L/2, z, 'EdgeColor', 'none', 'FaceColor', 'b');
    h(6) = surf(x, y + L/2, z, 'EdgeColor', 'none', 'FaceColor', 'b');

    % Conjoin all quadcopter parts into one object.
    t = hgtransform;
    set(h, 'Parent', t);
    h = t;
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