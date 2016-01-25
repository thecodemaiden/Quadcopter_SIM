% Compute the points on the edge of a prism centered at
% location (x, y, z) with width w, length l, and height h.
function [X, Y, Z] = prism_faces(x, y, z, w, l, h)
    X = [x x x x x+w x+w x+w x+w] - w/2;
    Y = [y y y+l y+l y y y+l y+l] - l/2;
    Z = [z z+h z z+h z z+h z z+h] - h/2;
end