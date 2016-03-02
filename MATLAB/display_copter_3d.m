function display_copter_3d(copter, plots)

motion = copter.motion;
drawing = copter.drawing;
x = motion.pos;
theta = motion.theta;

subplot(plots(1));
   % hold off;
            % Compute translation to correct linear position coordinates.
    move = makehgtform('translate', x);
    
      % Compute rotation to correct angles. Then, turn this rotation
        % into a 4x4 matrix represting this affine transformation.
    rotate = rotation(theta);
    rotate = [rotate zeros(3, 1); zeros(1, 3) 1];
    
    
        % Move the quadcopter to the right place, after putting it in the correct orientation.
        set(drawing.model,'Matrix', move * rotate);
        
        drawnow;

end

function Mr = rotation(theta)
Mr = zeros(3);
r = theta(1); p = theta(2); y = theta(3);
Mr(1,1) = cos(r)*cos(y)-cos(p)*sin(r)*sin(y);
Mr(1,2) = -cos(r)*sin(y)-cos(y)*cos(p)*sin(r);
Mr(1,3) = sin(r)*sin(p);

Mr(2,1) = cos(r)*cos(p)*sin(y) + cos(y)*sin(r);
Mr(2,2) = cos(r)*cos(p)*cos(y) - sin(r)*sin(y);
Mr(2,3) = -cos(r)*sin(p);

Mr(3,1) = sin(y)*sin(p);
Mr(3,2) = cos(y)*sin(p);
Mr(3,3) = cos(p);
end
