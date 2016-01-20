function display_copter_3d(thrustin, x, theta, drawing, plots)
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


