function display_copter_2d(copter, plots)

motion = copter.motion;
x = motion.pos;
% first we will draw copter from above, then we will add a side view
% may need to read https://en.wikipedia.org/wiki/Motion_field to correct
% this
%thrust_mult = 0.2;
%quiver(x(1),x(2),thrustin(1)*thrust_mult, thrustin(2)*thrust_mult, 'AutoScale', 'off');

%top view
subplot(plots(1));

%quiver messes up our scale :(
%old_xlim = xlim;
%old_ylim = ylim;

if ~isfield(copter.drawing, 'inZ')
   copter.drawing.inZ = plot(x(1), x(2), 'r+', 'MarkerSize', 20);
else
    set(copter.drawing.inZ, 'XData', x(1), 'yData', x(2))
end
%hold on;

%axis([old_xlim old_ylim]);
%xlabel('x');
%ylabel('y');
drawnow;

%side view #1
subplot(plots(2));

%quiver messes up our scale :(
%old_xlim = xlim;
%old_ylim = ylim;

if ~isfield(copter.drawing, 'inX')
    copter.drawing.inX = plot(x(2), x(3), 'r+', 'MarkerSize', 20);
else
    set(copter.drawing.inX, 'XData', x(2), 'YData', x(3));
end

%axis([old_xlim old_ylim]);
%xlabel('y');
%ylabel('z');
drawnow;

%side view #2
subplot(plots(3));

%quiver messes up our scale :(
%old_xlim = xlim;
%old_ylim = ylim;

if ~isfield(copter.drawing, 'inY')
    plot(x(1), x(3), 'r+', 'MarkerSize', 20);
else
    set(copter.drawing.inY, 'XData', x(1), 'YData', x(2))
end
    %hold on;

%axis([old_xlim old_ylim]);
%xlabel('x');
%ylabel('z');
%grid on;

drawnow;
end
