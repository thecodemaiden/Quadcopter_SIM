function display_copter_2d(motion, plots)

x = motion.pos;
% first we will draw copter from above, then we will add a side view
% may need to read https://en.wikipedia.org/wiki/Motion_field to correct
% this
%thrust_mult = 0.2;
%quiver(x(1),x(2),thrustin(1)*thrust_mult, thrustin(2)*thrust_mult, 'AutoScale', 'off');

%top view
subplot(plots(1));
%clf;
hold off;

%quiver messes up our scale :(
old_xlim = xlim;
old_ylim = ylim;

plot(x(1), x(2), 'r+', 'MarkerSize', 20);
%hold on;

axis([old_xlim old_ylim]);
xlabel('x');
ylabel('y');
grid on;
drawnow;

%side view #1
subplot(plots(2));
hold off;

%quiver messes up our scale :(
old_xlim = xlim;
old_ylim = ylim;

plot(x(2), x(3), 'r+', 'MarkerSize', 20);
%hold on;

axis([old_xlim old_ylim]);
xlabel('y');
ylabel('z');
grid on;

%side view #2
subplot(plots(3));
hold off;

%quiver messes up our scale :(
old_xlim = xlim;
old_ylim = ylim;

plot(x(1), x(3), 'r+', 'MarkerSize', 20);
%hold on;

axis([old_xlim old_ylim]);
xlabel('x');
ylabel('z');
grid on;

drawnow;
end
