function display_copter_2d(thrustin, x, plots)

% first we will draw copter from above, then we will add a side view
subplot(plots(1));
cla;
hold off;

plot(x(1), x(2), 'r+', 'MarkerSize', 10);

% may need to read https://en.wikipedia.org/wiki/Motion_field to correct
% this
thrust_mult = 5.0;
quiver(x(1),x(2),thrustin(1)*thrust_mult, thrustin(2)*thrust_mult);

end
