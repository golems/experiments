figure;
plot(bla(:, end), bla(:, 1), 'r'); hold on;
plot(bla(:, end), bla(:, 2), 'k'); hold on;
plot(bla(:, end), bla(:, 3), 'g'); hold on;
plot(bla(:, end), bla(:, 4), 'y'); hold on;
plot(bla(:, end), bla(:, 5), 'c'); hold on;
plot(bla(:, end), bla(:, 6)*180/pi, 'b'); hold on;
plot(bla(:, end), bla(:, 7)*180/pi, 'm'); hold on;
plot(bla(:, end), bla(:, 8)*180/pi, 'k');
legend('mean torque', 'torque', 'current cmd', 'curent left', 'current right', 'theta', 
	'theta_{ref}', 'theta_{error}');

title('Force Responsive Balancing')
