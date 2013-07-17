figure;
colors = jet(15);
for i = 1:12;
	plot(bla(:,end), bla(:,i), 'color', colors(i,:)); hold on
end
plot(bla(:,end), bla(:,7)-bla(:,10), 'color', colors(13,:));
plot(bla(:,end), bla(:,8)-bla(:,11), 'color', colors(14,:));
plot(bla(:,end), bla(:,9)-bla(:,12), 'color', colors(15,:));

str{1} = 'Avg Torque due to external force';
str{2} = 'Raw Torque due to external force';
str{3} = 'Left Wheel Current Cmd';
str{4} = 'Right Wheel Current Cmd';
str{5} = 'Left Wheel Raw Current';
str{6} = 'Right Wheel Raw Current';
str{7} = 'Theta';
str{8} = 'x';
str{9} = 'psi';
str{10} = 'Theta_{ref}';
str{11} = 'x_{ref}';
str{12} = 'psi_{ref}';
str{13} = 'Theta_{error}';
str{14} = 'x_{error}';
str{15} = 'psi_{error}';

legend(str);
title('Force Responsive Balancing')
