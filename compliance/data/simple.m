% This script is to plot the simple 8-dof data which the first 6 values for f/t, 7th for the 
% moving angle.
subject = '0 -90 0 0 0 0 0 - dart - estimation graph - ';
colors = {'k', 'b', 'r', 'g', 'c', 'm'}; 
subplot(2,1,1); 
for i = 1 : 3, plot(1:size(bla,1), bla(1:end,i), colors{i}); hold on; end                        
% hold on; plot(1:size(bla,1), bla(1:end,7), '--g');	
f = bla(1:end,1:3);
g = norm(f, 2, 'rows');
hold on; plot(1:size(bla,1), g, '--c');	
title([subject, 'Forces'], 'FontSize', 20) 	
legend('fx', 'fy', 'fz', 'norm');

subplot(2,1,2); 
for i = 4 : 6, plot(1:size(bla,1), bla(1:end,i), colors{i}); hold on; end                        
f = bla(1:end,4:6);
g = norm(f, 2, 'rows');
hold on; plot(1:size(bla,1), g, '--r');	
title([subject, 'Torques'], 'FontSize', 20) 	
legend('tx', 'ty', 'tz', 'norm');
