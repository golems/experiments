subplot(2,1,1); colors = {'k', 'b', 'r', 'g', 'c', 'm'}; for i = 1 : 3, plot(1:3:size(bla,1), bla(2:3:end,i), colors{i}); hold on; end                        
title([subject, ' - Unbiased with 0'], 'FontSize', 20) 	
legend('x', 'y', 'z');
subplot(2,1,2); colors = {'k', 'b', 'r', 'g', 'c', 'm'}; for i = 1 : 2, plot(1:3:size(bla,1), bla(3:3:end,i), colors{i}); hold on; end                        
title([subject, ' - Unbiased with initial value'], 'FontSize', 20) 
legend('x', 'y');
