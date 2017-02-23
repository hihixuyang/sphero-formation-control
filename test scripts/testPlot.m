%close all
clear all
logCounter = 5;
logPosition(:, 1, 1) = 1:2:10;
logPosition(:, 2, 1) = 1:2:10;
logPosition(:, 1, 2) = 11:2:20;
logPosition(:, 2, 2) = 11:2:20;
N = 2;
hold on
for i = 1 : N
    
   h(i, 1)= plot(logPosition(:, 1, i), logPosition(:, 2, i), '-')
   
   h(i, 2) = plot(logPosition(1, 1, i), logPosition(1, 2, i), 'Marker','o','Color',h(i, 1).Color)
   %hold on
   h(i, 3) = plot(logPosition(end, 1, i), logPosition(end, 2, i), 'Marker','*','Color',h(i, 1).Color)
   %hold on
end
hold off
Legend=cell(N,1);
 for iter=1:N
   Legend{iter}=strcat('agent ', num2str(iter));
 end
 legend(h(:,1), Legend)
