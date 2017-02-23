clear;
r = 1;
R = 5;
agent = [6 0]';
obstacle = [0 0]';
%%
% dVadP = avoidanceFunctionDerivative(agent, obstacle, r, R)
% 
% plot(agent(1), agent(2),'ro',obstacle(1), obstacle(2),'b+');
% rectangle('Position', [obstacle(1)-R, obstacle(2)-R, 2*R,2*R],'Curvature', [1, 1]);
% rectangle('Position', [obstacle(1)-r, obstacle(2)-r, 2*r,2*r],'Curvature', [1, 1]);
% line([agent(1),dVadP(1)+agent(1) ], [agent(2),dVadP(2)+agent(2)])
% 
% grid on
% axis equal;
% axis([-1, 1, -1, 1]);
%%
x = -6:0.001:6;
for i = 1 : length(x)

agent = [x(i) 0]';
obstacle = [0 0]';
dVadP(:, i) = avoidanceFunctionDerivative(agent, obstacle, r, R);

% plot(agent(1), agent(2),'ro',obstacle(1), obstacle(2),'b+');
% rectangle('Position', [obstacle(1)-R, obstacle(2)-R, 2*R,2*R],'Curvature', [1, 1]);
% rectangle('Position', [obstacle(1)-r, obstacle(2)-r, 2*r,2*r],'Curvature', [1, 1]);
% line([agent(1),dVadP(1)+agent(1) ], [agent(2),dVadP(2)+agent(2)])
end
subplot(1,2,1);
plot(x, dVadP(1, :))
grid on
axis([-6, 6, -1.1, 1.1]);
set(gca,'xtick',[-100:1:100])
set(gca,'ytick',[-100:0.5:100])
xlabel('x');
title('linear V_a')
ylabel('$$\mathrm{\frac{\partial V_a}{\partial p}}$$','Interpreter','latex','rot',0);
%%
% x = -6:0.001:6;
% for i = 1 : length(x)
% 
% agent = [x(i) 0]';
% obstacle = [0 0]';
% dVadP(:, i) = avoidanceFunctionDerivative(agent, obstacle, r, R, true);
% 
% % plot(agent(1), agent(2),'ro',obstacle(1), obstacle(2),'b+');
% % rectangle('Position', [obstacle(1)-R, obstacle(2)-R, 2*R,2*R],'Curvature', [1, 1]);
% % rectangle('Position', [obstacle(1)-r, obstacle(2)-r, 2*r,2*r],'Curvature', [1, 1]);
% % line([agent(1),dVadP(1)+agent(1) ], [agent(2),dVadP(2)+agent(2)])
% end
% subplot(1,2,2);
% plot(x, dVadP(1, :))
% grid on
% axis([-6, 6, -1.1*10^6, 1.1*10^6]);
% set(gca,'xtick',[-5:1:5])
% set(gca,'ytick',[-10^6:(10^6/2):10^6])
% xlabel('x');
% title('nonlinear V_a')
% ylabel('$$\mathrm{\frac{\partial V_a}{\partial p}}$$','Interpreter','latex','rot',0);
%%
% grid on
% axis equal;
% axis([-1, 1, -1, 1]);
% N = 200;
% posi = 100000*rand([2 N]);
% posj = 100000*rand([2 N]);
%
% for i = 1 : N
% d(i) = norm(posi(i)-posj(i));
% end
%
% dVadP = zeros(2, N);
% tic
%
% for i = 1 : N
%     if d(i)<R
%         avoidanceFunction(d(i), r, R );
%         dVadP(:, i) = avoidanceFunctionDerivative(d(i), r, R, posj(:, i), posi(:, i));
%     end
% end
% toc
%
%
% tic
% for i = 1 : N
%     avoidanceFunction(d(i), r, R );
%     dVadP(:, i) = avoidanceFunctionDerivative(d(i), r, R, posj(:, i), posi(:, i));
% end
% toc
% dVadP = [100 -500]';
% min(100, max(-100, dVadP))