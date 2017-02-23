clear all
N=1000;
p1 = [3.2*rand(1,N); 2.4*rand(1,N)];
p2 = [3.2*rand(1,N); 2.4*rand(1,N)];
p3 = [3.2*rand(1,N); 2.4*rand(1,N)];
p12 = sqrt(sum((p1-p2).^2, 1));
figure(1);
histogram(p12,50);

%%
angleRef = 0;
for i = 1:N
[ phi(i), ~ ] = angleErrorGradient(angleRef, p1(:,i), p2(:,i), p3(:,i));
end
figure(2);
histogram(phi,50);