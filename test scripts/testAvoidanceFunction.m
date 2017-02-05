clear;
r = 10;
R = 50;
N = 200;
posi = 100000*rand([2 N]);
posj = 100000*rand([2 N]);

for i = 1 : N
d(i) = norm(posi(i)-posj(i));
end

dVadP = zeros(2, N);
tic

for i = 1 : N
    if d(i)<R
        avoidanceFunction(d(i), r, R );
        dVadP(:, i) = avoidanceFunctionDerivative(d(i), r, R, posj(:, i), posi(:, i));
    end
end
toc


tic
for i = 1 : N
    avoidanceFunction(d(i), r, R );
    dVadP(:, i) = avoidanceFunctionDerivative(d(i), r, R, posj(:, i), posi(:, i));
end
toc