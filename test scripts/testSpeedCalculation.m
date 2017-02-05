velocity=[1 2; 3 4; 5 6]';
testNo = 10000;
%%
tic
for i = 1 : testNo
hypot(velocity(1, :), velocity(2, :));
end
toc
%%
tic
for i = 1 : testNo
    for j = 1 :N
norm(velocity(:, j));
    end;
end
toc
%%
tic
for i = 1 : testNo
sqrt(sum(velocity.^2, 1));
end
toc
%%
tic
for i = 1 : testNo
sqrt(sum(velocity.^2, 1));
end
toc
%%
dist = pdist2([0 0; 0 0], [1 1; 3 3 ])