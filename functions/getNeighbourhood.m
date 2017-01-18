function n = getNeighbourhood(laplacian, d)
%This function calculates the size of the neighbourhood 
%of every vertix of a graph given by its laplacian matrix
n=zeros([D,1]);
for i = 1:d
    tmp = 0;
    for j = 1:d
        if laplacian(i, j) < 0
            tmp = tmp + 1;
        end
    end
    n(i) = tmp;
end
    