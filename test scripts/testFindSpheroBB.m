load('3framesLEDon');
threshold = 0.35;
%to avoid false recognitions
imshow(frame);
tic;
for i = 1:100
[centers, mask] = findSpheroCentroid(frame);
end;
toc;
timePerFrame = toc/100
imshow(mask);

