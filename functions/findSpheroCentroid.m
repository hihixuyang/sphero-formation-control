function [centroid, filteredImage] = findSpheroCentroid(frame)
%Finds all Spheros in a given frame and returns their centroids[N*2]
%function already tuned for conditions in the lab

%%
%imshow(frame); pause();
%% Preprocess image

% Convert RGB image to grayscale
grayImage = rgb2gray(frame);
% figure(1); imshow(grayImage); pause();

%% Standard Thresholding

%Binarize image by thresholding, smaller threshold =>more sensitive
binarImage = imbinarize(grayImage, 0.1);
%figure(2); imshow(binarImage); pause();

%Extracts all connected components in the image within the area range
filteredImage = bwareafilt(binarImage, [500 1300]); 
%figure(3); imshow(filteredImage); pause();

properties = regionprops(filteredImage, 'Centroid');
centroid = vertcat(properties(:).Centroid);

%% Hough Transformation by Tao Peng (mathworks file exchange)-slow
% 
% [~, circen, ~] = CircularHough_Grd(grayImage, [15 20], 15);
% centroid = circen;
% filteredImage = grayImage;
% % imshow(filteredImage);
%% imfindcircles method - very slow
% 
% [icenters, ~] = imfindcircles(grayImage,[15 20],...
%     'ObjectPolarity', 'bright', 'Sensitivity', .95, 'EdgeThreshold',0.2);
% % EdgeThreshold 1 = strict 0 = less; Sensitivity 1 = sensible, .85 default
% 
% % imshow(grayImage)
% % viscircles( icenters, iradii,'EdgeColor','g');
% filteredImage = grayImage;
% centroid = icenters;
end