function [association, success] = associateSpheros(spheros, spheroPos, cam)
%Designed to find out which tracked robot belongs to which bluetooth object.
%In order to do this all spheros are demanded to have the same color one
%Sphero. Then one after another they are commanded to change its color to a
%rather different value. This change shall expose which tracked robot
%belongs to the bluetooth connection which issued the change color command


%turn off RGB LED for all Spheros for better contrast 
numberSpheros = length(spheros);
for i = 1:numberSpheros
   spheros(i).SetRGBLEDOutput([0, 0, 0], false);  
end

association = zeros(1, numberSpheros);
diff = zeros(1, numberSpheros);

for i = 1:numberSpheros
        %set color to BLUE --> best distinction between blue and red
        spheros(i).SetRGBLEDOutput([0, 0, 1], false);
        pause(0.5);
        
        %take reference snapshot    
        ref_frame = getsnapshot(cam);
        pause(0.1);
%       %frame = insertObjectAnnotation(ref_frame, 'rectangle', [sphero_positions, [40 40 ; 40 40]], [1 2]);
%       imshow(frame);
%       title(['ref frame for sphero i=', num2str(i)])
%       pause();        
        
        %set color to RED --> best possible distinction between blue and red
        spheros(i).SetRGBLEDOutput([1, 0, 0], false);
        pause(0.5);
        
        %take second snapshot for comparison
        sec_frame = getsnapshot(cam);
        pause(0.1);
%       %frame = insertObjectAnnotation(sec_frame, 'rectangle', [sphero_positions, [40 40 ; 40 40]], [1 2]);
%       imshow(frame);
%       title(['sec frame for sphero i=', num2str(i)])
%       pause();     
                
    for j = 1:numberSpheros
        %analyze reference frame (BLUE) for the j-th sphero
        ref_extract = cropFrame(ref_frame, spheroPos(j, :), 100);
%       imshow(extract);
%       title(['extract frame for sphero j=', num2str(j)])
%       pause()
        %calculates the mean value of the RGB colours in the extract
        colourRef = mean(mean(ref_extract));
        blueRef = colourRef(:,:,3);
        redRef = colourRef(:,:,1);
%         
%         fprintf('Sphero position %d', j);
%         disp(spheroPos(j, :))
%         
        %analyze second frame (RED) for the j-th sphero
        sec_extract = cropFrame(sec_frame, spheroPos(j, :), 100);
        colourSec = mean(mean(sec_extract));
        blueSec = colourSec(:,:,3);
        redSec = colourSec(:,:,1);
%         if i == 1
%             k = j - 1;
%         else
%             k = j;
%         end
%         subplot(numberSpheros, numberSpheros, i + k);
%         imshow(extract);        
        diff(j) = (blueRef - blueSec)-(redRef - redSec);
    end
    
%     pause();
    [~, jIndexes] = sort(diff, 'descend');
    %The association vector is composed by the indeces that belong to the
    %biggest differences between reference and measurement frames.
    association(i) = jIndexes(1); 
    spheros(i).SetRGBLEDOutput([0, 0, 0], false); %turn off RGD LED
end

%tests if the association is unique
if length(unique(association)) == length(association)
    disp('Association successful')
    success = true;
else
    disp('Association FAILED')
    association = zeros(1,numberSpheros);
    success = false;
end

%turn on RGB LED for all Spheros for better contrast in tracking
for i = 1:numberSpheros
   spheros(i).SetRGBLEDOutput([1, 1, 1], false);  
end