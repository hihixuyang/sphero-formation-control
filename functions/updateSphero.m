function [ nextState ] = updateSphero( spheroStruct, frame )
global dt;
%updateSphero updates the struct after a new image is acquired
    size = 500;
    %predict the next centroid using the kalmannFilter
    nextCentroid = predict(spheroStruct.kalmannFilter);
    
    %exctract the smallFrame at the predicted location
    [smallFrame, offset] = extractFromFrame(frame, spheroStruct.centroid, size);
%     imshow(smallFrame);
    %update states
    nextState = spheroStruct;
    %update extracted frame data
    nextState.offset = offset;
    nextState.smallFrame = smallFrame;
    %update Centroids and direction
    [centroid, num] = findSphero(smallFrame);
%     pause()
    nextState.lastCentroid = nextState.centroid;
    

    if num > 1
        %if several Spheros are found in a small frame, take the one that
        %is nearest to the predicted location
        for i=1:length(centroid)
           if abs(centroid{i} - nextCentroid) < abs(nextState.centroid - nextCentroid)
               nextState.centroid = centroid{i};
           end
        end
%         nextState.centroid = min((centroid - (ones(1,num) * nextCentroid{})));

        %increase streak
        nextState.streak = nextState.streak + 1;
        
    elseif num == 1
        %If exactly one sphero is found this is the actual centroid
        nextState.centroid = centroid{1} + nextState.offset;
        correct(nextState.kalmannFilter, nextState.centroid);
        %increase streak
        nextState.streak = nextState.streak + 1;
    else
        %If no SPhero is found the predicted location is used as next
        %centroid
        nextState.centroid = nextCentroid;
        %reset streak
        nextState.streak = 0;
    end
    
    
    nextState.heading = wrapTo360(atan2d(nextState.centroid(1,2) -...
                                        nextState.lastCentroid(1,2),...
                                        nextState.centroid(1,1) -...
                                        nextState.lastCentroid(1,1)...
                                    ));
                                
    nextState.velocity = (nextState.centroid - nextState.lastCentroid) ./ dt;
%     nextState.centroid
end

