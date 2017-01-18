function [ spheros ] = connectSpheros(  )
%Connect all desired Spheros. It returns an array of sphero objects with
%the connected spheros

spherosToBeConnected = findSpheroDevices();
%%
%and establish connection for each sphero device
spherosConnected = 0;
disp('Connecting Spheros...');
for i = 1:length(spherosToBeConnected)
    spheros(i) = Sphero(spherosToBeConnected{i});%connecting
    disp(spherosToBeConnected(i));
    spheros(i).SetRGBLEDOutput([1 1 1], false);%set colour to white
    %TODO check is they are really connected
    spherosConnected = spherosConnected + 1;
end
disp('All Spheros are connected');

% %If all are connected
% if spherosConnected == length(spherosToBeConnected)
%     disp('All Spheros are connected');
% else
%     disp('Not all devices could be connected, deleting them')
%     delete(spheros);     %clear spheros for safety reason
%     return
% end
