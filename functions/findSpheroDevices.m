function spherosToBeConnected = findSpheroDevices()
%It finds all Sphero device names and than asks the user to decide to which 
%of them to connect. The user input is a string of 0 and 1, that represents
%if the Sphero on that position is to be connected or not. Returns a cell
%array of strings with the desired names

bluetoothDevices = instrhwinfo('Bluetooth');%array of all bluetooth devices
bluetoothNames = bluetoothDevices.RemoteNames();%name of all devices
isSphero = strncmp(bluetoothNames, 'Sphero', 6);
%compares strings, the result is 1 if contains Sphero, 0 else
l=0;
for i=1:length(isSphero) %eliminating non-Sphero objects from the list
    if isSphero(i)
        l=l+1;
        availableSpheroNames(l,1)=bluetoothNames(i);
    end
end

disp('Available Spheros:');
availableSpheroNames
prompt = 'Connect to all Spheros ? [Y]/vector : ';
str = input(prompt,'s');

if  isempty(str)% || str == 'Y' 
    spherosToBeConnected = availableSpheroNames;
    return;
else
    user_in = str - '0' %string to array
    l=0;
    for i=1:length(user_in)        
        if user_in(i)
            l=l+1;
            spherosToBeConnected(l,1)=availableSpheroNames(i,1);
        end
    end
end
disp('Spheros to be connected:');
spherosToBeConnected


