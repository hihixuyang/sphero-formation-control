function [ angleError ] = angleError( angle, angleRef,...
                           iPosition, kPosition, jPosition )
%angleError returns the angle error, keeping track of the sign
%  INPUT:
%-angle: angle of the agents
%-angleRef: angle reference for the agents
%-iPositon:
%-kPosition:
%-jPosition:
%OUTPUT:
%-angleError: sigma in the paper

angleError = (angle - angleRef)*signSi(iPosition, kPosition, jPosition);

end

