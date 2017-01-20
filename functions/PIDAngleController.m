function [output, proportionalout, integralout, derivativeout ] = ...
    PIDAngleController( error, dt, PIDGains)
%controller for the multi agent formation control. PID. Inputs should
%controllerOutput is the speed for each Sphero
persistent previousError integral ;
if isempty(previousError)
    previousError = 0;
end
if isempty(integral)
    integral = 0;
end
Kp = PIDGains(1);
Ki = PIDGains(2);
Kd = PIDGains(3);

%actual control

integral = integral + error*dt;
%integral = min(iSaturation, max(-iSaturation, integral));
derivative = (error - previousError) / dt;

proportionalout = Kp * error;
integralout = Ki * integral;
derivativeout = Kd * derivative;

output = proportionalout + integralout + derivativeout;
%output = min(0.5, max(0, output));
%saturation
%output = min(0.4, max(0, output));

previousError = error;
end

