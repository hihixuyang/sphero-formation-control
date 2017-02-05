function [output, proportionalout, integralout, derivativeout ] = ...
    PIDVelocityController( error, dt, PIDGains, saturation)
%PID controller.
%INPUT:
% -error
% -dt : time step
% -PIDGains: [Kp Ki Kd]
%OUTPUT:
% -output: control output
% -proportionalout
% -integralout
% -derivativeout
%%

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

output = min(saturation, max(-saturation, output));

previousError = error;
end

