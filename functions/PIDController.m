function [output, proportionalout, integralout, derivativeout ] = ...
    PIDController(error, dt, PIDGains)
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

integral = integral + error*dt;
derivative = (error - previousError) / dt;

proportionalout = Kp * error;
integralout = Ki * integral;
derivativeout = Kd * derivative;

output = proportionalout + integralout + derivativeout;

previousError = error;
end

