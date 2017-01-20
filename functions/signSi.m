function [ signSi ] = signSi( iPosition, kPosition, jPosition  )
%SIGNSI Considering and angle defined by three points i, k, j given in
%their caertesian coordinates, it returns 1 if the angles is in quadrants 1
%or 2 and -1 if it is in quadrants 3 and 4. Useful for calculating the sign
%of angles obtained by using acos, which is defined only on (0, 180)degrees
xi = iPosition(1);
xk = kPosition(1);
xj = jPosition(1);

yi = iPosition(2);
yk = kPosition(2);
yj = jPosition(2);

S = (xi-xk)*(yj-yk)-(yi-yk)*(xj-xk);

signSi = sign(S);

end

