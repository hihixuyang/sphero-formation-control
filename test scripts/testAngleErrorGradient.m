clear all
angleRef = -140;
iPosition = [2 2]';
kPosition = [0 0]';
jPosition = [0 3]';
dnx = kPosition(1,1);
dny = kPosition(2,1);
anx = jPosition(1,1);
any = jPosition(2,1);

[phi, Phi] = angleErrorGradient(angleRef, iPosition, kPosition, jPosition)

plot (iPosition(1), iPosition(2), '*',kPosition(1),  kPosition(2), '*',...
    jPosition(1),  jPosition(2), '*')
%     if angleRef > 0
%         line ([dnx 0], [dny (dny-dnx*tand(angleRef+atan2d(any-dny, anx-dnx)))]);
%     else
%         line ([dnx 3.2], [dny dny+(3.2-dnx)*tand(angleRef+atan2d(any-dny, anx-dnx))]);
%     end
grid on
axis([-5, 5, -5, 5])