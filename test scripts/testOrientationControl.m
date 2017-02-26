positionRef = [5 5; 0 0]';

spheroPos = [2 2; -1 -1]';

[ Vo, dVodP ] = orientationControl(positionRef, spheroPos )

plot (positionRef(1,1), positionRef(2,1), '*', positionRef(1,2),  positionRef(2,2), '*',...
    spheroPos(1,1), spheroPos(2,1), 'o', spheroPos(1,2),  spheroPos(2,2), 'o')
legend('pofref1', 'posref2', 'pos1', 'pos2');
axis([-10, 10, -10, 10])
grid on
% 
% [ dVodP ] = orientationControl(positionRef, spheroPos )