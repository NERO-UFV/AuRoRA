% Direct transformation
% Calculate formation variables using robots pose
%
%  + --------------------------------+     +-------------------------------+
%  | x1 y1 z1 | x2 y2 z2 | x3 y3 z3  | ==> |   xF  yF  psi  pF  qF  beta   |
%  |  Desired Formation pose(Xd)     |     |   Formation variables(Qd)     |
%  +---------------------------------+     +-------------------------------+

function mDirTransDesired(obj)

x1 = obj.pPos.Xd(1,1);   
y1 = obj.pPos.Xd(2,1);
z1 = obj.pPos.Xd(3,1);
x2 = obj.pPos.Xd(4,1);
y2 = obj.pPos.Xd(5,1);
z2 = obj.pPos.Xd(6,1);
x3 = obj.pPos.Xd(7,1);   
y3 = obj.pPos.Xd(8,1);
z3 = obj.pPos.Xd(9,1);

xF = (x1+x2+x3)/3;                       % xF
yF = (y1+y2+y3)/3;                       % yF
psi = atan2((y1-yF),(x1-xF));            % psi
pF = sqrt((x1-x2)^2+(y1-y2)^2);          % pF
qF = sqrt((x1-x3)^2+(y1-y3)^2);          % qF
rF = sqrt((x2-x3)^2+(y2-y3)^2);          % rF (auxiliar)
beta = acos((pF^2+qF^2-rF^2)/(2*pF*qF)); % beta

obj.pPos.Qd(1) = xF;
obj.pPos.Qd(2) = yF;
obj.pPos.Qd(3) = psi;
obj.pPos.Qd(4) = pF;
obj.pPos.Qd(5) = qF;
obj.pPos.Qd(6) = beta;

end



% % % function mDirTransDesired(obj)
% % % 
% % %     xf = (Robo(1,1)+Robo(1,2)+Robo(1,3))/3;
% % %     
% % %     yf = (Robo(2,1)+Robo(2,2)+Robo(2,3))/3;
% % %     
% % %     pf = sqrt((Robo(1,1)-Robo(1,2))^2+(Robo(2,1)-Robo(2,2))^2);
% % %     
% % %     qf = sqrt((Robo(1,1)-Robo(1,3))^2+(Robo(2,1)-Robo(2,3))^2);
% % %     
% % %     rf = sqrt((Robo(1,2)-Robo(1,3))^2+(Robo(2,2)-Robo(2,3))^2);
% % %     beta = acos((pf^2+qf^2-rf^2)/(2*pf*qf));
% % %     
% % %     psi = atan2((Robo(2,1)-yf),(Robo(1,1)-xf));
% % %     
% % %     obj.pPos.Xd(1:6) = [xf yf psi pf qf beta]';
% % % 
% % % 
% % % end