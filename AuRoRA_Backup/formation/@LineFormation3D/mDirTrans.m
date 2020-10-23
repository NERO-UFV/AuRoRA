% Direct transformation
% Calculate formation variables using robots pose
%
%  + ------------------+      +--------------------------+
%  | x1 y1 z1 x2 y2 z2 | ===> |xf yf zf rhof alphaf betaf|
%  | Formation pose(X) |      |  Formation variables(Q)  |
%  +-------------------+      +--------------------------+

function mDirTrans(obj)

x1 = obj.pPos.X(1,1);   
y1 = obj.pPos.X(2,1);
z1 = obj.pPos.X(3,1);
x2 = obj.pPos.X(4,1);
y2 = obj.pPos.X(5,1);
z2 = obj.pPos.X(6,1);

obj.pPos.Q(1) = x1;                                             % xf
obj.pPos.Q(2) = y1;                                             % yf
obj.pPos.Q(3) = z1;                                             % zf
obj.pPos.Q(4) = sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2);  % rho_f
obj.pPos.Q(5) = atan2((y2 - y1),(x2 - x1)) ;                    % alpha_f
obj.pPos.Q(6) = atan2((z2-z1),sqrt((x2-x1)^2 + (y2-y1)^2));     % beta_f

end