function mInvTransAngular(obj)
% Inverse transformation
% Get robots pose using formation variables
% 
%  + ----------------------------+      +---------------------+
%  | [xf yf zf rhof alfaf betaf] | ===> | [x1 y1 z1 x2 y2 z2] |
%  |   Formation variables(Q)    |      |  Formation Pose(X)  |
%  +-----------------------------+      +---------------------+
% 

% Coordenadas Esféricas
% xf     = obj.pPos.Q(1);
% yf     = obj.pPos.Q(2);
% zf     = obj.pPos.Q(3);
% rhof   = obj.pPos.Q(4);
% alphaf = obj.pPos.Q(5);
% betaf  = obj.pPos.Q(6);
% 
% % Inverse transform: Coordenadas Esféricas
% obj.pPos.X(1) = xf;                               % x1
% obj.pPos.X(2) = yf;                               % y1
% obj.pPos.X(3) = zf;                               % z1 
% obj.pPos.X(4) = xf + rhof*cos(alphaf)*cos(betaf); % x2
% obj.pPos.X(5) = yf + rhof*sin(alphaf)*cos(betaf); % y2
% obj.pPos.X(6) = zf + rhof*sin(betaf);             % z2

% Desired Positions .................................................
% Proposta Valentim 2019        
xF     = obj.pPos.Qd(1);
yF     = obj.pPos.Qd(2);
zF     = obj.pPos.Qd(3);
rho    = obj.pPos.Qd(4);
alpha  = obj.pPos.Qd(5);
beta   = obj.pPos.Qd(6);

% Inverse transform
obj.pPos.Xd(1,1) = xF;
obj.pPos.Xd(2,1) = yF;
obj.pPos.Xd(3,1) = zF;
obj.pPos.Xd(4,1) = xF + rho*sin(alpha);
obj.pPos.Xd(5,1) = yF + rho*sin(beta);
obj.pPos.Xd(6,1) = zF + rho*sqrt(1 - (sin(alpha))^2 - (sin(beta))^2);

end