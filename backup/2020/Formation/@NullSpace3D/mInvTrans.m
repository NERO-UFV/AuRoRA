function mInvTrans(obj)
% Inverse transformation
% Get robots pose using formation variables
% 
%  + ----------------------------+      +---------------------+
%  | [xf yf zf rhof alfaf betaf] | ===> | [x1 y1 z1 x2 y2 z2] |
%  |   Formation variables(Q)    |      |  Formation Pose(X)  |
%  +-----------------------------+      +---------------------+
% 
% xf     = obj.pPos.Q(1);
% yf     = obj.pPos.Q(2);
% zf     = obj.pPos.Q(3);
% rhof   = obj.pPos.Q(4);
% alphaf = obj.pPos.Q(5);
% betaf  = obj.pPos.Q(6);
% 

% % Inverse transform
% obj.pPos.Xr(1) = xf;                               % x1
% obj.pPos.Xr(2) = yf;                               % y1
% obj.pPos.Xr(3) = zf;                               % z1 
% obj.pPos.Xr(4) = xf + rhof*cos(alphaf)*cos(betaf); % x2
% obj.pPos.Xr(5) = yf + rhof*sin(alphaf)*cos(betaf); % y2
% obj.pPos.Xr(6) = zf + rhof*sin(betaf);             % z2

% Desired Positions .................................................
xfd     = obj.pPos.Qd(1);
yfd     = obj.pPos.Qd(2);
zfd     = obj.pPos.Qd(3);
rhofd   = obj.pPos.Qd(4);
alphafd = obj.pPos.Qd(5);
betafd  = obj.pPos.Qd(6);

% Inverse transform
obj.pPos.Xd(1,1) = xfd;
obj.pPos.Xd(2,1) = yfd;
obj.pPos.Xd(3,1) = zfd;
obj.pPos.Xd(4,1) = xfd + rhofd*cos(alphafd)*cos(betafd);
obj.pPos.Xd(5,1) = yfd + rhofd*sin(alphafd)*cos(betafd);
obj.pPos.Xd(6,1) = zfd + rhofd*sin(betafd);

end