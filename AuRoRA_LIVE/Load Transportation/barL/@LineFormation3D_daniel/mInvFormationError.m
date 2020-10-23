function mInvFormationError(obj)

% Desired Positions .................................................
xfd     = obj.pPos.Qd(1);
yfd     = obj.pPos.Qd(2);
zfd     = obj.pPos.Qd(3);
rhofd   = obj.pPos.Qd(4);
alphafd = obj.pPos.Qd(5);
betafd  = obj.pPos.Qd(6);

% Inverse transform
obj.pPos.Xd(1) = xfd;
obj.pPos.Xd(2) = yfd;
obj.pPos.Xd(3) = zfd;
obj.pPos.Xd(4) = xfd + rhofd*cos(alphafd)*cos(betafd);
obj.pPos.Xd(5) = yfd + rhofd*sin(alphafd)*cos(betafd);
obj.pPos.Xd(6) = zfd + rhofd*sin(betafd);

% Formation Error
obj.pPos.Xtil = obj.pPos.Xd' - obj.pPos.X;


