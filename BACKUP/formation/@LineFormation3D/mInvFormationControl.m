% Calculate kinematic control velocity
function mInvFormationControl(obj)

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

%Formation Error
obj.mFormationError;

% Robot Error
obj.mInvFormationError;

% Desired trajectory
obj.pPos.dXd(1) = obj.pPos.dXr(1);
obj.pPos.dXd(2) = obj.pPos.dXr(2);
obj.pPos.dXd(3) = obj.pPos.dXr(3);
obj.pPos.dXd(4) = obj.pPos.dXr(1);
obj.pPos.dXd(5) = obj.pPos.dXr(2);
obj.pPos.dXd(6) = obj.pPos.dXr(3);

% Por que não seria dXr([4 5 6]) ?  [marcos]
% obj.pPos.dXd(4) = obj.pPos.dXr(4);
% obj.pPos.dXd(5) = obj.pPos.dXr(5);
% obj.pPos.dXd(6) = obj.pPos.dXr(6);
% 


% Robots velocities
obj.pPos.dXr = obj.pPos.dXd' + obj.pPar.K1*tanh(obj.pPar.K2*obj.pPos.Xtil);

end