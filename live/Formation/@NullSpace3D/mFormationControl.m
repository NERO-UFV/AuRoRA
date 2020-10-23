% Calculate kinematic control velocity
function mFormationControl(obj)
% Direct transformation
obj.mDirTrans;                   % get formation variables

% Formation pose error
obj.mFormationError;

% Inverse Jacobian Matrix  {Formation vector: [xf yf zf rhof alphaf betaf]}
rhof   = obj.pPos.Q(4);
alphaf = obj.pPos.Q(5);
betaf  = obj.pPos.Q(6);

Jinv = [1, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0;
        1, 0, 0, cos(alphaf)*cos(betaf), -sin(alphaf)*cos(betaf)*rhof, -cos(alphaf)*sin(betaf)*rhof;
        0, 1, 0, sin(alphaf)*cos(betaf), cos(alphaf)*cos(betaf)*rhof,  -sin(alphaf)*sin(betaf)*rhof;
        0, 0, 1, sin(betaf),                      0,                            cos(betaf)*rhof];

% Formation velocity
obj.pPos.dQr = obj.pPos.dQd + obj.pPar.K1*tanh(obj.pPar.K2*obj.pPos.Qtil);

% Robots velocities 
obj.pPos.dXr = Jinv*obj.pPos.dQr;

% Modificar para a classe 
obj.pPos.Xr = obj.pPos.Xr + obj.pPos.dXr * obj.SampleTime;

end