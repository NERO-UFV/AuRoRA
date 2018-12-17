% Initialize variables
function mInit(obj)
% Posture variables
obj.pPos.X    = zeros(6,1);  % real pose [x1 y1 z1 x2 y2 z2]
obj.pPos.Xr   = zeros(6,1);  % reference pose [x1 y1 z1 x2 y2 z2]

obj.pPos.dX   = zeros(6,1);  % real pose first derivative [dx1 dy1 dz1 dx2 dy2 dz2]
obj.pPos.dXr  = zeros(6,1);  % desired pose first derivative


obj.pPos.Q    = zeros(6,1);  % formation variables [xf yf zf rhof alfaf betaf]
obj.pPos.Qd   = zeros(6,1);  % desired formation   
obj.pPos.Qtil = zeros(6,1);  % formation error
obj.pPos.dQr  = zeros(6,1);  % reference formation first derivative
obj.pPos.dQd  = zeros(6,1);  % desired formation first derivative

% Control Signal variables
obj.pSC.Ur = zeros(8,1);     % reference control signal (sent to dynamic compensator)
obj.pSC.Ud = zeros(8,1);     % desired control signal (sent to robot)

% Parameters variables
% obj.pPar.K1 = 0.7*diag([0.5 0.5 0.4 .2 .5 .5]);   % kinematic control gain
% obj.pPar.K2 = 1*diag([.5 .5 .5 .5 .5 .5]);        % kinematic control gain

%
obj.pPar.K1 = 1*diag([0.8 0.8 0.5 0.02 0.015 0.03]);    % kinematic control gain  - controls amplitude
obj.pPar.K2 = 1*diag([0.1 0.1 0.5 0.5 0.5 0.5]);        % kinematic control gain - control saturation

% obj.pPar.K1 = 1*diag([0.7 0.7 0.7 0.8 0.4 0.4]);          % kinematic control gain
% obj.pPar.K2 = 0.5*diag([1 1 1 1 1 1]);  % kinematic control gain
end

