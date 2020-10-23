% Initialize variables
function mInit(obj)
% Posture variables
obj.pPos.X    = zeros(6,1);  % real pose [x1 y1 z1 x2 y2 z2]
obj.pPos.Xd   = zeros(6,1);  % desired pose [x1 y1 z1 x2 y2 z2]
obj.pPos.Xr   = zeros(6,1);  % reference pose [x1 y1 z1 x2 y2 z2]

obj.pPos.dX   = zeros(6,1);  % real pose first derivative [dx1 dy1 dz1 dx2 dy2 dz2]
obj.pPos.dXr  = zeros(6,1);  % desired pose first derivative

obj.pPos.Q    = zeros(6,1);  % formation variables [xf yf zf rhof alfaf betaf]
obj.pPos.Qr  = zeros(6,1);   % reference formation
obj.pPos.Qd   = zeros(6,1);  % desired formation   
obj.pPos.Qtil = zeros(6,1);  % formation error
obj.pPos.dQr  = zeros(6,1);  % reference formation first derivative
obj.pPos.dQd  = zeros(6,1);  % desired formation first derivative

% Control Signal variables
obj.pSC.Ur = zeros(8,1);     % reference control signal (sent to dynamic compensator)
obj.pSC.Ud = zeros(8,1);     % desired control signal (sent to robot)

%
obj.pPar.K1 = diag([  0.1   0.1   1.0   0.75   0.5   0.5]);     % kinematic control gain  - controls amplitude
obj.pPar.K2 = diag([  0.5   0.5   0.5   0.5    0.25  0.25 ]);   % kinematic control gain - control saturation

obj.pPar.Ts = 0.200; % time sample: time of the formation control loop
end

