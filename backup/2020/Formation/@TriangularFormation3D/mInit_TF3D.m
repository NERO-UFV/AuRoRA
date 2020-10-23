% Initialize variables
function mInit_TF3D(obj)
% Posture variables
obj.pPos.X    = zeros(9,1);  % real pose [  x1 y1 z1    x2 y2 z2   x3 y3 z3  ]
obj.pPos.Xd    = zeros(9,1);  % real pose [  x1 y1 z1    x2 y2 z2   x3 y3 z3  ]
obj.pPos.Xr   = zeros(9,1);  % reference pose [  x1 y1 z1    x2 y2 z2   x3 y3 z3  ]

obj.pPos.dX   = zeros(9,1);  % real pose first derivative [dx1 dy1 dz1 dx2 dy2 dz2]
obj.pPos.dXr  = zeros(9,1);  % desired pose first derivative

obj.pPos.Q    = zeros(9,1);  % formation variables [xf yf zf rhof alfaf betaf]
obj.pPos.Qr   = zeros(9,1);  % reference formation
obj.pPos.Qd   = zeros(9,1);  % desired formation   
obj.pPos.Qtil = zeros(9,1);  % formation error
obj.pPos.dQr  = zeros(9,1);  % reference formation first derivative
obj.pPos.dQd  = zeros(9,1);  % desired formation first derivative

% Control Signal variables
obj.pSC.Ur = zeros(8,1);     % reference control signal (sent to dynamic compensator)
obj.pSC.Ud = zeros(8,1);     % desired control signal (sent to robot)

% Parameters variables

obj.pPar.K1 = 3*diag([   1 1 1   1 1 1    1 1 1   ]);    % kinematic control gain  - controls amplitude
obj.pPar.K2 = 1*diag([   1 1 1   1 1 1    1 1 1   ]);        % kinematic control gain - control saturation

obj.pPar.Ts = .050;

end

