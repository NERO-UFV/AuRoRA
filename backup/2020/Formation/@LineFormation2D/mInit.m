% Inicializa variáveis
function mInit(obj)
obj.pPos.X    = zeros(12,1); % posição inicial dos robôs [x1 y1 z1 phi1 theta1 psi1 x2 y2 z2 phi2 theta2 psi2]
% obj.pPos.X    = zeros(4,1);  % posição dos robôs [x1 y1 x2 y2]
obj.pPos.dX   = zeros(4,1);  % velocidades dos robôs [dx1 dy1 dx2 dy2]
obj.pPos.Xr   = zeros(4,1);  % posição dos robôs [x1 y1 x2 y2]
obj.pPos.dXr  = zeros(4,1);  % velocidades de referência dos robôs [dxr1 dyr1 dxr2 dyr2]

obj.pPos.Q    = zeros(4,1);  % variáveis da formação [xf yf rof alfaf]
obj.pPos.Qd   = zeros(4,1);  % formação desejada
obj.pPos.dQd  = zeros(4,1);  % derivada da formação desejada
obj.pPos.Qtil = zeros(4,1);  % erro de formação
obj.pPos.dQr  = zeros(4,1);  % formação de referência

obj.pSC.Ur = zeros(4,1);     % sinal de controle [u1 w1 u2 w2];
obj.pSC.Ud = zeros(4,1);     % sinal de controle [u1 w1 u2 w2];


% Gain
% obj.pPar.K1 = 1*diag([.23 .23 0.6 0.5]);          % ganho controlador cinemático
% obj.pPar.K2 = 1*diag([.5 .5 .5 .5]);              % ganho controlador cinemático

% ganho trajetoria - teste marcos
obj.pPar.K1 = 1*diag([.4 .4 0.6 1.5]);          % ganho controlador cinemático
obj.pPar.K2 = 1*diag([.5 .5 .5 .5]);              % ganho controlador cinemático

end

