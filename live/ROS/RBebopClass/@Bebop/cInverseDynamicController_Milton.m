function cInverseDynamicController_Milton(obj,dXd_max,dX_max,model,gains)

if nargin < 5
    gains = [  0.5   0.5   0.5   0.5 ... %Ks
        1.0   1.0   1.0   1.0];   %Ky
end
if nargin < 4
%     disp('Model not given. Using standard ones.');
    model = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ];
end
if nargin < 3
    dX_max = diag([2.5 2.5 1.0 1.7453]);
end
if nargin < 2
    dXd_max = diag([1.0 1.0 1.0 1.0]);
end

%% Parâmetros iniciais

% Ganhos Dinâmicos
Ku = diag([model(1) model(3) model(5) model(7)]);

Kv = diag([model(2) model(4) model(6) model(8)]);

Ks = diag([gains(1) gains(2) gains(3) gains(4)]);

Ky = diag([gains(5) gains(6) gains(7) gains(8)]);

% Kc = diag([1.0 1.0 1.0 1.0]);
Kc = (dX_max - dXd_max)/dX_max;

X = [obj.pPos.X(1:3); obj.pPos.X(6)];   % Posição do robô no mundo
dX = [obj.pPos.X(7:9); obj.pPos.X(12)]; % Velocidade do robô no mundo

Xd = [obj.pPos.Xd(1:3); obj.pPos.Xd(6)]; % Posição Desejada ( Xd Yd Zd Psid )
dXd = [obj.pPos.Xd(7:9); obj.pPos.Xd(12)]; % Velocidade Desejada ( dXd dYd dZd dPsid )
ddXd = [obj.pPos.dXd(7:9); obj.pPos.dXd(12)]; % Aceleração desejada ( ddXd ddYd ddZd ddPsid )

Xtil = Xd - X;

F = [  cos(X(4))   -sin(X(4))     0     0; % Cinemática direta
    sin(X(4))    cos(X(4))     0     0;
    0           0           1     0;
    0           0           0     1];

Ucw_ant = obj.pSC.Ur;
Ucw = (dXd + Kc*tanh(Ky*Xtil)); % Comando referente ao mundo
dUcw = (Ucw - Ucw_ant)/toc(obj.pPar.ti);

Ucw_ant = Ucw;
obj.pSC.Ur = Ucw_ant;

% Sd = dX_max*tanh(Ucw);
% S = dX_max*tanh(dX);
% Stil = dX_max*tanh((Sd - S));
% 
% Chd = diag([cosh(Ucw(1)) cosh(Ucw(2)) cosh(Ucw(3)) cosh(Ucw(4))]);
% dSd = dX_max/(Chd^(2))*dUcw;

Sd = tanh(Ucw);
S = tanh(dX);
Stil = tanh((Sd - S));

Chd = diag([cosh(Ucw(1)) cosh(Ucw(2)) cosh(Ucw(3)) cosh(Ucw(4))]);
dSd = diag([1.0 1.0 1.0 1.0])/(Chd^(2))*dUcw;

Ud = (F*Ku)\(dSd + Ks*Stil) + (F*Ku)\Kv*Sd;

U = Ud;

% Comandos enviados ao Bebop 2
obj.pSC.Ud(1) = U(1); % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
obj.pSC.Ud(2) = U(2); % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda
obj.pSC.Ud(3) = U(3); % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
obj.pSC.Ud(4) = 0; % Não Rotaciona
obj.pSC.Ud(5) = 0; % Não Rotaciona
obj.pSC.Ud(6) = U(4); % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z

end