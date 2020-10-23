function cInverseDynamicController_Compensador(obj,gains)
if nargin < 2
    %          X     Y     Z    Psi
    gains = [  2     2    3     1.5 ...
               2     2    1.8   5 ...
               1     1     1     1.5 ...
               1     1     1     1];
end

obj.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295  ]';

% Ganhos Dinâmicos
Ku = diag([obj.pPar.Model_simp(1) obj.pPar.Model_simp(3) obj.pPar.Model_simp(5) obj.pPar.Model_simp(7)]);

Kv = diag([obj.pPar.Model_simp(2) obj.pPar.Model_simp(4) obj.pPar.Model_simp(6) obj.pPar.Model_simp(8)]);



% Ganhos Controlador
Ksp = [  gains(1)      0            0          0;
            0        gains(2)       0          0;
            0         0          gains(3)      0;
            0         0             0        gains(4)];

Ksd = [  gains(5)      0            0          0;
            0        gains(6)       0          0;
            0         0          gains(7)      0;
            0         0             0        gains(8)];

Kp = [  gains(9)      0            0          0;
            0        gains(10)       0          0;
            0         0          gains(11)      0;
            0         0             0        gains(12)];

Kd = [  gains(13)      0            0          0;
            0        gains(14)       0          0;
            0         0          gains(15)      0;
            0         0             0        gains(16)];


X = [obj.pPos.X(1:3); obj.pPos.X(6)];   % Posição do robô no mundo
dX = [obj.pPos.X(7:9); obj.pPos.X(12)]; % Velocidade do robô no mundo

Xd = [obj.pPos.Xd(1:3); obj.pPos.Xd(6)]; % Posição Desejada ( Xd Yd Zd Psid )
dXd = [obj.pPos.Xd(7:9); obj.pPos.Xd(12)]; % Velocidade Desejada ( dXd dYd dZd dPsid )
ddXd = [obj.pPos.dXd(7:9); obj.pPos.dXd(12)]; % Aceleração desejada ( ddXd ddYd ddZd ddPsid )

if norm (obj.pPos.Xtil) == 0
    Xtil = Xd - X;
    dXtil = dXd - dX;
else
    Xtil = obj.pPos.Xtil([1:3 6]);
    dXtil = obj.pPos.Xtil([7:9 12]);
end

if abs(Xtil(4)) > pi
    if Xtil(4) > 0
        Xtil(4) = -2*pi + Xtil(4);
    else
        Xtil(4) = 2*pi + Xtil(4);
    end
end

 % Controle cinemático
Ucw_ant = obj.pSC.Ur;

Ucw = (dXd + Ksp*tanh(Kp*Xtil));

if obj.pSC.Kinematics_control == 1
    Ucw(1:3) = obj.pPos.Xr([7 8 9]);
end

dUcw = (Ucw - Ucw_ant)/toc(obj.pSC.tcontrole);

Ucw_ant = Ucw;
obj.pSC.Ur = Ucw_ant;

% Cinemática direta
F = [  cos(X(4))   -sin(X(4))     0     0; 
       sin(X(4))    cos(X(4))     0     0;
          0           0           1     0;
          0           0           0     1];

% Compensador dinâmico
Udw = (F*Ku)\(dUcw + Ksd*(Ucw - dX) + Kv*dX); % Equação de Controle


% Comandos enviados ao Bebop 2
obj.pSC.Ud(1) = Udw(1); % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
obj.pSC.Ud(2) = Udw(2); % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda
obj.pSC.Ud(3) = Udw(3); % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
obj.pSC.Ud(4) = 0; % Não Rotaciona
obj.pSC.Ud(5) = 0; % Não Rotaciona
obj.pSC.Ud(6) = Udw(4); % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z

obj.pSC.tcontrole = tic;

end