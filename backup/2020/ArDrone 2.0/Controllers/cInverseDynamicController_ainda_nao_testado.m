function drone = cInverseDynamicController_Compensador_ArDrone(drone,gains)
if nargin < 2

%% Não apagar estes ganhos
%     %          X     Y     Z    Psi
%     gains = [  2     2    3     1 ...
%                2     2    1.8   .5 ...
%                1     1     1     1 ...
%                1     1     1     1];

    %          X     Y     Z    Psi
    gains = [  2    2    3     1 ...
               2     2    2   .5 ...
               1.1     1.1     1.1     1 ...
               1     1     1     1];
           
end
if norm(drone.pPar.Model_simp) == 0
%     disp('Model not given. Using standard ones.');
  
    drone.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
end

% Ganhos Dinâmicos
Ku = diag([drone.pPar.Model_simp(1) drone.pPar.Model_simp(3) drone.pPar.Model_simp(5) drone.pPar.Model_simp(7)]);

Kv = diag([drone.pPar.Model_simp(2) drone.pPar.Model_simp(4) drone.pPar.Model_simp(6) drone.pPar.Model_simp(8)]);

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


X = [drone.pPos.X(1:3); drone.pPos.X(6)];   % Posição do robô no mundo
dX = [drone.pPos.X(7:9); drone.pPos.X(12)]; % Velocidade do robô no mundo

Xd = [drone.pPos.Xd(1:3); drone.pPos.Xd(6)]; % Posição Desejada ( Xd Yd Zd Psid )
dXd = [drone.pPos.Xd(7:9); drone.pPos.Xd(12)]; % Velocidade Desejada ( dXd dYd dZd dPsid )
ddXd = [drone.pPos.dXd(7:9); drone.pPos.dXd(12)]; % Aceleração desejada ( ddXd ddYd ddZd ddPsid )

Xtil = Xd - X;

    if abs(Xtil(4)) > pi
        Xtil(4) = -2*pi + Xtil(4);
    end
    if Xtil(4) < -pi
        Xtil(4) = 2*pi + Xtil(4);
    end

v = ddXd + Ksp*tanh(Kp*Xtil) + Ksd*tanh(Kd*dXtil); % 

%      if abs(Xtil(4)) > pi
%          if Xtil(4) < 0
%              Xtil(4) = Xtil(4) + 2*pi;
%          else
%              Xtil(4) = Xtil(4) - 2*pi;
%          end
%      end

 
dXtil = dXd - dX;

F = [  cos(X(4))   -sin(X(4))     0     0; % Cinemática direta
       sin(X(4))    cos(X(4))     0     0;
          0           0           1     0;
          0           0           0     1];


dXc = F\dX; % Velocidade do robô referente ao eixo do robô

f1 = [   k1*cos(X(4))     -k3*sin(X(4))         0         0;
         k1*sin(X(4))      k3*cos(X(4))         0         0;
             0                  0               k5        0;
             0                  0               0         k7];

f2 = [   k2*cos(X(4))     -k4*sin(X(4))         0         0;
         k2*sin(X(4))      k4*cos(X(4))         0         0;
             0                  0               k6        0;
             0                  0               0         k8];

U = (f1)\(v + f2*dXc); % Equação de Controle

% Comandos enviados ao Bebop 2
obj.pSC.Ud(1) = -U(2); % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
obj.pSC.Ud(2) = -U(1); % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda
obj.pSC.Ud(3) = U(3); % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
obj.pSC.Ud(4) = 0; % Não Rotaciona
obj.pSC.Ud(5) = 0; % Não Rotaciona
obj.pSC.Ud(6) = -U(4)/drone.pPar.uSat(4); % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z


end