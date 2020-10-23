function drone = cArDroneSimulacao(drone,gains)
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

%      if abs(Xtil(4)) > pi
%          if Xtil(4) < 0
%              Xtil(4) = Xtil(4) + 2*pi;
%          else
%              Xtil(4) = Xtil(4) - 2*pi;
%          end
%      end

 
dXtil = dXd - dX;

% Controle cinemático
Ucw_ant = drone.pSC.Ur;


Ucw = (dXd + Ksp*tanh(Kp*Xtil));

if drone.pSC.Kinematics_control == 1
    Ucw(1:3) = drone.pPos.Xr([7 8 9]);
end

dUcw = (Ucw - Ucw_ant)/toc(drone.pPar.ti);

Ucw_ant = Ucw;
drone.pSC.Ur = Ucw_ant;

F = [  cos(X(4))   -sin(X(4))     0     0; % Cinemática direta
       sin(X(4))    cos(X(4))     0     0;
          0           0           1     0;
          0           0           0     1];

% Compensador dinâmico
Udw = (F*Ku)\(dUcw + Ksd*(Ucw - dX) + Kv*dX); % Equação de Controle

% Comandos enviados ao Bebop 2

drone.pSC.Ud =  Udw;   % Phi

drone.pSC.Ud = tanh(drone.pSC.Ud);
% drone.pPar.ti = tic;
end