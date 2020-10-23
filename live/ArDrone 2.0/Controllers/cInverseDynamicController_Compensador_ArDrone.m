function drone = cInverseDynamicController_Compensador_ArDrone(drone,gains)

if nargin < 2

%% Não apagar estes ganhos
%     %          X     Y     Z    Psi
%     gains = [  2     2     3     1 ...
%                2     2     2    .5 ...
%               1.1   1.1   1.1    1 ...
%                1     1     1     1];

    %          X     Y     Z    Psi
    gains = [  2     2     3     1 ...
               2     2     2    .05 ...
              1.1   1.1   1.1    1 ...
               1     1     1     .1];
           
end

% AR.Drone parameters  
drone.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';

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

    if abs(Xtil(4)) > pi                       % Yaw Test
        if Xtil(4) < 0
            Xtil(4) =  2*pi + Xtil(4);
        else
            Xtil(4) = -2*pi + Xtil(4);
        end
    end
    
 
dXtil = dXd - dX;

% Controle cinemático
Ucw_ant = drone.pSC.Ur;

Ucw = (dXd + Ksp*tanh(Kp*Xtil));

if drone.pSC.Kinematics_control == 1
    Ucw(1:3) = drone.pPos.Xr([7 8 9]);
end

dUcw = (Ucw - Ucw_ant)/toc(drone.pSC.tcontrole);

Ucw_ant = Ucw;
drone.pSC.Ur = Ucw_ant;

F = [  cos(X(4))   -sin(X(4))     0     0; % Cinemática direta
       sin(X(4))    cos(X(4))     0     0;
          0           0           1     0;
          0           0           0     1];

% Compensador dinâmico
Udw = (F*Ku)\(dUcw + Ksd*(Ucw - dX) + Kv*dX); % Equação de Controle

% Comandos enviados ao ArDrone

drone.pSC.Ud(1) =  -Udw(2);   % Phi
drone.pSC.Ud(2) =  -Udw(1);   % Theta
drone.pSC.Ud(3) =  Udw(3);   % dZ
drone.pSC.Ud(4) = -Udw(4); % dPsi
drone.pSC.Ud(4) = 0;


drone.pSC.Ud = tanh(drone.pSC.Ud);

% drone.pSC.Ud(1) =  drone.pSC.Ud(1)/drone.pPar.uSat(1);   % Phi
% drone.pSC.Ud(2) =  drone.pSC.Ud(2)/drone.pPar.uSat(2);   % Theta
% drone.pSC.Ud(3) =  drone.pSC.Ud(3)/drone.pPar.uSat(3);   % dZ
% drone.pSC.Ud(4) =  drone.pSC.Ud(4)/drone.pPar.uSat(4); % dPsi

drone.pSC.tcontrole = tic;

end