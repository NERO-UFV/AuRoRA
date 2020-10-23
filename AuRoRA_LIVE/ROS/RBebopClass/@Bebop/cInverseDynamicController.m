function cInverseDynamicController(obj,model,gains)
if nargin < 3
    %          X     Y     Z    Psi
    gains = [  1     1    1     1 ...
               1.2     1.2   1   1 ...
               1     1     1     1 ...
               1     1     1     1];
end
if nargin < 2
%     disp('Model not given. Using standard ones.');
    model = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ];
end

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

% Ganhos
% v = ddXd + Ksp*tanh(Kp*Xtil) + Ksd*tanh(Kd*dXtil);

k1 = model(1);
k2 = model(2);
k3 = model(3);
k4 = model(4);
k5 = model(5);
k6 = model(6);
k7 = model(7);
k8 = model(8);

X = [obj.pPos.X(1:3); obj.pPos.X(6)];   % Posição do robô no mundo
dX = [obj.pPos.X(7:9); obj.pPos.X(12)]; % Velocidade do robô no mundo

Xd = [obj.pPos.Xd(1:3); obj.pPos.Xd(6)]; % Posição Desejada ( Xd Yd Zd Psid )
dXd = [obj.pPos.Xd(7:9); obj.pPos.Xd(12)]; % Velocidade Desejada ( dXd dYd dZd dPsid )
ddXd = [obj.pPos.dXd(7:9); obj.pPos.dXd(12)]; % Aceleração desejada ( ddXd ddYd ddZd ddPsid )


Xtil = Xd - X;
dXtil = dXd - dX;

v = ddXd + Ksp*tanh(Kp*Xtil) + Ksd*tanh(Kd*dXtil); % 

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
obj.pSC.Ud(1) = U(1); % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
obj.pSC.Ud(2) = U(2); % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda
obj.pSC.Ud(3) = U(3); % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
obj.pSC.Ud(4) = 0; % Não Rotaciona
obj.pSC.Ud(5) = 0; % Não Rotaciona
obj.pSC.Ud(6) = U(4); % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z

end