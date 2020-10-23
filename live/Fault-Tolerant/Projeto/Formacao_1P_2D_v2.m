close all
clear all
clc

try
    fclose(instrfindall);
catch
end
%% ADD TO PATH
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% INICIO DO PROGRAMA
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Iniciando os Robôs
A{1} = ArDrone; % Drone em cima do Pioneer3DX
A{2} = ArDrone; % Drone voando
P = Pioneer3DX; % Pioneer3DX

A{1}.pPos.X(1:3) = [0 0 0.3225]; % Altura do modelo do pioneer 0.3225

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);

Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);
L = diag([1 1 1 1 1 1 1 1 1]);

phi = 0;
psi = 0;
theta = 0;
beta = pi/4;
raio = 1.5;

X_d2 = (rand-0.5)*5;
if X_d2 >= 0
    X_d2 = X_d2 + raio;
else
    X_d2 = X_d2 - raio;
end
Y_d2 = (rand-0.5)*5;
if Y_d2 >= 0
    Y_d2 = Y_d2 + raio;
else
    Y_d2 = Y_d2 - raio;
end
Quad = 0;
switch Quad
    case 1
        %1 Quadrante
        X_d2 = 4;
        Y_d2 = 4;
    case 2
        %2 Quadrante
        X_d2 = -4;
        Y_d2 = 4;
    case 3
        %3 Quadrante
        X_d2 = -4;
        Y_d2 = -4;
    case 4
        %4 Quadrante
        X_d2 = 4;
        Y_d2 = -4;
end
Z_d2 = rand*5;
psi = atan2(Y_d2,X_d2) + pi;

% A{2}.pPos.X(1:3) = [X_d2 Y_d2 raio*cos(beta)];
A{2}.pPos.X(1:3) = [X_d2 Y_d2 0];

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Formação Inicial
Q = [0;              % x
     0;              % y
     0;              % z
     phi;            % phi
     psi;            % psi
     theta;          % tetha
     raio;           % p
     raio;           % q
     beta];          % beta
    
Qd = Q;

% Posição dos Robôs
X = FT_inversa2(Q);
X_c = FT_inversa(Q);
Xd = X;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Esfera limite
E = 0:0.1:pi;

X_E = [raio*cos(psi)*cos(E);
       raio*sin(psi)*cos(E);
       raio*sin(E)];
   
% R rotaciona em relação ao eixo-z
R = [cos(pi/10)  -sin(pi/10)  0;
     sin(pi/10)  cos(pi/10)   0;
     0           0            1];
 
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Figura da simulação
figure
% H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b');
hold on
% H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b');
% H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r');
A{1}.mCADplot;
A{2}.mCADplot;
P.mCADplot(1,'b');
grid on

% for i=1:10
% for i=1:(size(X_E,2)-1)
%     plot3([X_E(1,i) X_E(1,i+1)],[X_E(2,i) X_E(2,i+1)],[X_E(3,i) X_E(3,i+1)],'k');
% end
%     X_E = R*X_E;
% end

AX = raio*2;
axis([-AX AX -AX AX 0 AX])
az = psi*180/pi;
el = 45;
view([az el])

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
Tmax = 30;
T_run = 1/30;       % Tempo de amostragem
ta = 0.03;
tp = 0.5;
t1 = tic;
t2 = tic;
% A.pPar.ti = tic;

pause
t = tic;
tPlot = tic;
Etapa = 1;
T_E = [5 5 10 5 10];

A{2}.pPar.ti = tic;
A{1}.pPar.ti = tic;
A{2}.pPar.Ts = 1/30;
A{1}.pPar.Ts = 1/30;

%% SIMULAÇÃO
while toc(t) < Tmax
    if toc(t1) > ta
        t1 = tic;
        
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % OBJETIVO DA MISSÃO PARA CADA CASO
        switch Etapa
            case 1
                if toc(t) > T_E(1)
                    Etapa = Etapa + 1;
                    t = tic;
                end
            case 2
                if toc(t) > T_E(2)
                    Etapa = Etapa + 1;
                    t = tic;
                    A{1}.pSC.Kinematics_control = 1;
                    A{2}.pSC.Kinematics_control = 1;
                end
            case 3
                if toc(t) > T_E(3)
                    Etapa = Etapa + 1;
                    t = tic;
                    A{1}.pSC.Kinematics_control = 0;
                    A{2}.pSC.Kinematics_control = 0;
                end
            case 4
                if toc(t) > T_E(4)
                    Etapa = Etapa + 1;
                    t = tic;
                    Ang = pi-acos(A{1}.pPos.X(3)/raio);
                end
            case 5
                if toc(t) > T_E(5)
                    break
                end
        end
        switch Etapa
            case 1 % Drone 2 voa até a esfera
                A{2}.pPos.Xd([1:3 6]) = [X_d2 + toc(t)/T_E(1)*(X(7) - X_d2);
                                         Y_d2 + toc(t)/T_E(1)*(X(8) - Y_d2);
                                         X(9);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(X(7) - X_d2)/T_E(1);
                                          (X(8) - Y_d2)/T_E(1);
                                          0;
                                          0];
            case 2 % Drone 1 levanta voo e inicia a formação
                A{1}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         0.3225 + (raio - 0.3225)*toc(t)/T_E(2);
                                         0];
                A{1}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          (raio - 0.3225)/T_E(2);
                                          0];
                A{2}.pPos.Xd([1:3 6]) = [X(7);
                                         X(8);
                                         X(9);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          0;
                                          0];
                                         
            case 3 % Formação movimenta colocando o A{1} 2 em cima do pioneer
                Qd = [0;                % x
                      0;                % y
                      0;                % z
                      beta*toc(t)/T_E(3);% phi
                      psi;              % psi
                      theta;            % theta
                      raio;             % p
                      raio;             % q
                      beta];            % beta
                 dQd = [0;              % x
                      0;                % y
                      0;                % z
                      beta/T_E(3);       % phi
                      0;                % psi
                      0;                % tetha
                      0;                % p
                      0;                % q
                      0];               % beta
                  Qtil = Qd - Q;
                  dQ = dQd + L*Qtil;
                  dX = FT_jacobianoInv2(Qd,dQd);
                  dX_c = FT_jacobianoInv(Qd,dQd);
                  A{1}.pPar.Xr([7:9]) = dX([4:6]);
                  A{2}.pPar.Xr([7:9]) = dX([7:9]);
%                  dX = Jinv(Q)*dQ
%                  dX no controle
%                  lê X e acha Q
            case 4 % Drone 2 pousa no pioneer e formação é desfeita
                A{1}.pPos.Xd([1:3 6]) = A{1}.pPos.X([1:3 6]);
                A{1}.pPos.Xd([7:9 12]) = [0 0 0 0]';
                A{2}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         raio + (0.3225 - raio)*toc(t)/T_E(4);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          (0.3225 - raio)/T_E(4);
                                          0];
            case 5 % Drone 1 segue voando, agora em cima do pioneer
                A{1}.pPos.Xd([1:3 6]) = [raio*cos(psi+pi)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         raio*sin(psi+pi)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         raio*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         0];
                A{1}.pPos.Xd([7:9 12]) = [-((pi/2 - Ang)/T_E(5))*raio*cos(psi+pi)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          -((pi/2 - Ang)/T_E(5))*raio*sin(psi+pi)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          ((pi/2 - Ang)/T_E(5))*raio*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          0];
                A{2}.pPos.Xd([1:3 6]) = A{2}.pPos.X([1:3 6]);
                A{2}.pPos.Xd([7:9 12]) = [0 0 0 0]';
        end
        
        % Recebendo informações dos sensores
        
        P.rGetSensorData;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % CONTROLE PARA CADA CASO
        switch Etapa
            case 1
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 2
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 3
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 4
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
            case 5
                A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});%% Send control signals
                A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
        end
        
        A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
        A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
        
        A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
        A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
        
        A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
        A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
        
        if Etapa == 3
            X([4:6]) = A{1}.pPos.X([1:3]);
            X([7:9]) = A{2}.pPos.X([1:3]);
            Q = FT_direta2(X);
            Q_c = FT_direta(X);
        end
        
        A{1}.pPar.ti = tic;
        A{2}.pPar.ti = tic;
    end
    if toc(tPlot) > tp
        tPlot = tic;
        try
            P.mCADdel;
            delete(H);
        end
        
        A{1}.mCADplot;
        A{2}.mCADplot;
        P.mCADplot(1,'b');
        if Etapa == 3
            H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b');
            H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b');
            H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r');
        end
        plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
        plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
        drawnow
    end
end
