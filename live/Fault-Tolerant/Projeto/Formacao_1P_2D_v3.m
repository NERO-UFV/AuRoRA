close all
clear all
clc

tryx
    fclose(instrfindall);
catch
end
%% ADD TO PATH
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

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
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);

Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);
L = diag([1 1 1 1 1 1 1 1 1]);

phi = 0;
psi = 0;
theta = 0;
beta = pi/4;
raio(1) = 0.8;
raio(2) = raio(1)*beta;

X_d2 = (rand-0.5)*5;
if X_d2 >= 0
    X_d2 = X_d2 + raio(1);
else
    X_d2 = X_d2 - raio(1);
end
Y_d2 = (rand-0.5)*5;
if Y_d2 >= 0
    Y_d2 = Y_d2 + raio(1);
else
    Y_d2 = Y_d2 - raio(1);
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
Z_d2 = rand*5 + 2;
Z_d2 = 1.8;

X_d2 = 0;
Y_d2 = -1;

psi = atan2(Y_d2,X_d2) + pi;

% A{2}.pPos.X(1:3) = [X_d2 Y_d2 raio(1)*cos(beta)];
A{2}.pPos.X(1:3) = [X_d2 Y_d2 0];

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Formação Inicial
Q = [0;              % x
     0;              % y
     0;              % z
     phi;            % phi
     psi;            % psi
     theta;          % tetha
     raio(1);           % p
     raio(1);           % q
     beta];          % beta
    
Qd = Q;

% Posição dos Robôs
X = FT_inversa2(Q);
X_c = FT_inversa(Q);
Xd = X;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Esfera limite
E = 0:pi/32:pi;
I = ones(1,2);

X_E = [raio(1)*cos(psi)*cos(E);
       raio(1)*sin(psi)*cos(E);
       raio(1)*sin(E)];
   
X_C = [raio(2)*cos(psi)*I;
       raio(2)*sin(psi)*I;
       [E(1) E(end)]];

% R rotaciona em relação ao eixo-z
R = [cos(pi/10)  -sin(pi/10)  0;
     sin(pi/10)  cos(pi/10)   0;
     0           0            1];

X_Ea = X_E;
X_E = R*X_E;

X_Ca = X_C;
X_C = R*X_C;
 
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

for j=1:10
for i=1:(size(X_E,2)-1)
%     plot3([X_E(1,i) X_E(1,i+1)],[X_E(2,i) X_E(2,i+1)],[X_E(3,i) X_E(3,i+1)],'k');
%     plot3([X_Ea(1,i) X_Ea(1,i+1)],[X_Ea(2,i) X_Ea(2,i+1)],[X_Ea(3,i) X_Ea(3,i+1)],'k');
%     plot3(X_E(1,i),X_E(2,i),X_E(3,i),'r*');
%     plot3(X_Ea(1,i),X_Ea(2,i),X_Ea(3,i),'r*');
    patch([X_Ea(1,i) X_Ea(1,i+1) X_E(1,i+1) X_E(1,i)],...
        [X_Ea(2,i) X_Ea(2,i+1) X_E(2,i+1) X_E(2,i)],...
        [X_Ea(3,i) X_Ea(3,i+1) X_E(3,i+1) X_E(3,i)],'r',...
        'EdgeColor','none','FaceAlpha',0.3);
end
    X_Ea = X_E;
    X_E = R*X_E;
end

for j = 1:10*2

    patch([X_Ca(1,1) X_Ca(1,2) X_C(1,2) X_C(1,1)],...
        [X_Ca(2,1) X_Ca(2,2) X_C(2,2) X_C(2,1)],...
        [X_Ca(3,1) X_Ca(3,2) X_C(3,2) X_C(3,1)],'b',...
        'EdgeColor','none','FaceAlpha',0.3);

    X_Ca = X_C;
    X_C = R*X_C;
end

AX = raio(1)*3;
axis([-AX AX -AX AX 0 Z_d2])
axis equal
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
T_E = [10 5 10 5 10];
T_e = [5];

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
                if toc(t) > T_E(1) || (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 <= raio(2)^2
                    X_A = A{2}.pPos.X(1);
                    Y_A = A{2}.pPos.X(2);
                    if (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 + (A{2}.pPos.X(3) - P.pPos.X(3))^2 <= raio(1)^2
                        Etapa = Etapa + 0.1;
                    else
                        Etapa = Etapa + 0.2;
                    end
                    t = tic;
                end
            case 1.1
                if toc(t) > T_e(1) || (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 + (A{2}.pPos.X(3) - P.pPos.X(3))^2 >= raio(1)^2
                    Etapa = Etapa + 0.9;
                    t = tic;
                end
            case 1.2
                if toc(t) > T_e(1) || (A{2}.pPos.X(1) - P.pPos.X(1))^2 + (A{2}.pPos.X(2) - P.pPos.X(2))^2 + (A{2}.pPos.X(3) - P.pPos.X(3))^2 <= raio(1)^2
                    Etapa = Etapa + 0.8;
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
                    Ang = pi-acos(A{1}.pPos.X(3)/raio(1));
                end
            case 5
                if toc(t) > T_E(5)
                    break
                end
        end
        switch Etapa
            case 1 % Drone 2 voa até o cilindro
                A{2}.pPos.Xd([1:3 6]) = [X_d2 + toc(t)/T_E(1)*(P.pPos.X(1) - X_d2);
                                         Y_d2 + toc(t)/T_E(1)*(P.pPos.X(2) - Y_d2);
                                         Z_d2 + toc(t)/T_E(1)*(P.pPos.X(3) - Z_d2);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(P.pPos.X(1) - X_d2)/T_E(1);
                                          (P.pPos.X(2) - Y_d2)/T_E(1);
                                          (P.pPos.X(3) - Z_d2)/T_E(1);
                                          0];
                A{2}.pPos.Xd([1:3 6]) = [X(7);
                                         X(8);
                                         X(9);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          0;
                                          0];
            case 1.1 % Drone 2 posiciona para iniciar a formação
                A{2}.pPos.Xd([1:3 6]) = [A{2}.pPos.X(1) + toc(t)/T_E(1)*(X_A - A{2}.pPos.X(1));
                                         A{2}.pPos.X(2) + toc(t)/T_E(1)*(Y_A - A{2}.pPos.X(2));
                                         A{2}.pPos.X(3) + toc(t)/T_E(1)*(raio(1)*cos(beta) - A{2}.pPos.X(3));
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(X_A - A{2}.pPos.X(1))/T_E(1);
                                          (Y_A - A{2}.pPos.X(2))/T_E(1);
                                          (raio(1)*cos(beta) - A{2}.pPos.X(3))/T_E(1);
                                          0];
            case 1.2 % Drone 2 posiciona para iniciar a formação
                A{2}.pPos.Xd([1:3 6]) = [A{2}.pPos.X(1) + toc(t)/T_E(1)*(X_A - A{2}.pPos.X(1));
                                         A{2}.pPos.X(2) + toc(t)/T_E(1)*(Y_A - A{2}.pPos.X(2));
                                         A{2}.pPos.X(3) + toc(t)/T_E(1)*(raio(1)*cos(beta) - A{2}.pPos.X(3));
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [(X_A - A{2}.pPos.X(1))/T_E(1);
                                          (Y_A - A{2}.pPos.X(2))/T_E(1);
                                          (raio(1)*cos(beta) - A{2}.pPos.X(3))/T_E(1);
                                          0];
            case 2 % Drone 1 levanta voo e inicia a formação
                A{1}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         0.3225 + (raio(1) - 0.3225)*toc(t)/T_E(2);
                                         0];
                A{1}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          (raio(1) - 0.3225)/T_E(2);
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
                      raio(1);             % p
                      raio(1);             % q
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
                  dX = FT_jacobianoInv2(Q,dQ);
                  
                  A{1}.pPos.Xr([7:9]) = dX([4:6]);
                  A{2}.pPos.Xr([7:9]) = dX([7:9]);
%                  dX = Jinv(Q)*dQ
%                  dX no controle
%                  lê X e acha Q
            case 4 % Drone 2 pousa no pioneer e formação é desfeita
                A{1}.pPos.Xd([1:3 6]) = A{1}.pPos.X([1:3 6]);
                A{1}.pPos.Xd([7:9 12]) = [0 0 0 0]';
                A{2}.pPos.Xd([1:3 6]) = [0;
                                         0;
                                         raio(1) + (0.3225 - raio(1))*toc(t)/T_E(4);
                                         0];
                A{2}.pPos.Xd([7:9 12]) = [0;
                                          0;
                                          (0.3225 - raio(1))/T_E(4);
                                          0];
            case 5 % Drone 1 segue voando, agora em cima do pioneer
                A{1}.pPos.Xd([1:3 6]) = [raio(1)*cos(psi+pi)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         raio(1)*sin(psi+pi)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         raio(1)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                         0];
                A{1}.pPos.Xd([7:9 12]) = [-((pi/2 - Ang)/T_E(5))*raio(1)*cos(psi+pi)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          -((pi/2 - Ang)/T_E(5))*raio(1)*sin(psi+pi)*sin(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
                                          ((pi/2 - Ang)/T_E(5))*raio(1)*cos(Ang + (pi/2 - Ang)*toc(t)/T_E(5));
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
                A{2} = cArDroneSimulacao(A{2});
            case 1.1
                A{2} = cArDroneSimulacao(A{2});
            case 1.2
                A{2} = cArDroneSimulacao(A{2});
            case 2
                A{1} = cArDroneSimulacao(A{1});%% Send control signals
                A{2} = cArDroneSimulacao(A{2});
            case 3
                A{1} = cArDroneSimulacao(A{1});%% Send control signals
                A{2} = cArDroneSimulacao(A{2});
            case 4
                A{1} = cArDroneSimulacao(A{1});%% Send control signals
                A{2} = cArDroneSimulacao(A{2});
            case 5
                A{1} = cArDroneSimulacao(A{1});%% Send control signals
                A{2} = cArDroneSimulacao(A{2});
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
