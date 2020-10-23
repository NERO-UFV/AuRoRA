close all
clear all
clc

try
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
A{1} = ArDrone(1); % Drone em cima do Pioneer3DX
A{2} = ArDrone(2); % Drone voando
A{3} = ArDrone(3);
A{4} = ArDrone(4);
P = Pioneer3DX; % Pioneer3DX

A{1}.pPos.X(1:3) = [0 0 0.3225]; % Altura do modelo do pioneer 0.3225
A{3}.pPos.X = A{1}.pPos.X;
TF = TriangularFormationBaju;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
A{3}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
A{4}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);
Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);
L = diag([1 1 1 1 1 1 1 1 1]);

phi = 0;
psi = 0;
theta = 0;
beta = pi/4;
raio(1) = 0.8;
raio(2) = raio(1)*beta;

X_d2 = 0;
Y_d2 = -1;

psi = atan2(Y_d2,X_d2);

% A{2}.pPos.X(1:3) = [X_d2 Y_d2 raio(1)*cos(beta)];
A{2}.pPos.X(1:3) = [X_d2 Y_d2 0];
A{4}.pPos.X = A{2}.pPos.X;

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
P.pPos.X(1:3) = X(1:3);
A{1}.pPos.X(1:3) = X(4:6);
A{2}.pPos.X(1:3) = X(7:9);
A{3}.pPos.X = A{1}.pPos.X;
A{4}.pPos.X = A{2}.pPos.X;

TF.pPos.X = [P.pPos.X(1:3); A{3}.pPos.X(1:3); A{4}.pPos.X(1:3)];

TF.tDirTrans;
TF.pPos.Qd = TF.pPos.Q;
Xt = X;


%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Figura da simulação
figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'--b','LineWidth',2);
hold on
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'--b','LineWidth',2);
H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'--r','LineWidth',2);
H(4) = plot3([Xt(1) Xt(4)],[Xt(2) Xt(5)],[Xt(3) Xt(6)],'b');
H(5) = plot3([Xt(1) Xt(7)],[Xt(2) Xt(8)],[Xt(3) Xt(9)],'b');
H(6) = plot3([Xt(4) Xt(7)],[Xt(5) Xt(8)],[Xt(6) Xt(9)],'r');
A{1}.mCADcolor([0 0 1])
A{1}.mCADplot;
A{2}.mCADcolor([0 1 0])
A{2}.mCADplot;
P.mCADplot(1,'b');
grid on

axis([-1.5 1.5 -1.5 1.5 0 3])
% axis equal
view([55 25])

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
Tmax = 5;
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

A{1}.pPar.ti = tic;
A{2}.pPar.ti = tic;
A{3}.pPar.ti = tic;
A{4}.pPar.ti = tic;
A{1}.pPar.Ts = 1/30;
A{2}.pPar.Ts = 1/30;
A{3}.pPar.Ts = 1/30;
A{4}.pPar.Ts = 1/30;

A{1}.pSC.Kinematics_control = 1;
A{2}.pSC.Kinematics_control = 1;
%% SIMULAÇÃO
while toc(t) < Tmax
    if toc(t1) > ta
        t1 = tic;
        
        TF.pPos.Qd = [0; 0; 0;
                      -beta*toc(t)/Tmax; theta; psi;
                      raio(1); raio(1); beta];
        TF.pPos.dQd = [0; 0; 0;
                       -beta/Tmax; 0; 0;
                       0; 0; 0];
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % OBJETIVO DA MISSÃO PARA CADA CASO
        Qd = [0;                % x
              0;                % y
              0;                % z
              beta*toc(t)/Tmax; % phi
              psi;              % psi
              theta;            % theta
              raio(1);          % p
              raio(1);          % q
              beta];            % beta
        dQd = [0;               % x
               0;               % y
               0;               % z
               beta/Tmax;       % phi
               0;               % psi
               0;               % tetha
               0;               % p
               0;               % q
               0];              % beta
        Qtil = Qd - Q;
        dQ = dQd + L*Qtil;
        dX = FT_jacobianoInv2(Q,dQ);

        A{1}.pPos.Xr([7:9]) = dX([4:6]);
        A{2}.pPos.Xr([7:9]) = dX([7:9]);
                   
        TF.tFormationControl;
        A{3}.pPos.Xr(7:9) = TF.pPos.dXr(4:6);
        A{4}.pPos.Xr(7:9) = TF.pPos.dXr(7:9);
        %                  dX = Jinv(Q)*dQ
        %                  dX no controle
        %                  lê X e acha Q

        % Recebendo informações dos sensores

        P.rGetSensorData;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
        % CONTROLE PARA CADA CASO

        A{1} = cArDroneSimulacao(A{1});%% Send control signals
        A{2} = cArDroneSimulacao(A{2});

        A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
        A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
        
        A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
        A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
        
        A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
        A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
        
        A{3} = cArDroneSimulacao(A{3});%% Send control signals
        A{4} = cArDroneSimulacao(A{4});

        A{3}.pPos.dX([7:9 12]) = Ku*A{3}.pSC.Ud - Kv*A{3}.pPos.X([7:9 12]); 
        A{4}.pPos.dX([7:9 12]) = Ku*A{4}.pSC.Ud - Kv*A{4}.pPos.X([7:9 12]);
        
        A{3}.pPos.X([7:9 12]) = A{3}.pPos.dX([7:9 12])*toc(A{3}.pPar.ti) + A{3}.pPos.X([7:9 12]);
        A{3}.pPos.X([1:3 6]) = A{3}.pPos.X([7:9 12])*toc(A{3}.pPar.ti) + A{3}.pPos.X([1:3 6]);
        
        A{4}.pPos.X([7:9 12]) = A{4}.pPos.dX([7:9 12])*toc(A{4}.pPar.ti) + A{4}.pPos.X([7:9 12]);
        A{4}.pPos.X([1:3 6]) = A{4}.pPos.X([7:9 12])*toc(A{4}.pPar.ti) + A{4}.pPos.X([1:3 6]);
        
        X([4:6]) = A{1}.pPos.X([1:3]);
        X([7:9]) = A{2}.pPos.X([1:3]);
        Xt([4:6]) = A{3}.pPos.X([1:3]);
        Xt([7:9]) = A{4}.pPos.X([1:3]);
        
        Q = FT_direta2(X);
        Q_c = FT_direta(X);
        
        
        TF.pPos.X = [P.pPos.X(1:3); A{3}.pPos.X(1:3); A{4}.pPos.X(1:3)];

        TF.pPos.Q(1:4) = Q(1:4);
        TF.pPos.Q(5) = Q(6);
        TF.pPos.Q(6) = Q(5);
        TF.pPos.Q(7:9) = Q(7:9);
        
        
        A{1}.pPar.ti = tic;
        A{2}.pPar.ti = tic;
        A{3}.pPar.ti = tic;
        A{4}.pPar.ti = tic;
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
        H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'--b','LineWidth',2);
        H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'--b','LineWidth',2);
        H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'--r','LineWidth',2);
        H(4) = plot3([Xt(1) Xt(4)],[Xt(2) Xt(5)],[Xt(3) Xt(6)],'b');
        H(5) = plot3([Xt(1) Xt(7)],[Xt(2) Xt(8)],[Xt(3) Xt(9)],'b');
        H(6) = plot3([Xt(4) Xt(7)],[Xt(5) Xt(8)],[Xt(6) Xt(9)],'r');
        plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
        plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
        drawnow
    end
end
