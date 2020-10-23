close all
clear all
clc

try
    fclose(instrfindall);
catch
end
%% Load Class
try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
    P = RPioneer(1,'P1');
    %     P = Pioneer3DX(1);
    %     idP = getID(OPT,P);
    
    % Joystick
    J = JoyControl;
    
    disp('################### Load Class Success #######################');
    
catch ME
    disp(' ');
    disp(' ################### Load Class Issues #######################');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

%% INICIO DO PROGRAMA
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Inicializando o OptiTrack
OPT = OptiTrack;    % Criando o OptiTrack
OPT.Initialize;     % Iniciando o OptiTrack

% Iniciando os Robôs
A{1} = ArDrone(1); % Drone em cima do Pioneer3DX
A{2} = ArDrone(2); % Drone voando
% P = RPioneer(1); % Pioneer3DX
A{1}.pPar.ti = tic;
A{2}.pPar.ti = tic;
P.pPar.ti = tic;

% Pegando o ID dos corpos rigidos no OptiTrack
idA{1} = getID(OPT,ArDrone,1);
idA{2} = getID(OPT,ArDrone,2);
idP = 1;

% Joystick
J = JoyControl;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);
Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);
K1 = .8*diag([.8  .8  1   1.5   2.2 4.2 2.2 2.2 1.5]);
K2 =  1*diag([1   1   1   1     1   1   1   1   1]);
%             x   y   z   theta phi psi p   q   beta

pgains = [0.13 0.13 1];
P.pPar.a = 0;
P.pPar.alpha = 0;

Qd = zeros(9,1);
Q = zeros(9,1);
dQd = zeros(9,1);
dQ = zeros(9,1);
Xd = zeros(9,1);
X = zeros(9,1);
dX = zeros(9,1);

TRAJETORIA = 2;
% TRAJETORIA
% 1 - 1P-2D - Os drones estão opostos um ao outro em relação ao plano-yz e mantem na região enquanto o pioneer anda pelo eixo-y
% 2 - 1P-2D - Um drone voa por cima do pioneer e troca de lugar com o outro, enquanto o pioneer anda pelo eixo-y

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Condições iniciais da formação
switch TRAJETORIA
    case 1
        % Abrão alas o terror chegou
        xi = 0;
        yi = -1;
        zi = 0;
        phi = -pi/6;
        psi = 0;
        beta = pi/3;
        theta = -beta/2;
        raio(1) = 1.7;
        raio(2) = raio(1)*beta;
    case 2
        % Moicano
        xi = 0;
        yi = -1;
        zi = 0;
        phi = 0;
        psi = pi/2;
        beta = pi/3;
        theta = 0;
        raio(1) = 1.2;
        raio(2) = raio(1)*beta;
end
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Formação Inicial
Q = [xi; yi; zi;
     theta; phi; psi;
     raio(1); raio(1); beta];
 
Qd = Q;
QA = Q;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Armazenando os dados da formação
xf = Q(1);
yf = Q(2);
zf = Q(3);
thetaf = Q(4);
phif = Q(5);
psif = Q(6);
pf = Q(7);
qf = Q(8);
betaf = Q(9);

% Matrizes de rotação
% Rotação em X
Rx = [1 0         0;
      0 cos(phif) -sin(phif);
      0 sin(phif) cos(phif)];
% Rotação em Y
Ry = [cos(thetaf)  0 sin(thetaf);
      0            1 0;
      -sin(thetaf) 0 cos(thetaf)];
% Rotação em Z
Rz = [cos(psif) -sin(psif) 0;
      sin(psif) cos(psif)  0;
      0         0          1];
% Rotação em Y,X e Z nesta ordem
R = Rz*Rx*Ry;

% Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
% respectivamente antes de aplicar as rotações
PF = [0; 0; pf];
QF = [qf*sin(betaf); 0; qf*cos(betaf)];
XF = [xf; yf; zf];

% Posição dos robos
X = [XF;
     R*PF + XF;
     R*QF + XF];

XA = X;
 
P.pPos.X(1:3) = X(1:3);
A{1}.pPos.X(1:3) = X(4:6);
A{2}.pPos.X(1:3) = X(7:9);

% ALTURA LIMITE 1.7
% disp(XA)
% disp(norm(XA(4:6)-XA(7:9)))
% a 


% Conectando nos Drones
A{1}.rConnect;

% ArDrone Takeoff
disp('Start Take Off Timming....');
A{1}.rTakeOff;
% pause(2);
disp('Taking Off End Time....');


A{2}.pPar.LocalPortControl = 5558;
A{2}.pPar.LocalPortState = 5552;
A{2}.rConnect;
% ArDrone Takeoff
disp('Start Take Off Timming....');
A{2}.rTakeOff;
% pause(2);
disp('Taking Off End Time....');

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Plot

figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
hold on
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
H(3) = plot3([X(7) X(4)],[X(8) X(5)],[X(9) X(6)],'r','LineWidth',2);
grid on
Point(1) = plot3(X(1),X(2),X(3),'ok','MarkerSize',10,'LineWidth',2);
Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone2','FontWeight','bold');
axis([-3 3 -3 3 0 3])
view(50,30)

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
T_MAX = 30;
T_LAND = 4;
H_LAND = 0.7;
BETA_LAND = betaf;
% T_run = 1/30;       % Tempo de amostragem
T_AMOSTRAGEM = A{1}.pPar.Ts;
% T_AMOSTRAGEM_P = P.pPar.Ts;
T_PLOT = 40;
Ta = tic;
Tp = tic;
T_ALFA = tic;
DADOS = [];

% pause
T = tic;

A{1}.pPar.ti = tic;
A{1}.pPar.Ts = 1/30;
A{2}.pPar.ti = tic;
A{2}.pPar.Ts = 1/30;

while toc(T) < 15 && J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
    Xd = [XA(1); XA(2); XA(3); XA(4); XA(5); XA(6); XA(7); XA(8); XA(9)];

    P.pPos.Xd(1:3) = Xd(1:3);
    A{1}.pPos.Xd(1:3) = Xd(4:6);
    A{2}.pPos.Xd(1:3) = Xd(7:9);
   
    
    P = fKinematicControllerExtended(P,pgains);
    
    A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
    A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
    
    A{1} = J.mControl(A{1});
    A{2} = J.mControl(A{2});
    
    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;

    % Drone 1
    try
    if rb(idA{1}).isTracked
        A{1} = getOptData(rb(idA{1}),A{1});
%         A{1}.pPos.X
    end
    end
    % Drone 2
    try
    if rb(idA{2}).isTracked
        A{2} = getOptData(rb(idA{2}),A{2});
%         A{2}.pPos.X
    end
    end
    % Pioneer
    try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
%         P.pPos.X
%         P.pPos.X(3) = 0;
    end
    end
    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                      A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                      A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                      Qd' Q' dQd' dQ'...
                      Xd' X' dX'...
                      toc(T) toc(T_ALFA)];

    %           1 -- 12         13 -- 24        25 -- 26        
    %           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
    %
    %           27 -- 38        39 -- 50        51 -- 62        63 -- 66
    %           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
    %
    %           67 -- 78        79 -- 90        91 -- 102       103 -- 106
    %           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
    % 
    %           107 -- 115      116 -- 124      125 -- 133      134 -- 142
    %           Qd'             Q'              dQd'            dQ'
    % 
    %           143 -- 151      152 -- 160      161 -- 169
    %           Xd'             X'              dX'
    % 
    %           170             171
    %           toc(T)          toc(T_ALFA)
    
    P.rCommand;
    A{1}.rSendControlSignals;
    A{2}.rSendControlSignals;
end
end

%% SIMULAÇÃO
% A{1}.rLand;
% A{2}.rLand;

% T_ALFA = tic;
T = tic;

% a
% P.pPar.a = 0;
A{1}.pSC.Kinematics_control = 1;
A{2}.pSC.Kinematics_control = 1;
while toc(T) < T_MAX && J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
    % Condições iniciais da formação
    
    switch TRAJETORIA
        case 1
            % Abrão alas o terror chegou
            Qd = [xi; yi+(1-yi)*toc(T)/T_MAX; zi;
                  theta; phi+(-phi-phi)*toc(T)/T_MAX; psi;
                  raio(1); raio(1); beta];
            dQd = [0; (1-yi)/T_MAX; 0;
                   0; (-phi-phi)/T_MAX; 0;
                   0; 0; 0];
        case 2
            % Moicano
            Qd = [xi; yi+(1-yi)*toc(T)/T_MAX; zi;
                  -beta*toc(T)/T_MAX; phi; psi;
                  raio(1); raio(1); beta];
            dQd = [0; (1-yi)/T_MAX; 0;
                   -beta/T_MAX; 0; 0;
                   0; 0; 0];
    end

    Qtil = Qd - Q;
    
    for ii = [4:6 9]
    if abs(Qtil(ii)) > pi
        if Qtil(ii) > 0
            Qtil(ii) = -2*pi + Qtil(ii);
        else
            Qtil(ii) = 2*pi + Qtil(ii);
        end
    end
    end
    
    dQ = dQd + K1*tanh(K2*Qtil);
    
    xdf = Qd(1);
    ydf = Qd(2);
    zdf = Qd(3);
    thetadf = Qd(4);
    phidf = Qd(5);
    psidf = Qd(6);
    pdf = Qd(7);
    qdf = Qd(8);
    betadf = Qd(9);

    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phidf) -sin(phidf);
          0 sin(phidf) cos(phidf)];
    % Rotação em Y
    Ry = [cos(thetadf)  0 sin(thetadf);
          0            1 0;
          -sin(thetadf) 0 cos(thetadf)];
    % Rotação em Z
    Rz = [cos(psidf) -sin(psidf) 0;
          sin(psidf) cos(psidf)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    R = Rz*Rx*Ry;

    % Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
    % respectivamente antes de aplicar as rotações
    PDF = [0; 0; pdf];
    QDF = [qdf*sin(betadf); 0; qdf*cos(betadf)];
    XDF = [xdf; ydf; zdf];

    % Posição dos robos
    Xd = [XDF;
         R*PDF + XDF;
         R*QDF + XDF];
    
    
    xf = Q(1);
    yf = Q(2);
    zf = Q(3);
    thetaf = Q(4);
    phif = Q(5);
    psif = Q(6);
    pf = Q(7);
    qf = Q(8);
    betaf = Q(9); 
    
    O = [0 0 0;
         0 0 0;
         0 0 0];
    
    I = [1 0 0;
         0 1 0;
         0 0 1];

    Jac.Inv.pPar{1} = [pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif) pf*cos(thetaf)*cos(phif)*sin(psif)  -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif);
                     pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif) -pf*cos(thetaf)*cos(phif)*cos(psif) pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif);
                     -pf*sin(thetaf)*cos(phif)                                   -pf*cos(thetaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{2} = [sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0 0;
                     sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0 0;
                     cos(thetaf)*cos(phif)                                 0 0];
                 
    Jac.Inv.pPar{3} = [qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif);
                     qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif);
                     -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{4} = [0 sin(thetaf+betaf)*cos(psif)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
                      0 sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
                      0 cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];
    
    Jac.Inv.pPos = [I O               O;
                    I Jac.Inv.pPar{1} Jac.Inv.pPar{2};
                    I Jac.Inv.pPar{3} Jac.Inv.pPar{4}];

    %       xf yf zf thetaf                                                                  phif                                      psif                                                                     pf                                                    qf                                                                betaf
%     JInv = [1  0  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  1  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  0  1  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             1  0  0  pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif)             pf*cos(thetaf)*cos(phif)*sin(psif)        -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif)             sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0                                                                 0;
%             0  1  0  pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif)             -pf*cos(thetaf)*cos(phif)*cos(psif)       pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif)              sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0                                                                 0;
%             0  0  1  -pf*sin(thetaf)*cos(phif)                                               -pf*cos(thetaf)*sin(phif)                 0                                                                        cos(thetaf)*cos(phif)                                 0                                                                 0;
%             1  0  0  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif) 0                                                     sin(thetaf+betaf)*cos(psi)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
%             0  1  0  qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif)  0                                                     sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
%             0  0  1  -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0                                                                        0                                                     cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];

    dX = Jac.Inv.pPos*dQ;    
    
%     A{1}.pPos.Xd(1:3) = Xd(4:6);
%     A{2}.pPos.Xd(1:3) = Xd(7:9);
    
    
    P.pPos.Xd(1:3) = Xd(1:3);
    P.pPos.Xd(7:9) = dX(1:3);
    A{1}.pPos.Xr(7:9) = dX(4:6);
    A{2}.pPos.Xr(7:9) = dX(7:9);
    
    
    P = fKinematicControllerExtended(P,pgains);
    
    A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
    A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
    
    A{1} = J.mControl(A{1});
    A{2} = J.mControl(A{2});
    
    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                     A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                     A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                     Qd' Q' dQd' dQ'...
                     Xd' X' dX'...
                     toc(T) toc(T_ALFA)];

%           1 -- 12         13 -- 24        25 -- 26        
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
%
%           27 -- 38        39 -- 50        51 -- 62        63 -- 66
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
%
%           67 -- 78        79 -- 90        91 -- 102       103 -- 106
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
% 
%           107 -- 115      116 -- 124      125 -- 133      134 -- 142
%           Qd'             Q'              dQd'            dQ'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           Xd'             X'              dX'
% 
%           170             171
%           toc(T)          toc(T_ALFA)
    
    P.rCommand;
    A{1}.rSendControlSignals;
    A{2}.rSendControlSignals;
    
%     A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
%     A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
% 
%     A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
%     A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
% 
%     A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
%     A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
    
    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;

    % Drone 1
    try
    if rb(idA{1}).isTracked
        A{1} = getOptData(rb(idA{1}),A{1});
    %     A{1}.pPos.X
    end
    end
    % Drone 2
    try
    if rb(idA{2}).isTracked
        A{2} = getOptData(rb(idA{2}),A{2});
    %     A{2}.pPos.X
    end
    end
    % Pioneer
    try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
    %     P.pPos.X
%         P.pPos.X(3) = 0;
    end
    end
%     X = X + T_AMOSTRAGEM*dX;
%     X = Xd;
    
%     X(1:3) = P.pPos.X(1:3);
%     X(4:6) = A{1}.pPos.X(1:3);
%     X(7:9) = A{2}.pPos.X(1:3);
    X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];

    x = X(1);
    y = X(2);
    z = X(3);
    x1 = X(4);
    y1 = X(5);
    z1 = X(6);
    x2 = X(7);
    y2 = X(8);
    z2 = X(9); 
    
    PA1 = [x1-x y1-y z1-z]';
    PA2 = [x2-x y2-y z2-z]';
    N = cross(PA2,PA1);
    
    phif = atan2(norm(N(1:2)),N(3)) - pi/2;
    psif = pi/2 + atan2(N(2),N(1));
    
    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phif) -sin(phif);
          0 sin(phif) cos(phif)];
    % Rotação em Z
    Rz = [cos(psif) -sin(psif) 0;
          sin(psif) cos(psif)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    PA1 = Rz\PA1;
    PA1 = Rx\PA1;    
    
    thetaf = atan2((PA1(1)-x),(PA1(3)-z));

    pf = sqrt((x-x1)^2 + (y-y1)^2 + (z-z1)^2);
    qf = sqrt((x-x2)^2 + (y-y2)^2 + (z-z2)^2);
    rf = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
    
    Q = [x; y; z;
         thetaf; phif; psif;
         pf; qf; acos((pf^2 + qf^2 - rf^2)/(2*pf*qf))];
%     a
end
if toc(Tp) > T_PLOT
    Tp = tic;
    
    try
        delete(H);
        delete(Point);
        delete(TPlot);
    catch
    end

    H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
    H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
    H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r','LineWidth',2);
    Point(1) = plot3(X(1),X(2),X(3),'ok','MarkerSize',10,'LineWidth',2);
    Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
    Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
    TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer','FontWeight','bold');
    TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
    TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone2','FontWeight','bold');
    Rastro(1) = plot3(X(4),X(5),X(6),'.k');
    Rastro(2) = plot3(X(7),X(8),X(9),'.r');
    
%     plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
%     plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
    drawnow
    
end
end
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Armazenando os dados da formação
xa = Qd(1);
ya = Qd(2);
za = Qd(3);
thetaa = Qd(4);
phia = Qd(5);
psia = Qd(6);
pa = Qd(7);
qa = Qd(8);
betaa = Qd(9);

% T_ALFA = tic;
T = tic;

% A{1}.rLand;
% A{2}.rLand;

%% LAND
while toc(T) < T_LAND && J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
    % Condições iniciais da formação
    
    Qd = [xa; ya; za;
          thetaa; phia; psia;
          pa; qa+(H_LAND-qa)*toc(T)/T_LAND; betaa+(BETA_LAND-betaa)*toc(T)/T_LAND];
    dQd = [0; 0; 0;
           0; 0; 0;
           0; (H_LAND-qa)/T_LAND; (BETA_LAND-betaa)/T_LAND];


    Qtil = Qd - Q;
    
    for ii = [4:6 9]
    if abs(Qtil(ii)) > pi
        if Qtil(ii) > 0
            Qtil(ii) = -2*pi + Qtil(ii);
        else
            Qtil(ii) = 2*pi + Qtil(ii);
        end
    end
    end
    
    dQ = dQd + K1*tanh(K2*Qtil);
    
    xdf = Qd(1);
    ydf = Qd(2);
    zdf = Qd(3);
    thetadf = Qd(4);
    phidf = Qd(5);
    psidf = Qd(6);
    pdf = Qd(7);
    qdf = Qd(8);
    betadf = Qd(9);

    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phidf) -sin(phidf);
          0 sin(phidf) cos(phidf)];
    % Rotação em Y
    Ry = [cos(thetadf)  0 sin(thetadf);
          0            1 0;
          -sin(thetadf) 0 cos(thetadf)];
    % Rotação em Z
    Rz = [cos(psidf) -sin(psidf) 0;
          sin(psidf) cos(psidf)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    R = Rz*Rx*Ry;

    % Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
    % respectivamente antes de aplicar as rotações
    PDF = [0; 0; pdf];
    QDF = [qdf*sin(betadf); 0; qdf*cos(betadf)];
    XDF = [xdf; ydf; zdf];

    % Posição desejada dos robos
    Xd = [XDF;
         R*PDF + XDF;
         R*QDF + XDF];
    
    
    xf = Q(1);
    yf = Q(2);
    zf = Q(3);
    thetaf = Q(4);
    phif = Q(5);
    psif = Q(6);
    pf = Q(7);
    qf = Q(8);
    betaf = Q(9); 
    
    O = [0 0 0;
         0 0 0;
         0 0 0];
    
    I = [1 0 0;
         0 1 0;
         0 0 1];

    Jac.Inv.pPar{1} = [pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif) pf*cos(thetaf)*cos(phif)*sin(psif)  -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif);
                     pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif) -pf*cos(thetaf)*cos(phif)*cos(psif) pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif);
                     -pf*sin(thetaf)*cos(phif)                                   -pf*cos(thetaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{2} = [sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0 0;
                     sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0 0;
                     cos(thetaf)*cos(phif)                                 0 0];
                 
    Jac.Inv.pPar{3} = [qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif);
                     qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif);
                     -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{4} = [0 sin(thetaf+betaf)*cos(psif)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
                      0 sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
                      0 cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];
    
    Jac.Inv.pPos = [I O               O;
                    I Jac.Inv.pPar{1} Jac.Inv.pPar{2};
                    I Jac.Inv.pPar{3} Jac.Inv.pPar{4}];

    %       xf yf zf thetaf                                                                  phif                                      psif                                                                     pf                                                    qf                                                                betaf
%     JInv = [1  0  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  1  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  0  1  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             1  0  0  pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif)             pf*cos(thetaf)*cos(phif)*sin(psif)        -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif)             sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0                                                                 0;
%             0  1  0  pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif)             -pf*cos(thetaf)*cos(phif)*cos(psif)       pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif)              sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0                                                                 0;
%             0  0  1  -pf*sin(thetaf)*cos(phif)                                               -pf*cos(thetaf)*sin(phif)                 0                                                                        cos(thetaf)*cos(phif)                                 0                                                                 0;
%             1  0  0  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif) 0                                                     sin(thetaf+betaf)*cos(psi)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
%             0  1  0  qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif)  0                                                     sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
%             0  0  1  -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0                                                                        0                                                     cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];

    dX = Jac.Inv.pPos*dQ;    
    
%     A{1}.pPos.Xd(1:3) = Xd(4:6);
%     A{2}.pPos.Xd(1:3) = Xd(7:9);
    
    
    P.pPos.Xd(1:3) = Xd(1:3);
    P.pPos.Xd(7:9) = dX(1:3);
    A{1}.pPos.Xr(7:9) = dX(4:6);
    A{2}.pPos.Xr(7:9) = dX(7:9);
    
    
    P = fKinematicControllerExtended(P,pgains);
    P.pSC.Ud = [0; 0];
    
    A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
    A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
    
    A{1} = J.mControl(A{1});
    A{2} = J.mControl(A{2});
        
    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                     A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                     A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                     Qd' Q' dQd' dQ'...
                     Xd' X' dX'...
                     toc(T) toc(T_ALFA)];

%           1 -- 12         13 -- 24        25 -- 26        
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
%
%           27 -- 38        39 -- 50        51 -- 62        63 -- 66
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
%
%           67 -- 78        79 -- 90        91 -- 102       103 -- 106
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
% 
%           107 -- 115      116 -- 124      125 -- 133      134 -- 142
%           Qd'             Q'              dQd'            dQ'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           Xd'             X'              dX'
% 
%           170             171
%           toc(T)          toc(T_ALFA)
    
    P.rCommand;
    A{1}.rSendControlSignals;
    A{2}.rSendControlSignals;
%     
%     A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
%     A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
% 
%     A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
%     A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
% 
%     A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
%     A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
    
    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;

    % Drone 1
    try
    if rb(idA{1}).isTracked
        A{1} = getOptData(rb(idA{1}),A{1});
    %     A{1}.pPos.X
    end
    end
    % Drone 2
    try
    if rb(idA{2}).isTracked
        A{2} = getOptData(rb(idA{2}),A{2});
    %     A{2}.pPos.X
    end
    end
    % Pioneer
    try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
    %     P.pPos.X
%         P.pPos.X(3) = 0;
    end
    end
%     X = X + T_AMOSTRAGEM*dX;
%     X = Xd;
    
%     X(1:3) = P.pPos.X(1:3);
%     X(4:6) = A{1}.pPos.X(1:3);
%     X(7:9) = A{2}.pPos.X(1:3);
    X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];

    x = X(1);
    y = X(2);
    z = X(3);
    x1 = X(4);
    y1 = X(5);
    z1 = X(6);
    x2 = X(7);
    y2 = X(8);
    z2 = X(9); 
    
    PA1 = [x1-x y1-y z1-z]';
    PA2 = [x2-x y2-y z2-z]';
    N = cross(PA2,PA1);
    
    phif = atan2(norm(N(1:2)),N(3)) - pi/2;
    psif = pi/2 + atan2(N(2),N(1));
    
    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phif) -sin(phif);
          0 sin(phif) cos(phif)];
    % Rotação em Z
    Rz = [cos(psif) -sin(psif) 0;
          sin(psif) cos(psif)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    PA1 = Rz\PA1;
    PA1 = Rx\PA1;    
    
    thetaf = atan2((PA1(1)-x),(PA1(3)-z));

    pf = sqrt((x-x1)^2 + (y-y1)^2 + (z-z1)^2);
    qf = sqrt((x-x2)^2 + (y-y2)^2 + (z-z2)^2);
    rf = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
    
    Q = [x; y; z;
         thetaf; phif; psif;
         pf; qf; acos((pf^2 + qf^2 - rf^2)/(2*pf*qf))];
%     a
end
end

% A{1}.rEmergency;
% A{2}.rEmergency;

for ii = 1:3
disp('2')
A{2}.rLand;
disp('1')
A{1}.rLand;
end

pause(5);
A{1}.rTakeOff;
A{2}.rTakeOff;

xi2 = Qd(1);
yi2 = Qd(2);
zi2 = Qd(3);
thetai2 = Qd(4);
phii2 = Qd(5);
psii2 = Qd(6);
pi2 = Qd(7);
qi2 = pi2;
betai2 = Qd(9);

% Matrizes de rotação
% Rotação em X
Rx = [1 0         0;
      0 cos(phii2) -sin(phii2);
      0 sin(phii2) cos(phii2)];
% Rotação em Y
Ry = [cos(thetai2)  0 sin(thetai2);
      0            1 0;
      -sin(thetai2) 0 cos(thetai2)];
% Rotação em Z
Rz = [cos(psii2) -sin(psii2) 0;
      sin(psii2) cos(psii2)  0;
      0         0          1];
% Rotação em Y,X e Z nesta ordem
R = Rz*Rx*Ry;

% Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
% respectivamente antes de aplicar as rotações
PI2 = [0; 0; pi2];
QI2 = [qi2*sin(betai2); 0; qi2*cos(betai2)];
XI2 = [xi2; yi2; zi2];

% Posição desejada dos robos
XA = [XI2;
     R*PI2 + XI2;
     R*QI2 + XI2];
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
T_MAX = 30;
T_LAND = 4;
H_LAND = 0.7;
BETA_LAND = betaf;
% T_run = 1/30;       % Tempo de amostragem
% T_AMOSTRAGEM = A{1}.pPar.Ts;
% T_AMOSTRAGEM_P = P.pPar.Ts;
T_PLOT = 40;
Ta = tic;
Tp = tic;
% T_ALFA = tic;
% DADOS = [];

% pause
T = tic;

A{1}.pSC.Kinematics_control = 0;
A{2}.pSC.Kinematics_control = 0;

A{1}.pPar.ti = tic;
% A{1}.pPar.Ts = 1/30;
A{2}.pPar.ti = tic;
% A{2}.pPar.Ts = 1/30;

while toc(T) < 15 && J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
    Xd = [XA(1); XA(2); XA(3); XA(4); XA(5); XA(6); XA(7); XA(8); XA(9)];

    P.pPos.Xd(1:3) = Xd(1:3);
    A{1}.pPos.Xd(1:3) = Xd(4:6);
    A{2}.pPos.Xd(1:3) = Xd(7:9);
   
    
    P = fKinematicControllerExtended(P,pgains);
    
    A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
    A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
    
    A{1} = J.mControl(A{1});
    A{2} = J.mControl(A{2});
    
    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;

    % Drone 1
    try
    if rb(idA{1}).isTracked
        A{1} = getOptData(rb(idA{1}),A{1});
%         A{1}.pPos.X
    end
    end
    % Drone 2
    try
    if rb(idA{2}).isTracked
        A{2} = getOptData(rb(idA{2}),A{2});
%         A{2}.pPos.X
    end
    end
    % Pioneer
    try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
%         P.pPos.X
%         P.pPos.X(3) = 0;
    end
    end
    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                      A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                      A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                      Qd' Q' dQd' dQ'...
                      Xd' X' dX'...
                      toc(T) toc(T_ALFA)];

    %           1 -- 12         13 -- 24        25 -- 26        
    %           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
    %
    %           27 -- 38        39 -- 50        51 -- 62        63 -- 66
    %           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
    %
    %           67 -- 78        79 -- 90        91 -- 102       103 -- 106
    %           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
    % 
    %           107 -- 115      116 -- 124      125 -- 133      134 -- 142
    %           Qd'             Q'              dQd'            dQ'
    % 
    %           143 -- 151      152 -- 160      161 -- 169
    %           Xd'             X'              dX'
    % 
    %           170             171
    %           toc(T)          toc(T_ALFA)
    
    P.rCommand;
    A{1}.rSendControlSignals;
    A{2}.rSendControlSignals;
end
end

%% SIMULAÇÃO
% A{1}.rLand;
% A{2}.rLand;

% T_ALFA = tic;
T = tic;

% a
% P.pPar.a = 0;
A{1}.pSC.Kinematics_control = 1;
A{2}.pSC.Kinematics_control = 1;
while toc(T) < T_MAX && J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
    % Condições iniciais da formação
    
    switch TRAJETORIA
        case 1
            % Abrão alas o terror chegou
            Qd = [xi2; yi2+(-1-yi2)*toc(T)/T_MAX; zi2;
                  thetai2; phii2+(-phii2-phii2)*toc(T)/T_MAX; psii2;
                  raio(1); raio(1); betai2];
            dQd = [0; (-1-yi2)/T_MAX; 0;
                   0; (-phii2-phii2)/T_MAX; 0;
                   0; 0; 0];
        case 2
            % Moicano
            Qd = [xi2; yi2+(-1-yi2)*toc(T)/T_MAX; zi2;
                  thetai2-thetai2*toc(T)/T_MAX; phii2; psii2;
                  raio(1); raio(1); betai2];
            dQd = [0; (-1-yi2)/T_MAX; 0;
                   -thetai2/T_MAX; 0; 0;
                   0; 0; 0];
    end

    Qtil = Qd - Q;
    
    for ii = [4:6 9]
    if abs(Qtil(ii)) > pi
        if Qtil(ii) > 0
            Qtil(ii) = -2*pi + Qtil(ii);
        else
            Qtil(ii) = 2*pi + Qtil(ii);
        end
    end
    end
    
    dQ = dQd + K1*tanh(K2*Qtil);
    
    xdf = Qd(1);
    ydf = Qd(2);
    zdf = Qd(3);
    thetadf = Qd(4);
    phidf = Qd(5);
    psidf = Qd(6);
    pdf = Qd(7);
    qdf = Qd(8);
    betadf = Qd(9);

    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phidf) -sin(phidf);
          0 sin(phidf) cos(phidf)];
    % Rotação em Y
    Ry = [cos(thetadf)  0 sin(thetadf);
          0            1 0;
          -sin(thetadf) 0 cos(thetadf)];
    % Rotação em Z
    Rz = [cos(psidf) -sin(psidf) 0;
          sin(psidf) cos(psidf)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    R = Rz*Rx*Ry;

    % Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
    % respectivamente antes de aplicar as rotações
    PDF = [0; 0; pdf];
    QDF = [qdf*sin(betadf); 0; qdf*cos(betadf)];
    XDF = [xdf; ydf; zdf];

    % Posição dos robos
    Xd = [XDF;
         R*PDF + XDF;
         R*QDF + XDF];
    
    
    xf = Q(1);
    yf = Q(2);
    zf = Q(3);
    thetaf = Q(4);
    phif = Q(5);
    psif = Q(6);
    pf = Q(7);
    qf = Q(8);
    betaf = Q(9); 
    
    O = [0 0 0;
         0 0 0;
         0 0 0];
    
    I = [1 0 0;
         0 1 0;
         0 0 1];

    Jac.Inv.pPar{1} = [pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif) pf*cos(thetaf)*cos(phif)*sin(psif)  -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif);
                     pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif) -pf*cos(thetaf)*cos(phif)*cos(psif) pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif);
                     -pf*sin(thetaf)*cos(phif)                                   -pf*cos(thetaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{2} = [sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0 0;
                     sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0 0;
                     cos(thetaf)*cos(phif)                                 0 0];
                 
    Jac.Inv.pPar{3} = [qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif);
                     qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif);
                     -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{4} = [0 sin(thetaf+betaf)*cos(psif)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
                      0 sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
                      0 cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];
    
    Jac.Inv.pPos = [I O               O;
                    I Jac.Inv.pPar{1} Jac.Inv.pPar{2};
                    I Jac.Inv.pPar{3} Jac.Inv.pPar{4}];

    %       xf yf zf thetaf                                                                  phif                                      psif                                                                     pf                                                    qf                                                                betaf
%     JInv = [1  0  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  1  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  0  1  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             1  0  0  pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif)             pf*cos(thetaf)*cos(phif)*sin(psif)        -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif)             sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0                                                                 0;
%             0  1  0  pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif)             -pf*cos(thetaf)*cos(phif)*cos(psif)       pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif)              sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0                                                                 0;
%             0  0  1  -pf*sin(thetaf)*cos(phif)                                               -pf*cos(thetaf)*sin(phif)                 0                                                                        cos(thetaf)*cos(phif)                                 0                                                                 0;
%             1  0  0  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif) 0                                                     sin(thetaf+betaf)*cos(psi)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
%             0  1  0  qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif)  0                                                     sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
%             0  0  1  -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0                                                                        0                                                     cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];

    dX = Jac.Inv.pPos*dQ;    
    
%     A{1}.pPos.Xd(1:3) = Xd(4:6);
%     A{2}.pPos.Xd(1:3) = Xd(7:9);
    
    
    P.pPos.Xd(1:3) = Xd(1:3);
    P.pPos.Xd(7:9) = dX(1:3);
    A{1}.pPos.Xr(7:9) = dX(4:6);
    A{2}.pPos.Xr(7:9) = dX(7:9);
    
    
    P = fKinematicControllerExtended(P,pgains);
    
    A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
    A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
    
    A{1} = J.mControl(A{1});
    A{2} = J.mControl(A{2});
    
    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                     A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                     A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                     Qd' Q' dQd' dQ'...
                     Xd' X' dX'...
                     toc(T) toc(T_ALFA)];

%           1 -- 12         13 -- 24        25 -- 26        
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
%
%           27 -- 38        39 -- 50        51 -- 62        63 -- 66
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
%
%           67 -- 78        79 -- 90        91 -- 102       103 -- 106
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
% 
%           107 -- 115      116 -- 124      125 -- 133      134 -- 142
%           Qd'             Q'              dQd'            dQ'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           Xd'             X'              dX'
% 
%           170             171
%           toc(T)          toc(T_ALFA)
    
    P.rCommand;
    A{1}.rSendControlSignals;
    A{2}.rSendControlSignals;
    
%     A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
%     A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
% 
%     A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
%     A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
% 
%     A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
%     A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
    
    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;

    % Drone 1
    try
    if rb(idA{1}).isTracked
        A{1} = getOptData(rb(idA{1}),A{1});
    %     A{1}.pPos.X
    end
    end
    % Drone 2
    try
    if rb(idA{2}).isTracked
        A{2} = getOptData(rb(idA{2}),A{2});
    %     A{2}.pPos.X
    end
    end
    % Pioneer
    try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
    %     P.pPos.X
%         P.pPos.X(3) = 0;
    end
    end
%     X = X + T_AMOSTRAGEM*dX;
%     X = Xd;
    
%     X(1:3) = P.pPos.X(1:3);
%     X(4:6) = A{1}.pPos.X(1:3);
%     X(7:9) = A{2}.pPos.X(1:3);
    X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];

    x = X(1);
    y = X(2);
    z = X(3);
    x1 = X(4);
    y1 = X(5);
    z1 = X(6);
    x2 = X(7);
    y2 = X(8);
    z2 = X(9); 
    
    PA1 = [x1-x y1-y z1-z]';
    PA2 = [x2-x y2-y z2-z]';
    N = cross(PA2,PA1);
    
    phif = atan2(norm(N(1:2)),N(3)) - pi/2;
    psif = pi/2 + atan2(N(2),N(1));
    
    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phif) -sin(phif);
          0 sin(phif) cos(phif)];
    % Rotação em Z
    Rz = [cos(psif) -sin(psif) 0;
          sin(psif) cos(psif)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    PA1 = Rz\PA1;
    PA1 = Rx\PA1;    
    
    thetaf = atan2((PA1(1)-x),(PA1(3)-z));

    pf = sqrt((x-x1)^2 + (y-y1)^2 + (z-z1)^2);
    qf = sqrt((x-x2)^2 + (y-y2)^2 + (z-z2)^2);
    rf = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
    
    Q = [x; y; z;
         thetaf; phif; psif;
         pf; qf; acos((pf^2 + qf^2 - rf^2)/(2*pf*qf))];
%     a
end
if toc(Tp) > T_PLOT
    Tp = tic;
    
    try
        delete(H);
        delete(Point);
        delete(TPlot);
    catch
    end

    H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
    H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
    H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r','LineWidth',2);
    Point(1) = plot3(X(1),X(2),X(3),'ok','MarkerSize',10,'LineWidth',2);
    Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
    Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
    TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer','FontWeight','bold');
    TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
    TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone2','FontWeight','bold');
    Rastro(1) = plot3(X(4),X(5),X(6),'.k');
    Rastro(2) = plot3(X(7),X(8),X(9),'.r');
    
%     plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
%     plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
    drawnow
    
end
end
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Armazenando os dados da formação
xa = Qd(1);
ya = Qd(2);
za = Qd(3);
thetaa = Qd(4);
phia = Qd(5);
psia = Qd(6);
pa = Qd(7);
qa = Qd(8);
betaa = Qd(9);

% T_ALFA = tic;
T = tic;

% A{1}.rLand;
% A{2}.rLand;

%% LAND
while toc(T) < T_LAND && J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
    % Condições iniciais da formação
    
    Qd = [xa; ya; za;
          thetaa; phia; psia;
          pa+(H_LAND-pa)*toc(T)/T_LAND; qa; betaa+(BETA_LAND-betaa)*toc(T)/T_LAND];
    dQd = [0; 0; 0;
           0; 0; 0;
           (H_LAND-qa)/T_LAND; 0; (BETA_LAND-betaa)/T_LAND];


    Qtil = Qd - Q;
    
    for ii = [4:6 9]
    if abs(Qtil(ii)) > pi
        if Qtil(ii) > 0
            Qtil(ii) = -2*pi + Qtil(ii);
        else
            Qtil(ii) = 2*pi + Qtil(ii);
        end
    end
    end
    
    dQ = dQd + K1*tanh(K2*Qtil);
    
    xdf = Qd(1);
    ydf = Qd(2);
    zdf = Qd(3);
    thetadf = Qd(4);
    phidf = Qd(5);
    psidf = Qd(6);
    pdf = Qd(7);
    qdf = Qd(8);
    betadf = Qd(9);

    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phidf) -sin(phidf);
          0 sin(phidf) cos(phidf)];
    % Rotação em Y
    Ry = [cos(thetadf)  0 sin(thetadf);
          0            1 0;
          -sin(thetadf) 0 cos(thetadf)];
    % Rotação em Z
    Rz = [cos(psidf) -sin(psidf) 0;
          sin(psidf) cos(psidf)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    R = Rz*Rx*Ry;

    % Matriz XF, PF e QF, representando a posição do pioneer, drone 1 e drone 2
    % respectivamente antes de aplicar as rotações
    PDF = [0; 0; pdf];
    QDF = [qdf*sin(betadf); 0; qdf*cos(betadf)];
    XDF = [xdf; ydf; zdf];

    % Posição desejada dos robos
    Xd = [XDF;
         R*PDF + XDF;
         R*QDF + XDF];
    
    
    xf = Q(1);
    yf = Q(2);
    zf = Q(3);
    thetaf = Q(4);
    phif = Q(5);
    psif = Q(6);
    pf = Q(7);
    qf = Q(8);
    betaf = Q(9); 
    
    O = [0 0 0;
         0 0 0;
         0 0 0];
    
    I = [1 0 0;
         0 1 0;
         0 0 1];

    Jac.Inv.pPar{1} = [pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif) pf*cos(thetaf)*cos(phif)*sin(psif)  -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif);
                     pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif) -pf*cos(thetaf)*cos(phif)*cos(psif) pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif);
                     -pf*sin(thetaf)*cos(phif)                                   -pf*cos(thetaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{2} = [sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0 0;
                     sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0 0;
                     cos(thetaf)*cos(phif)                                 0 0];
                 
    Jac.Inv.pPar{3} = [qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif);
                     qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif);
                     -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0];
                 
    Jac.Inv.pPar{4} = [0 sin(thetaf+betaf)*cos(psif)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
                      0 sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
                      0 cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];
    
    Jac.Inv.pPos = [I O               O;
                    I Jac.Inv.pPar{1} Jac.Inv.pPar{2};
                    I Jac.Inv.pPar{3} Jac.Inv.pPar{4}];

    %       xf yf zf thetaf                                                                  phif                                      psif                                                                     pf                                                    qf                                                                betaf
%     JInv = [1  0  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  1  0  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             0  0  1  0                                                                       0                                         0                                                                        0                                                     0                                                                 0;
%             1  0  0  pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif)             pf*cos(thetaf)*cos(phif)*sin(psif)        -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif)             sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0                                                                 0;
%             0  1  0  pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif)             -pf*cos(thetaf)*cos(phif)*cos(psif)       pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif)              sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0                                                                 0;
%             0  0  1  -pf*sin(thetaf)*cos(phif)                                               -pf*cos(thetaf)*sin(phif)                 0                                                                        cos(thetaf)*cos(phif)                                 0                                                                 0;
%             1  0  0  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif) 0                                                     sin(thetaf+betaf)*cos(psi)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
%             0  1  0  qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif)  0                                                     sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
%             0  0  1  -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0                                                                        0                                                     cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];

    dX = Jac.Inv.pPos*dQ;    
    
%     A{1}.pPos.Xd(1:3) = Xd(4:6);
%     A{2}.pPos.Xd(1:3) = Xd(7:9);
    
    
    P.pPos.Xd(1:3) = Xd(1:3);
    P.pPos.Xd(7:9) = dX(1:3);
    A{1}.pPos.Xr(7:9) = dX(4:6);
    A{2}.pPos.Xr(7:9) = dX(7:9);
    
    
    P = fKinematicControllerExtended(P,pgains);
    P.pSC.Ud = [0; 0];
    
    A{1} = cInverseDynamicController_Compensador_ArDrone(A{1});
    A{2} = cInverseDynamicController_Compensador_ArDrone(A{2});
    
    A{1} = J.mControl(A{1});
    A{2} = J.mControl(A{2});
        
    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                     A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                     A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                     Qd' Q' dQd' dQ'...
                     Xd' X' dX'...
                     toc(T) toc(T_ALFA)];

%           1 -- 12         13 -- 24        25 -- 26        
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'    
%
%           27 -- 38        39 -- 50        51 -- 62        63 -- 66
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pPos.Xr'   A{1}.pSC.U'
%
%           67 -- 78        79 -- 90        91 -- 102       103 -- 106
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pPos.Xr'   A{2}.pSC.U'
% 
%           107 -- 115      116 -- 124      125 -- 133      134 -- 142
%           Qd'             Q'              dQd'            dQ'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           Xd'             X'              dX'
% 
%           170             171
%           toc(T)          toc(T_ALFA)
    
    P.rCommand;
    A{1}.rSendControlSignals;
    A{2}.rSendControlSignals;
%     
%     A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
%     A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
% 
%     A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
%     A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
% 
%     A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
%     A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
    
    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;

    % Drone 1
    try
    if rb(idA{1}).isTracked
        A{1} = getOptData(rb(idA{1}),A{1});
    %     A{1}.pPos.X
    end
    end
    % Drone 2
    try
    if rb(idA{2}).isTracked
        A{2} = getOptData(rb(idA{2}),A{2});
    %     A{2}.pPos.X
    end
    end
    % Pioneer
    try
    if rb(idP).isTracked
        P = getOptData(rb(idP),P);
    %     P.pPos.X
%         P.pPos.X(3) = 0;
    end
    end
%     X = X + T_AMOSTRAGEM*dX;
%     X = Xd;
    
%     X(1:3) = P.pPos.X(1:3);
%     X(4:6) = A{1}.pPos.X(1:3);
%     X(7:9) = A{2}.pPos.X(1:3);
    X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];

    x = X(1);
    y = X(2);
    z = X(3);
    x1 = X(4);
    y1 = X(5);
    z1 = X(6);
    x2 = X(7);
    y2 = X(8);
    z2 = X(9); 
    
    PA1 = [x1-x y1-y z1-z]';
    PA2 = [x2-x y2-y z2-z]';
    N = cross(PA2,PA1);
    
    phif = atan2(norm(N(1:2)),N(3)) - pi/2;
    psif = pi/2 + atan2(N(2),N(1));
    
    % Matrizes de rotação
    % Rotação em X
    Rx = [1 0         0;
          0 cos(phif) -sin(phif);
          0 sin(phif) cos(phif)];
    % Rotação em Z
    Rz = [cos(psif) -sin(psif) 0;
          sin(psif) cos(psif)  0;
          0         0          1];
    % Rotação em Y,X e Z nesta ordem
    PA1 = Rz\PA1;
    PA1 = Rx\PA1;    
    
    thetaf = atan2((PA1(1)-x),(PA1(3)-z));

    pf = sqrt((x-x1)^2 + (y-y1)^2 + (z-z1)^2);
    qf = sqrt((x-x2)^2 + (y-y2)^2 + (z-z2)^2);
    rf = sqrt((x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2);
    
    Q = [x; y; z;
         thetaf; phif; psif;
         pf; qf; acos((pf^2 + qf^2 - rf^2)/(2*pf*qf))];
%     a
end
end

% A{1}.rEmergency;
% A{2}.rEmergency;

for ii = 1:3
disp('1')
A{1}.rLand;
disp('2')
A{2}.rLand;
end






