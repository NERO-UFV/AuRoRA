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
P = Pioneer3DX(1); % Pioneer3DX

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Variáveis iniciais
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);
Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);
L = 10*diag([1 1 1 1 1 1 1 1 1]);

xi = 0;
yi = -1;
zi = 0;
phi = 0;
psi = pi/2;
beta = 0;
theta = 0;
raio(1) = 1.3;
raio(2) = raio(1)*beta;


% phi = -pi/4;
% psi = 0;
% beta = pi/2.5;
% theta = -beta/2;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -% 
% Formação Inicial
Q = [xi; yi; zi;
     theta; phi; psi;
     raio(1); raio(1)/2; beta];
 
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
% T_run = 1/30;       % Tempo de amostragem
T_AMOSTRAGEM = 1/30;
T_PLOT = 0.5;
Ta = tic;
Tp = tic;

pause
T = tic;

A{1}.pPar.ti = tic;
A{1}.pPar.Ts = 1/30;
A{1}.pSC.Kinematics_control = 1;
A{2}.pPar.ti = tic;
A{2}.pPar.Ts = 1/30;
A{2}.pSC.Kinematics_control = 1;

% a

%% SIMULAÇÃO
while toc(T) < T_MAX
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
    
    Qd = [0; yi+(1-yi)*toc(T)/T_MAX; 0;
          -beta*toc(T)/T_MAX; phi; psi;
          raio(1); raio(1)/2; beta];
    dQd = [0; (1+1)/T_MAX; 0;
           -beta/T_MAX; 0; 0;
           0; 0; 0];

%     Qd = [0; yi+(1-yi)*toc(T)/T_MAX; 0;
%           theta; phi+(-phi-phi)*toc(T)/T_MAX; psi;
%           raio(1); raio(1); beta];
%     dQd = [0; (1-yi)/T_MAX; 0;
%            0; (-phi-phi)/T_MAX; 0;
%            0; 0; 0];

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
    
    dQ = dQd + L*tanh(Qtil);
    
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

    J.Inv.pPar{1} = [pf*cos(thetaf)*cos(psif)-pf*sin(thetaf)*sin(phif)*sin(psif) pf*cos(thetaf)*cos(phif)*sin(psif)  -pf*sin(thetaf)*sin(psif)+pf*cos(thetaf)*sin(phif)*cos(psif);
                     pf*cos(thetaf)*sin(psif)+pf*sin(thetaf)*sin(phif)*cos(psif) -pf*cos(thetaf)*cos(phif)*cos(psif) pf*sin(thetaf)*cos(psif)+pf*cos(thetaf)*sin(phif)*sin(psif);
                     -pf*sin(thetaf)*cos(phif)                                   -pf*cos(thetaf)*sin(phif)           0];
                 
    J.Inv.pPar{2} = [sin(thetaf)*cos(psif)+cos(thetaf)*sin(phif)*sin(psif) 0 0;
                     sin(thetaf)*sin(psif)-cos(thetaf)*sin(phif)*cos(psif) 0 0;
                     cos(thetaf)*cos(phif)                                 0 0];
                 
    J.Inv.pPar{3} = [qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif) qf*cos(thetaf+betaf)*cos(phif)*sin(psif)  -qf*sin(thetaf+betaf)*sin(psif)+qf*cos(thetaf+betaf)*sin(phif)*cos(psif);
                     qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif) -qf*cos(thetaf+betaf)*cos(phif)*cos(psif) qf*sin(thetaf+betaf)*cos(psif)+qf*cos(thetaf+betaf)*sin(phif)*sin(psif);
                     -qf*sin(thetaf+betaf)*cos(phif)                                         -qf*cos(thetaf+betaf)*sin(phif)           0];
                 
    J.Inv.pPar{4} = [0 sin(thetaf+betaf)*cos(psif)+cos(thetaf+betaf)*sin(phif)*sin(psif)  qf*cos(thetaf+betaf)*cos(psif)-qf*sin(thetaf+betaf)*sin(phif)*sin(psif);
                      0 sin(thetaf+betaf)*sin(psif)-cos(thetaf+betaf)*sin(phif)*cos(psif) qf*cos(thetaf+betaf)*sin(psif)+qf*sin(thetaf+betaf)*sin(phif)*cos(psif);
                      0 cos(thetaf+betaf)*cos(phif)                                       -qf*sin(thetaf+betaf)*cos(phif)];
    
    J.Inv.pPos = [I O             O;
                  I J.Inv.pPar{1} J.Inv.pPar{2};
                  I J.Inv.pPar{3} J.Inv.pPar{4}];

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

    dX = J.Inv.pPos*dQ;    
%     A{1}.pPos.Xr(7:9) = dX(4:6);
%     A{2}.pPos.Xr(7:9) = dX(7:9);
    
    
%     A{1} = cArDroneSimulacao(A{1});
%     A{2} = cArDroneSimulacao(A{2});
%     
%     A{1}.pPos.dX([7:9 12]) = Ku*A{1}.pSC.Ud - Kv*A{1}.pPos.X([7:9 12]); 
%     A{2}.pPos.dX([7:9 12]) = Ku*A{2}.pSC.Ud - Kv*A{2}.pPos.X([7:9 12]);
% 
%     A{1}.pPos.X([7:9 12]) = A{1}.pPos.dX([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([7:9 12]);
%     A{1}.pPos.X([1:3 6]) = A{1}.pPos.X([7:9 12])*toc(A{1}.pPar.ti) + A{1}.pPos.X([1:3 6]);
% 
%     A{2}.pPos.X([7:9 12]) = A{2}.pPos.dX([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([7:9 12]);
%     A{2}.pPos.X([1:3 6]) = A{2}.pPos.X([7:9 12])*toc(A{2}.pPar.ti) + A{2}.pPos.X([1:3 6]);
    
    
    X = X + T_AMOSTRAGEM*dX;
%     X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];

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
    
%     plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
%     plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
    drawnow
    
end
end


















