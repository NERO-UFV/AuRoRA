close all
clear all
clc

try
    fclose(instrfindall);
catch
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              Load Class                                 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try
    % Load Classes
    RI = RosInterface;
    setenv('ROS_IP','192.168.0.158')
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
    RI.rConnect('192.168.0.146');
    
    % Inicializando o OptiTrack
    OPT = OptiTrack;    % Criando o OptiTrack
    OPT.Initialize;     % Iniciando o OptiTrack

    % Iniciando os Robôs
    A{1} = ArDrone(1); % Drone em cima do Pioneer3DX
    A{2} = ArDrone(2); % Drone voando
    DRONES = [1 0];    % Drones ativos
    P = RPioneer(1,'P1'); % Pìoneer3DX
    
    % Pegando o ID dos corpos rigidos no OptiTrack
    idA{1} = getID(OPT,ArDrone,1);
    idA{2} = getID(OPT,ArDrone,2);
    idP = 1;
    
    % Iniciando a formação triangular
    TF = TriangularFormationBaju;
    
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           INICIO DO PROGRAMA                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Ganhos drones
A{1}.pSC.Kinematics_control = 1;
A{2}.pSC.Kinematics_control = 1;
A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);
Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);

% Ganhos formação
% TF.pPar.K1 = .8*diag([.6  .6  1   1.5   2.2 2.2 2.2 2.2 3]);
TF.pPar.K1 = .8*diag([.6  .6  1   1.5   2.2 2.2 1 1 1.5]);
TF.pPar.K2 =  1*diag([1   1   1   1     1   1   1   1   1]);
%                     x   y   z   theta phi psi p   q   beta

% Ganhos pioneer
pgains = [0.13 0.13 1];
P.pPar.a = 0;
P.pPar.alpha = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           FORMAÇÃO INICIAL                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Parametros
Xfi = 0;
Yfi = 0;
Zfi = 0;
THETAfi = 0;
PHIfi = 0;
PSIfi = pi/2;
BETAfi = pi/3;

% Parametro de proteção
RAIO(1) = 1.2;
RAIO(2) = RAIO(1)*BETAfi;

% Parametros finais da formação
Pfi = RAIO(1);
Qfi = RAIO(1);

% Formação inicial
TF.pPos.Q = [Xfi; Yfi; Zfi;
             THETAfi; PHIfi; PSIfi;
             Pfi; Qfi; BETAfi];

TF.pPos.Qd = TF.pPos.Q;

% Transformada inversa
TF.tInvTrans;

TF.pPos.X = TF.pPos.Xd;

P.pPos.X(1:3) = TF.pPos.X(1:3);
A{1}.pPos.X(1:3) = TF.pPos.X(4:6);
A{2}.pPos.X(1:3) = TF.pPos.X(7:9);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           ROTINA DE CONEXÃO                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for ii = 1:2
    if ii == 2
        A{ii}.pPar.LocalPortControl = 5558;
        A{ii}.pPar.LocalPortState = 5552;
    end
    % Conectando nos Drones
    A{ii}.rConnect;

    % ArDrone Takeoff 
    if DRONES(ii) == 1
        disp('Start Take Off Timming....');
        A{ii}.rTakeOff;
        pause(2)
        disp('Taking Off End Time....');
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             ROTINA DE PLOT                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PLOTT = 0;
X = TF.pPos.X;
XA = X;

if PLOTT == 1
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
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        VARIÁVEIS PARA SIMULAÇÃO                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Marcadores de tempo
T_MAX = 30;                 % Tempo maximo de duração de cada tarefa
T_LAND = 4;                 % Tempo de pouso
T_AMOSTRAGEM = A{1}.pPar.Ts;% Tempo de amostragem
T_PLOT = 0.5;               % Tempo de amostragem do plot

% Constantes da simulação
H_LAND = 0.7;               % Altura para o pouso
BETA_LAND = BETAfi;         % Beta para o pouso
DADOS = [];                 % Iniciando a matriz de dados
MISSAO = [];
ETAPA = 1;
Qd = [];

% Constantes da trajetoria
rx = .6;                    % Raio em relação a x
ry = rx;                    % Raio em relação a y
W = 2*pi/T_MAX;             % Omega

% Iniciando os temporizadores
T_ALFA = tic;               % Temporizador total
T = tic;                    % Temporizador das rotinas
Ta = tic;                   % Temporizador de amostragem
Tp = tic;                   % Temporizador de amostragem de plot
Td = tic;                   % Temporizador de desempenho

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              SIMULAÇÂO                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          FORMAÇÃO DESEJADA                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

    % Missão 1 - Posição inicial da Missão 2
    Qd{1}(:,1) = [Xfi+(rx-Xfi)*toc(T)/(T_MAX/4); Yfi; Zfi;
                  -BETAfi/2; PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];

    Qd{1}(:,2) = [Xfi+(rx-Xfi)*toc(T)/(T_MAX/4); Yfi; Zfi;
                  -BETAfi/2+BETAfi/2*(toc(T)-T_MAX/8)/(T_MAX/8); PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];

    % Missão 2 - Trajetoria em forma de circunferencia
    Qd{2}(:,1) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  THETAfi; PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];

    % Missão 3 - Troca dos drones
    % Drone 1 se afasta para o drone 2 levantar voo
    Qd{3}(:,1) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  -BETAfi*(toc(T)-T_MAX/2)/(T_MAX/2); PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];

    % Drone 2 levanta voo
    Qd{3}(:,2) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  -BETAfi; PHIfi; PSIfi;
                  Pfi; 0.8+(Qfi-0.8)*(toc(T)-T_MAX)/(T_MAX/6); BETAfi];

    % Formação desloca o angulo THETA para o drone 1 ficar em cima do pioneer
    Qd{3}(:,3) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  -BETAfi+BETAfi*(toc(T)-(T_MAX+T_MAX/6))/(T_MAX/2); PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];

    % Drone 1 prepara o pouso
    Qd{3}(:,4) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  THETAfi; PHIfi; PSIfi;
                  Pfi+(H_LAND-Pfi)*(toc(T)-(T_MAX+T_MAX/6+T_MAX/2))/(T_MAX/6); Qfi; BETAfi];

    % Drone 2 continua a missao em cima do pioneer
    Qd{3}(:,5) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  -BETAfi*(toc(T)-(T_MAX+T_MAX/6+T_MAX/2+T_MAX/6))/(T_MAX/6); PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];
              
    % Missão 4 - Trajetoria em forma de circunferencia 2
    Qd{4}(:,1) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  -BETAfi; PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];
              
    % Missão 5 - Pouso no chao
    Qd{5}(:,1) = [Xfi+rx*cos(W*toc(T)); Yfi+ry*sin(W*toc(T)); Zfi;
                  -BETAfi+BETAfi*(toc(T)-T_MAX/2)/(T_MAX/2); PHIfi; PSIfi;
                  Pfi; Qfi; BETAfi];
         
    % Flag das missões
    if size(MISSAO,1) == 0
        for ii = 1:size(Qd,2)
            for jj = 1:size(Qd{ii},2)
                MISSAO(end+1,:) = [ii jj];
            end
        end
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           MISSÃO DESEJADA                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    switch MISSAO(ETAPA,1)
        case 1
            if toc(T) > T_MAX/4
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
                T = tic;
            end
        case 2
            if toc(T) > T_MAX/2
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            end        
        case 3
            if DRONES(2) == 0 && MISSAO(ETAPA,2) == 2
                A{DRONES==0}.rTakeOff
                DRONES = [1 1];
            end
            if DRONES(1) == 1 && toc(T) > T_MAX+T_MAX/6+T_MAX/2+T_MAX/6 && MISSAO(ETAPA,2) == 4
                A{1}.rLand;
                DRONES = [0 1];
            end
            if toc(T) > T_MAX*2
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
                T = tic;
            end
        case 4
            if toc(T) > T_MAX/2
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            end
        case 5
            if toc(T) > T_MAX
                break
            end 
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        POSIÇÃO REAL DOS ROBÔS                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    % Coletando os dados dos corpos rígidos do OptiTrack
    rb = OPT.RigidBody;
    
    % Pioneer
    try
        if rb(idP).isTracked
            P = getOptData(rb(idP),P);
%             P.pPos.X
%             P.pPos.X(3) = 0;
        end
    end
    % Drones
    for ii = find(DRONES==1)
        try
            if rb(idA{ii}).isTracked
                A{ii} = getOptData(rb(idA{ii}),A{ii});
%                A{ii}.pPos.X
            end
        end
    end
    
    TF.pPos.X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          FORMAÇÃO DESEJADA                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
        
    switch MISSAO(ETAPA,1)
        case 1
            if toc(T) > T_MAX/8 && MISSAO(ETAPA,2) == 1
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            end
        case 3
            if toc(T) > T_MAX && MISSAO(ETAPA,2) == 1 % Drone 1 afasta para o drone 2 levantar voo
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            elseif toc(T) > T_MAX+T_MAX/6 && MISSAO(ETAPA,2) == 2 % Drone 2 levanta voo com trajetoria para nao se perder
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            elseif toc(T) > T_MAX+T_MAX/6+T_MAX/2 && MISSAO(ETAPA,2) == 3 % Troca de lugares
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            elseif toc(T) > T_MAX+T_MAX/6+T_MAX/2+T_MAX/6 && MISSAO(ETAPA,2) == 4 % Drone 1 pousa
                ETAPA = ETAPA + 1;
                disp(MISSAO(ETAPA,:))
            end
    end
    
    TF.pPos.Qd = Qd{MISSAO(ETAPA,1)}(:,MISSAO(ETAPA,2));
    TF.pPos.dQd = (TF.pPos.Qd - TF.pPos.QdA)/toc(TF.pPar.ti);
    
    TF.pPar.ti = tic;
    TF.pPos.QdA = TF.pPos.Qd;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            CONTROLADORES                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    % Controle da formação
    TF.tFormationControl;
    TF.tInvTrans; 
    
    % Entregando a referencia para cada robô
    P.pPos.Xd(1:3) = TF.pPos.Xd(1:3);
    P.pPos.Xd(7:9) = TF.pPos.dXr(1:3);
    A{1}.pPos.Xr(7:9) = TF.pPos.dXr(4:6);
    A{2}.pPos.Xr(7:9) = TF.pPos.dXr(7:9);
    
    % Controlador dos robôs
    P = fKinematicControllerExtended(P,pgains);
    for ii = find(DRONES==1)
        A{ii} = cInverseDynamicController_Compensador_ArDrone(A{ii});
    end
    
    % Joystick control
    for ii = find(DRONES==1)
        A{ii} = J.mControl(A{ii});
    end
    
    % Drone virtual
    if sum(DRONES==0) > 0
        A{DRONES==0}.pPos.X(1:3) = TF.pPos.Xd((1:3)+3*find(DRONES==0));
    end
    
%     for ii = 1:2
%         A{ii}.pPos.X = TF.pPos.Xd((1:3)+3*ii);
%     end
%     P.pPos.X = TF.pPos.Xd(1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         ARMAZENANDO OS DADOS                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    DADOS(end+1,:) = [P.pPos.Xd' P.pPos.X' P.pSC.Ud'...
                     A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pPos.Xr' A{1}.pSC.Ud'...
                     A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pPos.Xr' A{2}.pSC.Ud'...
                     TF.pPos.Qd' TF.pPos.Q' TF.pPos.dQd' TF.pPos.dQr'...
                     TF.pPos.Xd' TF.pPos.X' TF.pPos.dXr'...
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
%           TF.pPos.Qd'     TF.pPos.Q'      TF.pPos.dQd'    TF.pPos.dQr'
% 
%           143 -- 151      152 -- 160      161 -- 169
%           TF.pPos.Xd'     TF.pPos.X'      TF.pPos.dXr'
% 
%           170             171'
%           toc(T)          toc(T_ALFA)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        CALCULO DE DESEMPENHO                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %IAE
    IAEt=IAEt+norm(TF.pPos.Qtil)*toc(Td);

    %ITAE
    ITAE=ITAE+norm(TF.pPos.Qtil)*toc(t)*toc(Td);
    
    %IASC
    IASC=IASC+norm(TF.pPos.dXr)*toc(Td);
    
    Td = tic;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         SINAIS DE CONTROLE                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    P.rCommand;
    for ii = find(DRONES==1)
        A{ii}.rSendControlSignals;
    end
        
%     a
end
if toc(Tp) > T_PLOT && PLOTT == 1
    Tp = tic;
    
    try
        delete(H);
        delete(Point);
        delete(TPlot);
    catch
    end
    
    X = TF.pPos.X;
    
    H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
    H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
    H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r','LineWidth',2);
    Point(1) = plot3(X(1),X(2),X(3),'ok','MarkerSize',10,'LineWidth',2);
    Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
    Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
    TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer','FontWeight','bold');
    TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
    TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone2','FontWeight','bold');
    Rastro(1) = plot3([XA(1) X(1)],[XA(2) X(2)],[XA(3) X(3)],'-r');
    Rastro(2) = plot3([XA(4) X(4)],[XA(5) X(5)],[XA(6) X(6)],'-g');
    Rastro(3) = plot3([XA(7) X(7)],[XA(8) X(8)],[XA(9) X(9)],'-b');
    
    XA = X;
%     plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
%     plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
    drawnow
    
end
end

for ii = 1:3
disp('1')
A{1}.rLand;
disp('2')
A{2}.rLand;
end






