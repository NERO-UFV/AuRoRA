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
    
%     RI = RosInterface;
%     setenv('ROS_IP','192.168.0.158')
%     setenv('ROS_MASTER_URI','http://192.168.0.146:11311')
%     RI.rConnect('192.168.0.146');
    
    % Inicializando o OptiTrack
%     OPT = OptiTrack;    % Criando o OptiTrack
%     OPT.Initialize;     % Iniciando o OptiTrack

    % Iniciando os Robôs
    A{1} = ArDrone(1); % Drone em cima do Pioneer3DX
    A{2} = ArDrone(2); % Drone voando
    A{3} = ArDrone(3); % Drone virtual
    A{4} = ArDrone(4); % Drone virtual
    DRONES = [1 1 0 0];    % Drones ativos
    
%     P{1} = RPioneer(1,'P1'); % Pioneer3DX Experimento
%     P{2} = RPioneer(2,'P2'); % Pioneer3DX Experimento
%     P{3} = RPioneer(3,'P3'); % Pioneer3DX Experimento
    P{1} = Pioneer3DX(1); % Pioneer3DX Simulado
    P{2} = Pioneer3DX(2); % Pioneer3DX Simulado
    P{3} = Pioneer3DX(3); % Pioneer3DX Simulado
%     for ii = 1:3
%         P{ii}.rDisableMotors;
%     end

    PIONEER = [1 1 0]; % Pioneer ativos
    
    % Pegando o ID dos corpos rigidos no OptiTrack
%     idA{1} = getID(OPT,ArDrone,1);
%     idA{2} = getID(OPT,ArDrone,2);
%     idP{1} = 1;
%     idP{2} = 2;
%     idP{3} = 3;
    
    % Iniciando a formação triangular
    TF{1} = TriangularFormationBaju(1);
    TF{1}.pPar.R = {1 1 3; 'P' 'A' 'A'};
    
    TF{2} = TriangularFormationBaju(2);
    TF{2}.pPar.R = {2 2 4; 'P' 'A' 'A'};
    
    TF{3} = TriangularFormationBaju(3);
    TF{3}.pPar.R = {3 0 0; 'P' '-' '-'};
    
    FORMACAO = [1 1 0];
    
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
% A{1}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
% A{2}.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
% Ku = diag([A{1}.pPar.Model_simp(1) A{1}.pPar.Model_simp(3) A{1}.pPar.Model_simp(5) A{1}.pPar.Model_simp(7)]);
% Kv = diag([A{1}.pPar.Model_simp(2) A{1}.pPar.Model_simp(4) A{1}.pPar.Model_simp(6) A{1}.pPar.Model_simp(8)]);

% Ganhos formação
TF{1}.pPar.K1 = .8*diag([.6  .6  1   1  4 2.2 2.2 2.2 1]);
% TF{1}.pPar.K1 = .8*diag([.6  .6  1   1.5   2.2 2.2 2.2 2.2 1.5]);
TF{1}.pPar.K2 =  1*diag([1   1   1   1     1   1   1   1   1]);
%                     x   y   z   theta phi psi p   q   beta

% Ganhos pioneer
pgains = [0.13 0.13 1];
for ii = 1:size(PIONEER,2)
    P{ii}.pPar.a = 0;
    P{ii}.pPar.alpha = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           FORMAÇÃO INICIAL                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Posição Formação 1
Xfi(1) = .8;
Yfi(1) = -1.3;
Zfi(1) = 0;

% Posição Formação 2
Xfi(2) = -.8;
Yfi(2) = -1.3;
Zfi(2) = 0;

% Posição Formação 3
Xfi(3) = .8;
Yfi(3) = -1.3;
Zfi(3) = 0;

% Orientação das Formações
THETAfi = 0;
PHIfi = 0;
PSIfi = pi/2;

% Postura das Formações
BETAfi = pi/5.5;

% Parametro de proteção
RAIO(1) = 1.5;
RAIO(2) = RAIO(1)*tan(BETAfi);

% Restante da Postura das formações
Pfi = RAIO(1);
Qfi = RAIO(2)/sin(BETAfi);

% Formação inicial
for ii = find(FORMACAO==1)
    TF{ii}.pPos.Q = [Xfi(ii); Yfi(ii); Zfi(ii);
                     THETAfi; PHIfi; PSIfi;
                     Pfi; Qfi; BETAfi];
    
    TF{ii}.pPos.Qd = TF{ii}.pPos.Q;
    QdA{ii} = TF{ii}.pPos.Qd;
    
    % Transformada inversa
    TF{ii}.tInvTrans;
    
    TF{ii}.pPos.X = TF{ii}.pPos.Xd;
    
    for jj = 1:3
        if TF{ii}.pPar.R{2,jj} == 'P'
            ID = TF{ii}.pPar.R{1,jj};
            P{ID}.pPos.X(1:3) = TF{ii}.pPos.X((1:3)+3*(jj-1));
        elseif TF{ii}.pPar.R{2,jj} == 'A'
            ID = TF{ii}.pPar.R{1,jj};
            A{ID}.pPos.X(1:3) = TF{ii}.pPos.X((1:3)+3*(jj-1));
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           ROTINA DE CONEXÃO                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for ii = 1:2 %size(DRONES,2)
%     if ii == 2
%         A{ii}.pPar.LocalPortControl = 5558;
%         A{ii}.pPar.LocalPortState = 5552;
%     end
%     % Conectando nos Drones
%     A{ii}.rConnect;
% 
%     ArDrone Takeoff 
%     if DRONES(ii) == 1
%         disp('Start Take Off Timming....');
%         A{ii}.rTakeOff;
%         pause(2)
%         disp('Taking Off End Time....');
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             ROTINA DE PLOT                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PLOTT = 1;
X = TF{1}.pPos.X;
X2 = TF{2}.pPos.X;

TF{3}.pPos.X(1:3) = [Xfi(3) Yfi(3) Zfi(3)]';
X3 = TF{3}.pPos.X;

XA = X;
XA2 = X2;
XA3 = X3;

if PLOTT == 1
figure
H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
hold on
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
H(3) = plot3([X(7) X(4)],[X(8) X(5)],[X(9) X(6)],'k','LineWidth',2);
grid on
Point(1) = plot3(X(1),X(2),X(3),'or','MarkerSize',10,'LineWidth',2);
Point(2) = plot3(X(4),X(5),X(6),'^r','MarkerSize',10,'LineWidth',2);
Point(3) = plot3(X(7),X(8),X(9),'^r','MarkerSize',10,'LineWidth',2);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer1','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone3','FontWeight','bold');

H(4) = plot3([X2(1) X2(4)],[X2(2) X2(5)],[X2(3) X2(6)],'b','LineWidth',2);
H(5) = plot3([X2(1) X2(7)],[X2(2) X2(8)],[X2(3) X2(9)],'b','LineWidth',2);
H(6) = plot3([X2(7) X2(4)],[X2(8) X2(5)],[X2(9) X2(6)],'k','LineWidth',2);
Point(4) = plot3(X2(1),X2(2),X2(3),'og','MarkerSize',10,'LineWidth',2);
Point(5) = plot3(X2(4),X2(5),X2(6),'^g','MarkerSize',10,'LineWidth',2);
Point(6) = plot3(X2(7),X2(8),X2(9),'^g','MarkerSize',10,'LineWidth',2);
TPlot(4) = text(X2(1)+.1,X2(2)+.1,X2(3)+.1,'Pioneer2','FontWeight','bold');
TPlot(5) = text(X2(4)+.1,X2(5)+.1,X2(6)+.1,'ArDrone2','FontWeight','bold');
TPlot(6) = text(X2(7)+.1,X2(8)+.1,X2(9)+.1,'ArDrone4','FontWeight','bold');

axis equal
axis([-3 3 -3 3 0 3])
view(50,30)
end
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        VARIÁVEIS PARA SIMULAÇÃO                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Marcadores de tempo
T_MAX = 10;                 % Tempo maximo de duração de cada tarefa
T_LAND = 4;                 % Tempo de pouso
T_AMOSTRAGEM = A{1}.pPar.Ts;% Tempo de amostragem
T_PLOT = 0.01;               % Tempo de amostragem do plot

% Constantes da simulação
H_LAND = 0.5;               % Altura para o pouso
BETA_LAND = BETAfi;         % Beta para o pouso
for ii = 1:size(FORMACAO,2)
    DADOS{ii} = [];                 % Iniciando a matriz de dados
end
etapa = 1;
                
% Constantes da trajetoria
rx = .6;                    % Raio em relação a x
ry = rx;                    % Raio em relação a y
W = 2*pi/(T_MAX*2);             % Omega

% Iniciando os temporizadores
T_ALFA = tic;               % Temporizador total
% for ii = 1:size(FORMACAO,2)
T = tic;            % Temporizador das rotinas
% end
Ta = tic;                   % Temporizador de amostragem
Tp = tic;                   % Temporizador de amostragem de plot

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              SIMULAÇÂO                                  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% try

% for ii = find(PIONEER==1)
%     P{ii}.rEnableMotors;
% end
    
while J.pFlag == 0
if toc(Ta) > T_AMOSTRAGEM
    Ta = tic;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           MISSÃO DESEJADA                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            ETAPA DESEJADA                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

    switch etapa
        case 1
            if toc(T) > T_MAX
                etapa = etapa + 1;
                T = tic;
                                
                TF{1}.pPos.Qd(2) = abs(Yfi(1));
                TF{2}.pPos.Qd(2) = abs(Yfi(2));
                
                QdA{1} = TF{1}.pPos.Qd;
                QdA{2} = TF{2}.pPos.Qd;
                QdA{2}(4) = -TF{2}.pPos.Qd(9);
                QdA{2}(7) = TF{2}.pPos.Qd(8);
                QdA{2}(8) = H_LAND;
                                
                FORMACAO(1) = 0;
                PIONEER(1) = 0;
                %DISABLE MOTORS
%                 P{1}.rDisableMotors;
                
                TF{1}.pPar.R{1,2} = 0;
                TF{1}.pPar.R{2,2} = '-';
                TF{1}.pPar.R{1,3} = 0;
                TF{1}.pPar.R{2,3} = '-';
                
                TF{2}.pPar.R{1,3} = 1;
                TF{2}.pPos.X(7:9) = A{1}.pPos.X(1:3);
                TF{2}.tDirTrans;
                QdAA{2} = TF{2}.pPos.Q;
                Qdtil{2} = QdA{2}-QdAA{2};
                for ii = [4:6 9]
                while abs(Qdtil{2}(ii)) > pi
                    if Qdtil{2}(ii) > 0
                        Qdtil{2}(ii) = -2*pi + Qdtil{2}(ii);
                    else
                        Qdtil{2}(ii) = 2*pi + Qdtil{2}(ii);
                    end
                end
                end
            end
        case 2
            if toc(T) > T_MAX/4
                etapa = etapa + 1;
                T_TEMP = T_MAX/4;
                
                TF{2}.pPos.Qd = QdA{2};
%                 TF{2}.pPos.Qd(2) = abs(Yfi(2));
            
                QdA{2} = TF{2}.pPos.Qd;
                
                DRONES(1) = 0;
                %LAND
                
                Qd{2} = TF{2}.pPos.Qd;
                Qd{2}(4) = 0;
                Qd{2}(7) = RAIO(1);
                Qd{2}(8) = TF{2}.pPos.Qd(7);
                Qdtil{2} = Qd{2}-QdA{2};
                for ii = [4:6 9]
                while abs(Qdtil{2}(ii)) > pi
                    if Qdtil{2}(ii) > 0
                        Qdtil{2}(ii) = -2*pi + Qdtil{2}(ii);
                    else
                        Qdtil{2}(ii) = 2*pi + Qdtil{2}(ii);
                    end
                end
                end
            end
        case 3
            if toc(T) > T_MAX/4 + T_TEMP
                etapa = etapa + 1;
                T_TEMP = T_TEMP + T_MAX/4;
                
                QdA{2} = Qd{2};
                
%                 Qd{2}(6) = -Qd{2}(6);
                Qdtil{2} = Qd{2}-QdA{2};
                for ii = [4:6 9]
                while abs(Qdtil{2}(ii)) > pi
                    if Qdtil{2}(ii) > 0
                        Qdtil{2}(ii) = -2*pi + Qdtil{2}(ii);
                    else
                        Qdtil{2}(ii) = 2*pi + Qdtil{2}(ii);
                    end
                end
                end
            end
        case 4
            if toc(T) > T_MAX/2 + T_TEMP
                etapa = etapa + 1;
                T = tic;
                TF{2}.pPos.Qd = Qd{2};
                TF{2}.pPos.Qd(2) = Yfi(2);
                
                PIONEER(2) = 0;
                %DISABLE MOTORS
%                 P{2}.rDisableMotors;
            end
        case 5
            if toc(T) > T_MAX/2
                etapa = etapa + 1;
                T = tic;
                
                FORMACAO(3) = 1;
                
                TF{2}.pPar.R{1,2} = 1;
                TF{2}.pPar.R{2,2} = 'A';
                TF{2}.pPar.R{1,3} = 4;
                TF{2}.pPar.R{2,3} = 'A';
                
                A{1}.pPos.X(1:3) = TF{2}.pPos.X(4:6);
                A{4}.pPos.X(1:3) = TF{2}.pPos.X(7:9);
                
                TF{3}.pPar.R{1,2} = 2;
                TF{3}.pPar.R{2,2} = 'A';
                TF{3}.pPar.R{1,3} = 3;
                TF{3}.pPar.R{2,3} = 'A';
                
                TF{3}.pPos.Q = [Xfi(3); Yfi(3); Zfi(3);
                                THETAfi; PHIfi; PSIfi;
                                Pfi; Qfi; BETAfi];
                
                TF{3}.pPos.Qd = TF{3}.pPos.Q;
                QdA{3} = TF{3}.pPos.Qd;
                
                TF{3}.tInvTrans;
                
                TF{3}.pPos.X = TF{3}.pPos.Xd;
                
                P{3}.pPos.X(1:3) = TF{3}.pPos.X(1:3);
                TF{3}.pPos.X(4:6) = A{2}.pPos.X(1:3);
                A{3}.pPos.X(1:3) = TF{3}.pPos.X(7:9);
                
                TF{3}.tDirTrans;
                QdAA{3} = TF{3}.pPos.Q;
                Qdtil{3} = QdA{3}-QdAA{3};
                for ii = [4:6 9]
                while abs(Qdtil{3}(ii)) > pi
                    if Qdtil{3}(ii) > 0
                        Qdtil{3}(ii) = -2*pi + Qdtil{3}(ii);
                    else
                        Qdtil{3}(ii) = 2*pi + Qdtil{3}(ii);
                    end
                end
                end
            end
        case 6
            if toc(T) > T_MAX/2
                etapa = etapa + 1;
                T = tic;
                QdA{2} = TF{2}.pPos.Qd;
                
                DRONES(1) = 1;
                %TAKEOFF
            end
        case 7
            if toc(T) > T_MAX/4
                etapa = etapa + 1;
                T = tic;
                
                PIONEER(2) = 1;
                PIONEER(3) = 1;
                %ENABLE MOTORS
%                 P{2}.rEnableMotors;
%                 P{3}.rEnableMotors;
                
                QdA{2} = TF{2}.pPos.Qd;
                QdA{3} = TF{3}.pPos.Qd;
            end
        case 8
            if toc(T) > T_MAX
                break
            end
    end
            
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         FORMAÇÕES DESEJADAS                             %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    

    
    switch etapa
        case 1
            % FORMAÇÃO 1
            TF{1}.pPos.Qd = QdA{1};
            TF{1}.pPos.Qd(2) = QdA{1}(2)+(abs(Yfi(1))-QdA{1}(2))*toc(T)/T_MAX;
            % FORMAÇÃO 2
            TF{2}.pPos.Qd = QdA{2};
            TF{2}.pPos.Qd(2) = QdA{2}(2)+(abs(Yfi(2))-QdA{2}(2))*toc(T)/T_MAX;
        case 2
            % FORMAÇÃO 2
            TF{2}.pPos.Qd = QdAA{2}+Qdtil{2}*toc(T)/(T_MAX/4);
            TF{2}.pPos.Qd(2) = QdA{2}(2)+(Yfi(2)-QdA{2}(2))*toc(T)/T_MAX;
        case 3
            % FORMAÇÃO 2
            TF{2}.pPos.Qd = QdA{2}+Qdtil{2}*(toc(T)-T_TEMP)/(T_MAX/4);
            TF{2}.pPos.Qd(2) = QdA{2}(2)+(Yfi(2)-QdA{2}(2))*toc(T)/T_MAX;
        case 4
            % FORMAÇÃO 2
            TF{2}.pPos.Qd = QdA{2}+Qdtil{2}*(toc(T)-T_TEMP)/(T_MAX/2);
            TF{2}.pPos.Qd(2) = QdA{2}(2)+(Yfi(2)-QdA{2}(2))*toc(T)/T_MAX;
        case 6
            TF{2}.pPos.Qd(7) = H_LAND; 
            % FORMAÇÃO 3
            TF{3}.pPos.Qd = QdAA{3}+Qdtil{3}*toc(T)/(T_MAX/2);
        case 7
            % FORMAÇÃO 2
            TF{2}.pPos.Qd(7) = QdA{2}(7)+(RAIO(1)-QdA{2}(7))*toc(T)/(T_MAX/4);
            % FORMAÇÃO 3
            TF{3}.pPos.Qd = QdA{3};
        case 8
            % FORMAÇÃO 2
            TF{2}.pPos.Qd = QdA{2};
            TF{2}.pPos.Qd(2) = QdA{2}(2)+(abs(Yfi(2))-QdA{2}(2))*toc(T)/T_MAX;
            % FORMAÇÃO 3
            TF{3}.pPos.Qd = QdA{3};
            TF{3}.pPos.Qd(2) = QdA{3}(2)+(abs(Yfi(3))-QdA{3}(2))*toc(T)/T_MAX;
    end
        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        POSIÇÃO REAL DOS ROBÔS                           %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    % Coletando os dados dos corpos rígidos do OptiTrack
%     rb = OPT.RigidBody;
    
    % Pioneer
%     for ii = find(PIONEER==1)
%         try
%             if rb(idP{ii}).isTracked
%                 P{ii} = getOptData(rb(idP{ii}),P{ii});
% %                 P{ii}.pPos.X
% %                 P{ii}.pPos.X(3) = 0;
%             end
%         catch
%         end
%     end
%     
%     % Drones
%     for ii = find(DRONES==1)
%         try
%             if rb(idA{ii}).isTracked
%                 A{ii} = getOptData(rb(idA{ii}),A{ii});
% %                A{ii}.pPos.X
%             end
%         catch
%         end
%     end
    
    for ii = find(FORMACAO==1)
        for jj = 1:3
            if TF{ii}.pPar.R{2,jj} == 'P'
                ID = TF{ii}.pPar.R{1,jj};
                TF{ii}.pPos.X((1:3)+3*(jj-1)) = P{ID}.pPos.X(1:3);
            elseif TF{ii}.pPar.R{2,jj} == 'A'
                ID = TF{ii}.pPar.R{1,jj};
                TF{ii}.pPos.X((1:3)+3*(jj-1)) = A{ID}.pPos.X(1:3);
            end
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          FORMAÇÃO DESEJADA                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    for ii = find(FORMACAO==1)
%         TF{ii}.pPos.Qd = Qd{ii}{MISSAO{ii}(ETAPA{ii},1)}(:,MISSAO{ii}(ETAPA{ii},2));
        TF{ii}.pPos.dQd = (TF{ii}.pPos.Qd - TF{ii}.pPos.QdA)/toc(TF{ii}.pPar.ti);
        
        TF{ii}.pPar.ti = tic;
        TF{ii}.pPos.QdA = TF{ii}.pPos.Qd;
    end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                            CONTROLADORES                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    % Controle da formação
    for ii = find(FORMACAO==1)
        TF{ii}.tFormationControl;
        TF{ii}.tInvTrans;
    end
        
    % Entregando a referencia para cada robô
    for ii = find(FORMACAO==1)
        for jj = 1:3
            if TF{ii}.pPar.R{2,jj} == 'P'
                ID = TF{ii}.pPar.R{1,jj};
                P{ID}.pPos.Xd(1:3) = TF{ii}.pPos.Xd((1:3)+3*(jj-1));
                P{ID}.pPos.Xd(7:9) = TF{ii}.pPos.dXr((1:3)+3*(jj-1));
            elseif TF{ii}.pPar.R{2,jj} == 'A'
                ID = TF{ii}.pPar.R{1,jj};
                A{ID}.pPos.Xr(7:9) = TF{ii}.pPos.dXr((1:3)+3*(jj-1));
            end
        end
    end
    
%     % Controlador dos robôs
%     for ii = find(PIONEER==1)
%         P{ii} = fKinematicControllerExtended(P{ii},pgains);
%     end
%     for ii = find(DRONES==1)
%         A{ii} = cInverseDynamicController_Compensador_ArDrone(A{ii});
%     end
%     
%     % Joystick control
%     for ii = find(DRONES==1)
%         A{ii} = J.mControl(A{ii});
%     end
%     for ii = find(PIONEER==1)
%         P{ii} = J.mControl(P{ii});
%     end
%     
%     % Drone virtual
%     for ii = find(FORMACAO==1)
%         for jj = 1:3
%             if TF{ii}.pPar.R{2,jj} == 'A'
%                 ID = TF{ii}.pPar.R{1,jj};
%                 if DRONES(ID) == 0
%                     A{ID}.pPos.X(1:3) = TF{ii}.pPos.Xd((1:3)+3*(jj-1));
%                 end
%             end
%         end
%     end
    
    % Simulação
    for ii = find(FORMACAO==1)
        for jj = 1:3
            if TF{ii}.pPar.R{2,jj} == 'P'
                ID = TF{ii}.pPar.R{1,jj};
                P{ID}.pPos.X(1:3) = TF{ii}.pPos.Xd((1:3)+3*(jj-1));
            elseif TF{ii}.pPar.R{2,jj} == 'A'
                ID = TF{ii}.pPar.R{1,jj};
                A{ID}.pPos.X(1:3) = TF{ii}.pPos.Xd((1:3)+3*(jj-1));
            end
        end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         ARMAZENANDO OS DADOS                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    for ii = find(FORMACAO==1)
        DADOS_TEMP = [];
        for jj = 1:3
            if TF{ii}.pPar.R{2,jj} == 'P'
                ID = TF{ii}.pPar.R{1,jj};
                DADOS_TEMP(1,end+1:end+26) = [P{ID}.pPos.Xd'...
                                              P{ID}.pPos.X'...
                                              P{ID}.pSC.Ud'];
            elseif TF{ii}.pPar.R{2,jj} == 'A'
                ID = TF{ii}.pPar.R{1,jj};
                DADOS_TEMP(1,end+1:end+40) = [A{ID}.pPos.Xd'...
                                              A{ID}.pPos.X'...
                                              A{ID}.pPos.Xr'...
                                              A{ID}.pSC.Ud'];
            end
        end
        DADOS{ii}(end+1,:) = [DADOS_TEMP...
                             TF{ii}.pPos.Qd' TF{ii}.pPos.Q' TF{ii}.pPos.dQd' TF{ii}.pPos.dQr'...
                             TF{ii}.pPos.Xd' TF{ii}.pPos.X' TF{ii}.pPos.dXr'...
                             toc(T) toc(T_ALFA)];
    end

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

%     %IAE
%     IAEt=IAEt+norm(TF.pPos.Qtil)*toc(Td);
% 
%     %ITAE
%     ITAE=ITAE+norm(TF.pPos.Qtil)*toc(t)*toc(Td);
%     
%     %IASC
%     IASC=IASC+norm(TF.pPos.dXr)*toc(Td);
%     
%     Td = tic;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         SINAIS DE CONTROLE                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
%     for ii = find(PIONEER==1)
%         P{ii}.rCommand;
%     end
%     for ii = find(DRONES==1)
%         A{ii}.rSendControlSignals;
%     end
        
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
    
    X = TF{1}.pPos.X;
    X2 = TF{2}.pPos.X;
    X3 = TF{3}.pPos.X;
    if FORMACAO(1) == 1
    H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
    H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
    H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'k','LineWidth',2);
    Point(1) = plot3(X(1),X(2),X(3),'or','MarkerSize',10,'LineWidth',2);
    TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,['Pioneer' num2str(P{TF{1}.pPar.R{1,1}}.pID)],'FontWeight','bold');
    if DRONES(TF{1}.pPar.R{1,2}) == 1
        Point(2) = plot3(X(4),X(5),X(6),'^r','MarkerSize',10,'LineWidth',2);
        TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,['ArDrone' num2str(A{TF{1}.pPar.R{1,2}}.pID)],'FontWeight','bold');
        Rastro(2) = plot3([XA(4) X(4)],[XA(5) X(5)],[XA(6) X(6)],'-r');
    end
    if DRONES(TF{1}.pPar.R{1,3}) == 1
        Point(3) = plot3(X(7),X(8),X(9),'^r','MarkerSize',10,'LineWidth',2);
        TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,['ArDrone' num2str(A{TF{1}.pPar.R{1,3}}.pID)],'FontWeight','bold');
        Rastro(3) = plot3([XA(7) X(7)],[XA(8) X(8)],[XA(9) X(9)],'-r');
    end
    Rastro(1) = plot3([XA(1) X(1)],[XA(2) X(2)],[XA(3) X(3)],'-r');
    end
    if FORMACAO(2) == 1
    H(4) = plot3([X2(1) X2(4)],[X2(2) X2(5)],[X2(3) X2(6)],'b','LineWidth',2);
    H(5) = plot3([X2(1) X2(7)],[X2(2) X2(8)],[X2(3) X2(9)],'b','LineWidth',2);
    H(6) = plot3([X2(7) X2(4)],[X2(8) X2(5)],[X2(9) X2(6)],'k','LineWidth',2);
    Point(4) = plot3(X2(1),X2(2),X2(3),'og','MarkerSize',10,'LineWidth',2);
    TPlot(4) = text(X2(1)+.1,X2(2)+.1,X2(3)+.1,['Pioneer' num2str(P{TF{2}.pPar.R{1,1}}.pID)],'FontWeight','bold');
    if DRONES(TF{2}.pPar.R{1,2}) == 1
        Point(5) = plot3(X2(4),X2(5),X2(6),'^g','MarkerSize',10,'LineWidth',2);
        TPlot(5) = text(X2(4)+.1,X2(5)+.1,X2(6)+.1,['ArDrone' num2str(A{TF{2}.pPar.R{1,2}}.pID)],'FontWeight','bold');
        Rastro(5) = plot3([XA2(4) X2(4)],[XA2(5) X2(5)],[XA2(6) X2(6)],'-g');
    end
    if DRONES(TF{2}.pPar.R{1,3}) == 1
        Point(6) = plot3(X2(7),X2(8),X2(9),'^g','MarkerSize',10,'LineWidth',2);
        TPlot(6) = text(X2(7)+.1,X2(8)+.1,X2(9)+.1,['ArDrone' num2str(A{TF{2}.pPar.R{1,3}}.pID)],'FontWeight','bold');
        Rastro(6) = plot3([XA2(7) X2(7)],[XA2(8) X2(8)],[XA2(9) X2(9)],'-g');
    end
    Rastro(4) = plot3([XA2(1) X2(1)],[XA2(2) X2(2)],[XA2(3) X2(3)],'-g');
    end
    if FORMACAO(3) == 1
    H(7) = plot3([X3(1) X3(4)],[X3(2) X3(5)],[X3(3) X3(6)],'b','LineWidth',2);
    H(8) = plot3([X3(1) X3(7)],[X3(2) X3(8)],[X3(3) X3(9)],'b','LineWidth',2);
    H(9) = plot3([X3(7) X3(4)],[X3(8) X3(5)],[X3(9) X3(6)],'k','LineWidth',2);
    Point(7) = plot3(X3(1),X3(2),X3(3),'ob','MarkerSize',10,'LineWidth',2);
    TPlot(7) = text(X3(1)+.1,X3(2)+.1,X3(3)+.1,['Pioneer' num2str(P{TF{3}.pPar.R{1,1}}.pID)],'FontWeight','bold');
    if DRONES(TF{3}.pPar.R{1,2}) == 1
        Point(8) = plot3(X3(4),X3(5),X3(6),'^b','MarkerSize',10,'LineWidth',2);
        TPlot(8) = text(X3(4)+.1,X3(5)+.1,X3(6)+.1,['ArDrone' num2str(A{TF{3}.pPar.R{1,2}}.pID)],'FontWeight','bold');
        Rastro(8) = plot3([XA3(4) X3(4)],[XA3(5) X3(5)],[XA3(6) X3(6)],'-b');
    end
    if DRONES(TF{3}.pPar.R{1,3}) == 1
        Point(9) = plot3(X3(7),X3(8),X3(9),'^b','MarkerSize',10,'LineWidth',2);
        TPlot(9) = text(X3(7)+.1,X3(8)+.1,X3(9)+.1,['ArDrone' num2str(A{TF{3}.pPar.R{1,3}}.pID)],'FontWeight','bold');
        Rastro(9) = plot3([XA3(7) X3(7)],[XA3(8) X3(8)],[XA3(9) X3(9)],'-b');
    end
    Rastro(7) = plot3([XA3(1) X3(1)],[XA3(2) X3(2)],[XA3(3) X3(3)],'-b');
    end
    
    XA = X;
    XA2 = X2;
    XA3 = X3;
%     plot3(A{2}.pPos.X(1),A{2}.pPos.X(2),A{2}.pPos.X(3),'.k');
%     plot3(A{2}.pPos.Xd(1),A{2}.pPos.Xd(2),A{2}.pPos.Xd(3),'.r');
    drawnow
    
end
end
% catch ME
%     disp(ME)
% end

% for ii = 1:3
%     P{ii}.rDisableMotors;
% end

% for ii = 1:3
%     for jj = 1:size(DRONES,2)
%         disp(jj)
%         A{jj}.rLand;
%     end
% end






