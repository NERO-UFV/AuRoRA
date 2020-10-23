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
%     
% %     % Inicializando o OptiTrack
%     OPT = OptiTrack;    % Criando o OptiTrack
%     OPT.Initialize;     % Iniciando o OptiTrack

    % Iniciando os Robôs
    A{1} = ArDrone(1); % Drone em cima do Pioneer3DX
    A{2} = ArDrone(2); % Drone voando
    A{3} = ArDrone(3); % Drone virtual
    DRONES = [1 1 0];    % Drones ativos
%     
%     P{1} = RPioneer(1,'P1'); % Pìoneer3DX Experimento
    P{1} = Pioneer3DX(1); % Pioneer3DX Simulado
%     P{ii}.rDisableMotors;

    PIONEER = [0]; % Pioneer ativos
    
    % Pegando o ID dos corpos rigidos no OptiTrack
%     idA{1} = getID(OPT,ArDrone,1);
%     idP{1} = 1;
    
    % Iniciando a formação triangular
    TF{1} = TriangularFormationBaju(1);
    TF{1}.pPar.R = {1 1 2; 'P' 'A' 'A'};
    
    FORMACAO = [1];
    
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
A{1}.pSC.Kinematics_control = 0;
A{2}.pSC.Kinematics_control = 0;

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
Xfi(1) = 0;
Yfi(1) = 0;
Zfi(1) = 0;

% Orientação das Formações
THETAfi = -pi/5.5;
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
%                            POSIÇÃO INICIAL                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A{1}.pPos.X(1:3) = [P{1}.pPos.X(1:2);  RAIO(1)];
A{2}.pPos.X(1:3) = [.8; 1.2;  RAIO(1)];

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

TF{1}.tDirTrans;

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

XA = X;

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

axis equal
axis([-3 3 -3 3 0 3])
view(50,30)
end
pause

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                        VARIÁVEIS PARA SIMULAÇÃO                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Marcadores de tempo
T_MAX = 15;                 % Tempo maximo de duração de cada tarefa
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
%                            ETAPA DESEJADA                               %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   

    switch etapa
        case 1
            if toc(T) > T_MAX/2
                etapa = etapa + 1;
                T = tic;
                
                PSI = atan2((A{2}.pPos.X(2)-A{1}.pPos.X(2)),(A{2}.pPos.X(1)-A{1}.pPos.X(1)));
                
                
                QdA{1}(6) = PSI;
                QdA{1}(7) = QdA{1}(8);
                QdA{1}(8) = H_LAND;
                
                Qd{1} = QdA{1};
                QdA{1} = TF{1}.pPos.Qd;
                Qdtil{1} = Qd{1}-QdA{1};
                for ii = [4:6 9]
                while abs(Qdtil{1}(ii)) > pi
                    if Qdtil{1}(ii) > 0
                        Qdtil{1}(ii) = -2*pi + Qdtil{1}(ii);
                    else
                        Qdtil{1}(ii) = 2*pi + Qdtil{1}(ii);
                    end
                end
                end
            end    
        case 2
            if toc(T) > T_MAX
                etapa = etapa + 1;
                T = tic;
                
                DRONES(2) = 0;
                %LAND
                
                TF{1}.pPar.R{1,3} = 3;
                TF{1}.pPar.R{2,3} = 'A';
                
                A{3}.pPos.X(1:3) = [P{1}.pPos.X(1:2); RAIO(1)];
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
                
                TF{1}.tDirTrans;
                QdA{1}(8) = Qfi;
                QdA{1}(9) = BETAfi;
                
                Qd{1} = QdA{1};
                QdA{1} = TF{1}.pPos.Q;
                Qdtil{1} = Qd{1}-QdA{1};
                for ii = [4:6 9]
                while abs(Qdtil{1}(ii)) > pi
                    if Qdtil{1}(ii) > 0
                        Qdtil{1}(ii) = -2*pi + Qdtil{1}(ii);
                    else
                        Qdtil{1}(ii) = 2*pi + Qdtil{1}(ii);
                    end
                end
                end
            end
        case 3
            if toc(T) > T_MAX
                etapa = etapa + 1;
                T = tic;
                
                
                Qdtil{1} = QdA{1}-Qd{1};
                for ii = [4:6 9]
                while abs(Qdtil{1}(ii)) > pi
                    if Qdtil{1}(ii) > 0
                        Qdtil{1}(ii) = -2*pi + Qdtil{1}(ii);
                    else
                        Qdtil{1}(ii) = 2*pi + Qdtil{1}(ii);
                    end
                end
                end
            end    
        case 4
            if toc(T) > T_MAX/2
                etapa = etapa + 1;
                T = tic;
            end         
        case 5
            if toc(T) > T_MAX
                etapa = etapa + 1;
                T = tic;
                QdA{1} = TF{1}.pPos.Qd;
                
                DRONES(2) = 1;
                %TAKE OFF
                
                TF{1}.pPar.R{1,3} = 2;
                TF{1}.pPar.R{2,3} = 'A';
                
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
                
                TF{1}.tDirTrans;
                QdAA{1} = TF{1}.pPos.Q;
                Qdtil{1} = QdA{1}-QdAA{1};
                for ii = [4:6 9]
                while abs(Qdtil{1}(ii)) > pi
                    if Qdtil{1}(ii) > 0
                        Qdtil{1}(ii) = -2*pi + Qdtil{1}(ii);
                    else
                        Qdtil{1}(ii) = 2*pi + Qdtil{1}(ii);
                    end
                end
                end
            end   
        case 6
            if toc(T) > T_MAX/2
                etapa = etapa + 1;
                T = tic;
                
                QdA{1} = TF{1}.pPos.Qd;
                
                QdA{1}(4) = 0;
                QdA{1}(8) = QdA{1}(7);
                QdA{1}(7) = H_LAND;
                
                Qd{1} = QdA{1};
                QdA{1} = TF{1}.pPos.Qd;
                Qdtil{1} = Qd{1}-QdA{1};
                for ii = [4:6 9]
                while abs(Qdtil{1}(ii)) > pi
                    if Qdtil{1}(ii) > 0
                        Qdtil{1}(ii) = -2*pi + Qdtil{1}(ii);
                    else
                        Qdtil{1}(ii) = 2*pi + Qdtil{1}(ii);
                    end
                end
                end
            end
        case 7
            if toc(T) > T_MAX
                etapa = etapa + 1;
                T = tic;
                
                DRONES(1) = 0;
                %LAND
                
                TF{1}.pPar.R{1,2} = 3;
                TF{1}.pPar.R{2,2} = 'A';
                
                A{3}.pPos.X(1:3) = [P{1}.pPos.X(1:2); RAIO(1)];
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
                
                TF{1}.tDirTrans;
                QdA{1}(8) = Pfi;
                QdA{1}(9) = BETAfi;
                
                Qd{1} = QdA{1};
                QdA{1} = TF{1}.pPos.Q;
                Qdtil{1} = Qd{1}-QdA{1};
                for ii = [4:6 9]
                while abs(Qdtil{1}(ii)) > pi
                    if Qdtil{1}(ii) > 0
                        Qdtil{1}(ii) = -2*pi + Qdtil{1}(ii);
                    else
                        Qdtil{1}(ii) = 2*pi + Qdtil{1}(ii);
                    end
                end
                end
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
            TF{1}.pPos.Qd = TF{1}.pPos.Q;
        case 2
            TF{1}.pPos.Qd = QdA{1} + Qdtil{1}*toc(T)/T_MAX;
        case 3
            TF{1}.pPos.Qd = QdA{1} + Qdtil{1}*toc(T)/T_MAX;
        case 5
            TF{1}.pPos.Qd = Qd{1} + Qdtil{1}*toc(T)/T_MAX;
        case 6
            TF{1}.pPos.Qd = QdAA{1} + Qdtil{1}*toc(T)/(T_MAX/2);
        case 7
            TF{1}.pPos.Qd = QdA{1} + Qdtil{1}*toc(T)/T_MAX;
        case 8
            TF{1}.pPos.Qd = QdA{1} + Qdtil{1}*toc(T)/T_MAX;
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
    
    XA = X;
    drawnow
    
end
end

% for ii = 1:3
%     P{ii}.rDisableMotors;
% end

% for ii = 1:3
%     for jj = 1:size(DRONES,2)
%         disp(jj)
%         A{jj}.rLand;
%     end
% end






