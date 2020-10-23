close all
clear all
clc

try
    rosshutdown
catch
end

try
    fclose(instrfindall);
catch
end

%%                          INICIO DO PROGRAMA                           %%
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                         CARREGANDO AS CLASSES                           %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

% Inicializando o OptiTrack
OPT = OptiTrack;    % Criando o OptiTrack
OPT.Initialize;     % Iniciando o OptiTrack

% Inicializando os robôs
A{1} = ArDrone(1);
A{2} = ArDrone;
P = Pioneer3DX;

A{2}.pPar.ti = tic;

% Pegando o ID dos corpos rigidos no OptiTrack
% idA{1} = getID(OPT,ArDrone,1);
idA{2} = getID(OPT,ArDrone,2);
% idP = getID(OPT,P);

% Joystick
J = JoyControl;

% Inicializando o ROS
if false
setenv('ROS_IP','192.168.0.158')                        % Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  % Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.148:11311/')  % Ip do master (raspberry)
rosinit                                                 % Inicia a comunicação
% Virando publicador no ROS
[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist');
% Publicando o primeiro sinal de controle 
cmd_vel.Linear.X = 0;
cmd_vel.Angular.Z = 0;
send(pub,cmd_vel);
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                          VARIÁVEIS INICIAIS                             %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

% Ganhos
cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];   % Pioneer
L = diag([ 1   1   1   1   1   1     1   1   1 ]);              % Formação
%          x   y   z   phi psi theta p   q   beta
%          X     Y     Z    Psi
% gains = [  2     2     3     1 ...
%            2     2     2    .5 ...
%            0.9   0.9   0.9  0.9 ...
%            1     1     1     1];

% Valores iniciais da Formação
phi = 0;                    % phi (Ângulo da Formação em relação ao eixo z)
psi = 0;                    % psi (Ângulo do eixo y)
theta = 0;                  % theta (Ângulo do plano da Formação)
beta = pi/4;                % beta (Ângulo interno da Formação)

% Valores iniciais dos robôs
% Coletando os dados dos corpos rígidos do OptiTrack
rb = OPT.RigidBody;
P.pPos.X(1:3) = [-1 0 0];

% Drone 1
try
if rb(idA{1}).isTracked
    A{1} = getOptData(rb(idA{1}),A{1});
    A{1}.pPos.X
end
end
% Drone 2
try
if rb(idA{2}).isTracked
    A{2} = getOptData(rb(idA{2}),A{2});
    A{2}.pPos.X
end
end
% Pioneer
try
if rb(idP).isTracked
    P = getOptData(rb(idP),P);
    P.pPos.X
end
end

% Valores iniciais do Drone fora do Pioneer
X_Drone2 = 0;   % Posição em x
Y_Drone2 = -1;  % Posição em y

% Angulo entre o Drone e o Pioneer
% psi = atan2(Y_Drone2-P.pPos.X(2),X_Drone2-P.pPos.X(1)) + pi; 

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                         DEFININDO AS PROTEÇÕES                          %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

% Distancia de segurança entre o drone e pioneer (raio da esfera)
RAIO(1) = 1.5;

% Distancia de segurança para o takeoff (raio do cilindro)
RAIO(2) = RAIO(1)*cos(beta);

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                       PLOT INICIAL DO EXPERIMENTO                       %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

RegiaoProtecaoBaju

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                          CONECTANDO NOS DRONES                          %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

if false
% Drone 1 (Em cima do Pioneer)
A{1}.pPar.ip = '192.168.1.30';
A{1}.rConnect;
end
if true
% Drone 2 (Fora do Pioneer)
% A{2}.pPar.LocalPortControl = 5558;
% A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.1';
A{2}.rConnect;

% ArDrone Takeoff
disp('Start Take Off Timming....');
% A{1}.rTakeOff;
A{2}.rTakeOff;
% pause(2);
disp('Taking Off End Time....');
end

% Iniciando os temporizadores dos robôs
% A{1}.pPar.ti = tic;
% A{2}.pPar.ti = tic;
% P.pPar.ti = tic;


%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                        VARIAVEIS DO EXPERIMENTO                         %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

% Variaveis temporais
T_MAX = 30;                 % Tempo total do experimento
T_AMOSTRAGEM = 1/30;        % Tempo de amostragem
T_PLOT = 0.5;               % Tempo para o ploit
T_ETAPA = [10 20 10 5 20];   % Tempo das etapas
T_ETAPINHAS = [5];          % Tempo das etapinhas
A{1}.pPar.Ts = 1/30;        % Tempo de amostragem do Drone 1
A{2}.pPar.Ts = 1/30;        % Tempo de amostragem do Drone 2

% Variáveis iniciais
ETAPA = 1;  % Definindo a etapa
DADOS = []; % Iniciando a matriz dos dados
CilindroFlag = 0;
EsferaFlag = 0;

% Iniciando os temporizadores
T_ALFA = tic;           % Temporizador TOTAL do experimento
T = tic;                % Temporizador do experimento
TA = tic;               % Temporizador da amostragem
TP = tic;               % Temporizador do plot
% A{1}.pPar.ti = tic;     % Temporizador do Drone 1
% A{2}.pPar.ti = tic;     % Temporizador do Drone 2

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                               EXPERIMENTO                               %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

while toc(T) < T_MAX
if toc(TA) > T_AMOSTRAGEM
TA = tic;       % Reiniciando o temporizador da amostragem
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                         INICIALIZANDO AS ETAPAS                         %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
switch ETAPA
    case 1
        if toc(T) > T_ETAPA(ETAPA)
            T = tic;
            ETAPA = ETAPA + 1;
            W = 2*pi/T_ETAPA(ETAPA);
%           
%             % Definindo o ponto do caminho mais proximo do robo
%             [CAMINHO{1}.MIN,CAMINHO{1}.K] = min(sqrt((A{2}.pPos.X(1) - CAMINHO{1}.X(1,:)).^2 +...
%                 (A{2}.pPos.X(2) - CAMINHO{1}.X(2,:)).^2 +...
%                 (A{2}.pPos.X(3) - CAMINHO{1}.X(3,:)).^2));
%             
%             if CAMINHO{1}.K == 1
%                 CAMINHO{1}.K = CAMINHO{1}.K + 1;
%             end
%             
%             CAMINHO{1}.BETA = atan2((CAMINHO{1}.X(2,CAMINHO{1}.K) - CAMINHO{1}.X(2,CAMINHO{1}.K-1)),...
%                 (CAMINHO{1}.X(1,CAMINHO{1}.K) - CAMINHO{1}.X(1,CAMINHO{1}.K-1)));
%             CAMINHO{1}.dBETA = 0;
%             CAMINHO{1}.ADV = tic;
%             CAMINHO{1}.ON = 1;
%             A{2}.pSC.Kinematics_control = 1;
%             %             break
        end
    case 2
        if toc(T) > T_ETAPA(ETAPA) || CilindroFlag == 1
%             break
            T = tic;
            ETAPA = ETAPA + 1;
        end
    case 3
        if toc(T) > T_ETAPA(ETAPA)
            break
            T = tic;
            ETAPA = ETAPA + 1;
        end
end
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           OBJETIVOS DAS ETAPAS                          %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
switch ETAPA
    case 1
        % Definindo a posição desejada do Drone 2
        A{2}.pPos.Xd([1:3 6]) = [CAMINHO{1}.X(1,1);
                                 CAMINHO{1}.X(2,1);
                                 CAMINHO{1}.X(3,1);
                                 0]; %atan2(CAMINHO{1}.X(2,3) - CAMINHO{1}.X(2,2),CAMINHO{1}.X(1,3) - CAMINHO{1}.X(1,2))];
        A{2}.pPos.Xd([7:9 12]) = [0;
                                  0;
                                  0;
                                  0];
    case 2
%         CAMINHO{1}.MIN = min(sqrt((A{2}.pPos.X(1) - CAMINHO{1}.X(1,CAMINHO{1}.K)).^2 +...
%                 (A{2}.pPos.X(2) - CAMINHO{1}.X(2,CAMINHO{1}.K)).^2 +...
%                 (A{2}.pPos.X(3) - CAMINHO{1}.X(3,CAMINHO{1}.K)).^2));
%         
%         if (CAMINHO{1}.MIN < 0.05 || CAMINHO{1}.ON == 1 && toc(CAMINHO{1}.ADV) > T_ETAPA(ETAPA)/CAMINHO{1}.PONTOS) && CAMINHO{1}.K < CAMINHO{1}.PONTOS
% %         if CAMINHO{1}.MIN < 0.1 && CAMINHO{1}.K < CAMINHO{1}.PONTOS
%             CAMINHO{1}.ADV = tic;
%             CAMINHO{1}.ON = 1;
%             CAMINHO{1}.K = CAMINHO{1}.K + 1;
%             CAMINHO{1}.K
%             disp(CAMINHO{1}.K)
%             
%             BETA = CAMINHO{1}.BETA;
%             CAMINHO{1}.BETA = atan2((CAMINHO{1}.X(2,CAMINHO{1}.K) - CAMINHO{1}.X(2,CAMINHO{1}.K-1)),...
%                 (CAMINHO{1}.X(1,CAMINHO{1}.K) - CAMINHO{1}.X(1,CAMINHO{1}.K-1)));
%             CAMINHO{1}.dBETA = CAMINHO{1}.BETA - BETA;
%             
%             A{2}.pPos.Xd([7:9 12]) = [CAMINHO{1}.VMAX/(1+200*abs(CAMINHO{1}.dBETA))*cos(CAMINHO{1}.BETA);
%                                    CAMINHO{1}.VMAX/(1+200*abs(CAMINHO{1}.dBETA))*sin(CAMINHO{1}.BETA);
%                                    0;
%                                    0];
%             A{2}.pPos.Xr([7:9 12]) = A{2}.pPos.Xd([7:9 12]);
%         end
%         
%         if CAMINHO{1}.K == CAMINHO{1}.PONTOS
%             A{2}.pPos.Xd([7:9 12]) = [0; 0; 0; 0];
%         end
%         % Definindo o caminho desejado do Drone 2
%         A{2}.pPos.Xd([1:3 6]) = [CAMINHO{1}.X(1,CAMINHO{1}.K);
%                               CAMINHO{1}.X(2,CAMINHO{1}.K);
%                               CAMINHO{1}.X(3,CAMINHO{1}.K);
%                               CAMINHO{1}.BETA];
%         TRAJETORIA
%           PSIa = A{2}.pPos.Xd(6);
%           A{2}.pPos.Xd([1:3 6]) = [cos(W*toc(T));
%                                    sin(W*toc(T));
%                                    1.5;
%                                    atan2(W*cos(W*toc(T)),-W*sin(W*toc(T)))];
%           A{2}.pPos.Xd([7:9 12]) = [-W*sin(W*toc(T));
%                                    W*cos(W*toc(T));
%                                    0;
%                                    (A{2}.pPos.Xd(6) - PSIa)/toc(A{2}.pPar.ti)];
%         LINHA
          if CilindroFlag == 0
              Xa = A{2}.pPos.X(1:3);
          end
          if norm(P.pPos.X(1:2)-A{2}.pPos.X(1:2)) > 1 && CilindroFlag == 0
              A{2}.pPos.Xd([1:3 6]) = [CAMINHO{1}.X(1,1) + (P.pPos.X(1) - CAMINHO{1}.X(1,1))*toc(T)/T_ETAPA(ETAPA);
                                       CAMINHO{1}.X(2,1) + (P.pPos.X(2) - CAMINHO{1}.X(2,1))*toc(T)/T_ETAPA(ETAPA);
                                       CAMINHO{1}.X(3,1) + (0.5 - CAMINHO{1}.X(3,1))*toc(T)/T_ETAPA(ETAPA);
                                       0];
              A{2}.pPos.Xd([7:9 12]) = [(P.pPos.X(1) - CAMINHO{1}.X(1,1))/T_ETAPA(ETAPA);
                                        (P.pPos.X(2) - CAMINHO{1}.X(2,1))/T_ETAPA(ETAPA);
                                        (0.5 - CAMINHO{1}.X(3,1))/T_ETAPA(ETAPA);
                                        0];
          else
              CilindroFlag = 1;
              A{2}.pPos.Xd([1:3 6]) = [Xa(1);
                                       Xa(2);
                                       Xa(3);
                                       0];
              A{2}.pPos.Xd([7:9 12]) = [0;
                                        0;
                                        0;
                                        0];
          end
    case 3
        if EsferaFlag == 0
            Xa = A{2}.pPos.X(1:3);
        end
        if norm(P.pPos.X(1:3)-A{2}.pPos.X(1:3)) > 1 && EsferaFlag == 0
            A{2}.pPos.Xd([1:3 6]) = [Xa(1) + (Xa(1)*.01 - Xa(1))*toc(T)/T_ETAPA(ETAPA);
                                     Xa(2) + (Xa(2)*.01 - Xa(2))*toc(T)/T_ETAPA(ETAPA);
                                     Xa(3) - Xa(3)*toc(T)/T_ETAPA(ETAPA);
                                     0];
            A{2}.pPos.Xd([7:9 12]) = [(Xa(1)*.01 - Xa(1))/T_ETAPA(ETAPA);
                                      (Xa(2)*.01 - Xa(2))/T_ETAPA(ETAPA);
                                      -Xa(3)/T_ETAPA(ETAPA);
                                      0];
        else
            EsferaFlag = 1;
            A{2}.pPos.Xd([1:3 6]) = [Xa(1);
                                     Xa(2);
                                     Xa(3);
                                     0];
            A{2}.pPos.Xd([7:9 12]) = [0;
                                      0;
                                      0;
                                      0];
        end
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                      LEITURA DE POSIÇÃO E VELOCIDADE                    %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

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
end
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                                CONTROLE                                 %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
% Calculando sinais de controle usando controlador
switch ETAPA
    case 1
%         A{1} = cInverseDynamicController_Compensador_ArDrone(A{1}); % Drone 1
        A{2} = cInverseDynamicController_Compensador_ArDrone(A{2}); % Drone 2
%         P = fDynamicController(P,cgains);                           % Pioneer
    case 2
%         A{1} = cInverseDynamicController_Compensador_ArDrone(A{1}); % Drone 1
        A{2} = cInverseDynamicController_Compensador_ArDrone(A{2}); % Drone 2
%         P = fDynamicController(P,cgains);                           % Pioneer
end

% Controlando os drones pelo Joystick
% A{1} = J.mControl(A{1});
A{2} = J.mControl(A{2});

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                                  DADOS                                  %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

DADOS(end+1,:) = [A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pSC.Ud' A{1}.pSC.U'...
                  A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pSC.Ud' A{2}.pSC.U'...
                  P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(T_ALFA)];
%                   Qd' Q' Xd' X' A{1}.pPar.Xr' A{2}.pPar.Xr' toc(T_Alfa)];
            
%           1 -- 12         13 -- 24        25 -- 28        29 -- 32
%           A{1}.pPos.Xd'   A{1}.pPos.X'    A{1}.pSC.Ud'    A{1}.pSC.U'
%
%           33 -- 44        45 -- 56        57 -- 60        61 -- 64
%           A{2}.pPos.Xd'   A{2}.pPos.X'    A{2}.pSC.Ud'    A{2}.pSC.U'
%
%           65 -- 76        77 -- 88        89 -- 90        91 -- 92
%           P.pPos.Xd'      P.pPos.X'       P.pSC.Ud'       P.pSC.U'
% 
%           93 -- 101       102 -- 110      111 -- 119      120 -- 128
%           Qd'             Q'              Xd'             X'
% 
%           129 -- 140      141 -- 152      153
%           A{1}.pPar.Xr'   A{2}.pPar.Xr'   toc(T_Alfa)

% Enviando os sinais de controle para os Drones
% A{1}.rSendControlSignals;   % Drone 1
A{2}.rSendControlSignals;   % Drone 2

% Usando ROS para enviar os sinais de controle para o Pioneer
if false
cmd_vel.Linear.X = P.pSC.Ud(1); % Velocidade u
cmd_vel.Angular.Z = P.pSC.Ud(2);% Velocidade w
send(pub,cmd_vel)               % Publicando na rede
end

% drawnow
end
if toc(TP) < T_PLOT
TP = tic; % Reiniciando o temporizador do plot
    
drawnow
end
end

A{2}.rLand;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           PLOTAR RESULTADOS                             %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

PlotarResultadosBaju



















