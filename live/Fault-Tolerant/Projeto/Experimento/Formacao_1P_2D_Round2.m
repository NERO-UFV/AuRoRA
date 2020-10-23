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
A{1} = ArDrone;
A{2} = ArDrone;
P = Pioneer3DX;

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
cmd_vel.Angular.Z = 0.0;
send(pub,cmd_vel);
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                          VARIÁVEIS INICIAIS                             %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

% Ganhos
cgains = [ 0.35  0.35  0.80  0.80  0.75  0.75  0.12  0.035 ];   % Pioneer
L = diag([ 1   1   1   1   1   1     1   1   1 ]);              % Formação
%          x   y   z   phi psi theta p   q   beta

% Valores iniciais da Formação
phi = 0;                    % phi (Ângulo da Formação em relação ao eixo z)
psi = 0;                    % psi (Ângulo do eixo y)
theta = 0;                  % theta (Ângulo do plano da Formação)
beta = pi/4;                % beta (Ângulo interno da Formação)
raio(1) = 2;                % Raio da Esfera
raio(2) = raio(1)*beta;     % Raio do Cilindro

% Valores iniciais dos robôs
% Iniciando os timers dos robôs
A{1}.pPar.ti = tic;
A{2}.pPar.ti = tic;
P.pPar.ti = tic;
% Coletando os dados dos corpos rígidos do OptiTrack
rb = OPT.RigidBody;
P.pPos.X(1:3) = [0 0 0];

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
%                       PLOT INICIAL DO EXPERIMENTO                       %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

RegiaoProtecaoBaju

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                         CONECTANDO NOS DRONES                           %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

if false
% Drone 1 (Em cima do Pioneer)
A{1}.pPar.ip = '192.168.1.30';
A{1}.rConnect;
end
if false
% Drone 2 (Fora do Pioneer)
% A{2}.pPar.LocalPortControl = 5558;
% A{2}.pPar.LocalPortState = 5552;
A{2}.pPar.ip = '192.168.1.50';
A{2}.rConnect;

% ArDrone Takeoff
disp('Start Take Off Timming....');
% A{1}.rTakeOff;
A{2}.rTakeOff;
pause(5);
disp('Taking Off End Time....');
end

% Iniciando os temporizadores dos robôs
A{1}.pPar.ti = tic;
A{2}.pPar.ti = tic;
P.pPar.ti = tic;


%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                        VARIAVEIS DO EXPERIMENTO                         %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

% Variaveis temporais
T_MAX = 30;                 % Tempo total do experimento
T_AMOSTRAGEM = 1/30;        % Tempo de amostragem
T_PLOT = 0.5;               % Tempo para o ploit
T_ETAPA = [10 20 5 5 20];   % Tempo das etapas
T_ETAPINHAS = [5];          % Tempo das etapinhas
A{1}.pPar.Ts = 1/30;        % Tempo de amostragem do Drone 1
A{2}.pPar.Ts = 1/30;        % Tempo de amostragem do Drone 2

% Variáveis iniciais
ETAPA = 1;  % Definindo a etapa
DADOS = []; % Iniciando a matriz dos dados

% Iniciando os temporizadores
T_ALFA = tic;           % Temporizador TOTAL do experimento
T = tic;                % Temporizador do experimento
TA = tic;               % Temporizador da amostragem
TP = tic;               % Temporizador do plot
A{1}.pPar.ti = tic;     % Temporizador do Drone 1
A{2}.pPar.ti = tic;     % Temporizador do Drone 2

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                               EXPERIMENTO                               %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
while toc(T) < T_MAX
if toc(TA) < T_AMOSTRAGEM
TA = tic; % Reiniciando o temporizador da amostragem
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                         INICIALIZANDO AS ETAPAS                         %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
switch ETAPA
    case 1
        if toc(T) > T_ETAPA(ETAPA)
            break
            T = tic;
            ETAPA = ETAPA + 1;
        end
    case 2
        disp(ETAPA)
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           OBJETIVOS DAS ETAPAS                          %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
switch ETAPA
    case 1
        % Definindo a posição do Drone 2
        A{2}.pPos.Xd([1:3 6]) = [0;
                                 -1;
                                 1;
                                 0];
        A{2}.pPos.Xd([7:9 12]) = [0;
                                  0;
                                  0;
                                  0];
    case 2
        xd = 1;
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

% Enviando os sinais de controle para os Drones
% A{1}.rSendControlSignals;   % Drone 1
A{2}.rSendControlSignals;   % Drone 2

% Usando ROS para enviar os sinais de controle para o Pioneer
if false
cmd_vel.Linear.X = P.pSC.Ud(1); % Velocidade u
cmd_vel.Angular.Z = P.pSC.Ud(2);% Velocidade w
send(pub,cmd_vel)               % Publicando na rede
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                                  DADOS                                  %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

DADOS(end+1,:) = [A{1}.pPos.Xd' A{1}.pPos.X' A{1}.pSC.Ud' A{1}.pSC.U'...
                  A{2}.pPos.Xd' A{2}.pPos.X' A{2}.pSC.Ud' A{2}.pSC.U'...
                  P.pPos.Xd' P.pPos.X' P.pSC.Ud' P.pSC.U' toc(T_Alfa)];
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

drawnow
end
if toc(TP) < T_PLOT
TP = tic; % Reiniciando o temporizador do plot
    
drawnow
end
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           PLOTAR RESULTADOS                             %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

PlotarResultadosBaju



















