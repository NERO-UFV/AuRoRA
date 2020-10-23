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
A{1} = ArDrone(2);
A{2} = ArDrone(1);
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
theta = 0;                  % phi (Ângulo da Formação em relação ao eixo y)
psif = pi/2;                % psi (Ângulo do eixo z)
phi = 0;                    % theta (Ângulo do plano da Formação)
beta = pi/4;                % beta (Ângulo interno da Formação)

% Valores iniciais dos robôs
% Coletando os dados dos corpos rígidos do OptiTrack
rb = OPT.RigidBody;
P.pPos.X(1:3) = [0 -1 0];
A{1}.pPos.X(1:3) = [0; -1; 1.5];

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

% RegiaoProtecaoBaju
figure
hold on
grid on
axis([-3 3 -3 3 0 3])
view(50,30)

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                          CONECTANDO NOS DRONES                          %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

if false
% Drone 1 (Em cima do Pioneer)
% A{1}.pPar.ip = '192.168.1.30';
A{1}.rConnect;
end
if true
% Drone 2 (Fora do Pioneer)
% A{2}.pPar.LocalPortControl = 5558;
% A{2}.pPar.LocalPortState = 5552;
% A{2}.pPar.ip = '192.168.1.40';
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
T_ETAPA = [10 20 10 20 20];   % Tempo das etapas
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
X = [P.pPos.X(1:3); A{2}.pPos.X(1:3)];
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
            disp(ETAPA)
            W = 2*pi/T_ETAPA(ETAPA);
        end
    case 2
        if CilindroFlag == 1 %toc(T) > T_ETAPA(ETAPA) || CilindroFlag == 1
            LF{A{2}.pID} = LineFormationBaju(A{2}.pID);
            LF{A{2}.pID}.pPar.K1 = 1*diag([0.2 0.2 0.2 3 1 1.8]);        % kinematic control gain  - controls amplitude
            LF{A{2}.pID}.pPar.K2 = 1*diag([1 1 1 1 1 1]);                % kinematic control gain - control saturation
            LF{A{2}.pID}.pPos.X = [P.pPos.X(1:3); A{2}.pPos.X(1:3)];
            LF{A{2}.pID}.fDirTrans;
            LF{A{2}.pID}.fFormationError;
            
            A{2}.pSC.Kinematics_control = 1;
            
            if norm(P.pPos.X([1:3]) - A{2}.pPos.X([1:3])) < RAIO(1)
                EsferaFlag = -1;
            else
                EsferaFlag = 0;
            end
            
            T = tic;    
            ETAPA = ETAPA + 1;
            disp(ETAPA)
        end
    case 3
        if toc(T) > T_ETAPA(ETAPA)% && EsferaFlag == 1 
%             break
%             A{2}.rLand;
            TF = TriangularFormationBaju;
            TF.pPar.K1 = 1*diag([0.2 0.2 0.2 1.8 1.8 1.8 3 3 1]);
            TF.pPar.K2 = 1*diag([1 1 1 1 1 1 3 3 1]);
%             TF.pPar.K1 = TF.pPar.K2;
%             TF.pPos.X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
            TF.pPos.X = [0; -1; 0;
                         0; -1; 1.5;
                         0; 0.0607; 1.0607];
            TF.tDirTrans;
            TF.tFormationError;
            
            T = tic;
            ETAPA = ETAPA + 1;
            disp(ETAPA)
        end
    case 4
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
        Xd{ETAPA} = [0;
                     1;
                     2.2;
                     0];
        A{2}.pPos.Xd([1:3 6]) = Xd{ETAPA};
        A{2}.pPos.Xd([7:9 12]) = [0;
                                  0;
                                  0;
                                  0];
    case 2
        if CilindroFlag == 0 && norm(P.pPos.X([1:2]) - A{2}.pPos.X([1:2])) > RAIO(2)
            A{2}.pPos.Xd([1:3 6]) = Xd{ETAPA-1} + (P.pPos.X([1:3 6]) - Xd{ETAPA-1})*toc(T)/T_ETAPA(ETAPA);
            A{2}.pPos.Xd([7:9 12]) = (P.pPos.X([1:3 6]) - Xd{ETAPA-1})/T_ETAPA(ETAPA);
        else
            CilindroFlag = 1;
        end
    case 3
%         if EsferaFlag == -1 && norm(P.pPos.X([1:3]) - A{2}.pPos.X([1:3])) < RAIO(1)
%             
        LF{A{2}.pID}.pPos.Qd = [P.pPos.X([1:3]);
                                RAIO(1);
                                pi/2;
                                beta];
%                                 
%         elseif EsferaFlag == 0 && norm(P.pPos.X([1:3]) - A{2}.pPos.X([1:3])) > RAIO(1)
%             
%             LF{A{2}.pID}.pPos.Qd = [P.pPos.X([1:3]);
%                                     RAIO(1);
%                                     pi/2;
%                                     0];
%         else
%             EsferaFlag = 1;
%         end
    case 4
        TF.pPos.Qd = [P.pPos.X(1:3);
                      -beta*toc(T)/T_ETAPA(ETAPA);
                      0;
                      psif;
                      RAIO(1);
                      RAIO(1);
                      beta];
        TF.pPos.dQd(4) = -beta/T_ETAPA(ETAPA);
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
%                          CONTROLE DE FORMAÇÃO                           %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

switch ETAPA
    case 1
        X = [P.pPos.X(1:3); A{2}.pPos.X(1:3)];
    case 2
        X = [P.pPos.X(1:3); A{2}.pPos.X(1:3)];
    case 3
        LF{A{2}.pID}.pPos.X = [P.pPos.X(1:3); A{2}.pPos.X(1:3)];
        X = LF{A{2}.pID}.pPos.X;
        LF{A{2}.pID}.fFormationControl;
        
        LF{A{2}.pID}.fInvTrans;
        
        A{2}.pPos.Xr(7:9) = LF{A{2}.pID}.pPos.dXr(4:6);
        A{2}.pPos.Xd(6) = 0;   %yaw desejado
        A{2}.pPos.Xd(12) = 0;  %vyaw desejado 
    case 4
        
        TF.pPos.X = [P.pPos.X(1:3); A{1}.pPos.X(1:3); A{2}.pPos.X(1:3)];
        X = TF.pPos.X;
        TF.tFormationControl;
        
        TF.tInvTrans;
        
        A{2}.pPos.Xr(7:9) = TF.pPos.dXr(7:9);
        A{2}.pPos.Xd(6) = 0;   %yaw desejado
        A{2}.pPos.Xd(12) = 0;  %vyaw desejado
        
        A{1}.pPos.X(1:3) = A{1}.pPos.X(1:3) + T_AMOSTRAGEM*TF.pPos.dXr(4:6);
%         A{2}.pPos.X(1:3) = A{2}.pPos.X(1:3) + T_AMOSTRAGEM*TF.pPos.dXr(7:9);
end

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           CONTROLE DOS ROBOS                            %
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
    case 3
%         A{1} = cInverseDynamicController_Compensador_ArDrone(A{1}); % Drone 1
        A{2} = cInverseDynamicController_Compensador_ArDrone(A{2}); % Drone 2
%         P = fDynamicController(P,cgains);      
    case 4
%         A{1} = cInverseDynamicController_Compensador_ArDrone(A{1}); % Drone 1
        A{2} = cInverseDynamicController_Compensador_ArDrone(A{2}); % Drone 2
%         P = fDynamicController(P,cgains);    
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
if toc(TP) > T_PLOT
TP = tic; % Reiniciando o temporizador do plot
    
try
    delete(H);
    delete(Point);
    delete(TPlot);
catch
end

H(1) = plot3([X(1) X(4)],[X(2) X(5)],[X(3) X(6)],'b','LineWidth',2);
try
H(2) = plot3([X(1) X(7)],[X(2) X(8)],[X(3) X(9)],'b','LineWidth',2);
H(3) = plot3([X(4) X(7)],[X(5) X(8)],[X(6) X(9)],'r','LineWidth',2);
catch
end
Point(1) = plot3(X(1),X(2),X(3),'ok','MarkerSize',10,'LineWidth',2);
Point(2) = plot3(X(4),X(5),X(6),'^k','MarkerSize',10,'LineWidth',2);
try
Point(3) = plot3(X(7),X(8),X(9),'^k','MarkerSize',10,'LineWidth',2);
TPlot(1) = text(X(1)+.1,X(2)+.1,X(3)+.1,'Pioneer','FontWeight','bold');
TPlot(2) = text(X(4)+.1,X(5)+.1,X(6)+.1,'ArDrone1','FontWeight','bold');
TPlot(3) = text(X(7)+.1,X(8)+.1,X(9)+.1,'ArDrone2','FontWeight','bold');
catch
end
if ETAPA == 4
    plot3(X(7),X(8),X(9),'.k');
else
    plot3(X(4),X(5),X(6),'.k');
end
drawnow
end
end

A{2}.rLand;

%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%
%                           PLOTAR RESULTADOS                             %
%- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -%

PlotarResultadosBaju



















