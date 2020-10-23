clear all; 
close all;
warning off;
clc;

% Carrega o diretï¿½rio corrente e subdiretï¿½rios 

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))


%% Load Data Test
dataPath = strcat(pwd,'\ROS\Dynamic Model Identification\Data\');
wsName = 'WS_Calibracao.mat';
varCal = 'PSI';
fileName = wsName;
fullFileName = strcat(dataPath,fileName);

if exist(fullFileName)
    load(fullFileName);
else
    serialNumber = 0;
end
btnEmergencia = 0;   
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% Drone  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
try
    fclose(instrfindall);
end


%% Load Class
try
    % Load Classes
    RI = RosInterface;
    %     setenv('ROS_MASTER_URI','http://192.168.0.106:11311')
    %     setenv('ROS_IP','192.168.0.101')
    %     RI.rConnect('172.20.24.221');
    RI.rConnect('192.168.0.166');
    B = Bebop(1,'B');
    
%     OPT = OptiTrack;
%     OPT.Initialize;
%     pause(1);
%     rb = OPT.RigidBody;            % read optitrack data
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
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

k1 = 0.8417;
k2 = 0.18227;
% k3 = 0.86877;
% k4 = 0.22695;
k3 = 0.8354;
k4 = 0.17095;
k5 = 3.966;
k6 = 4.001;
k7 = 9.8524;
k8 = 4.7295;

f1 = [k1  0  0  0;
    0 k3  0  0;
    0  0 k5  0;
    0  0  0 k7];

f2 = [k2  0  0  0;
    0 k4  0  0;
    0  0 k6  0;
    0  0  0 k8];

p = zeros(30/0.02,4);
v = zeros(30/0.02,4);
a = zeros(30/0.02,4);

t = tic;
x(1) = 0;
y(1) = 0;
z(1) = 1;
phi(1) = 0;
phid = 0;

% Inicializa Parametros de configuraï¿½ï¿½o do teste
tmax = 300; % Tempo Simulacao em segundos
kA = 0.4;
kw = 0.1*2*pi;
% kw = abs(10*2*pi*sin(2*pi/300));

% Tempo de Amostragem
To = 1/50;

% =========================================================================
%t  = tic; % Tempo de simulacao
tc = tic; % Tempo de controle
tp = tic; % Tempo de plotagem e exibiï¿½ï¿½o
nLandMsg = 3;    % Numero de mensagens enviadas para pouso do drone
dLandMsg = 1; % Atraso de tempo entre messagens de pouso


%% Decolagem do Drone

% Assegura a decolagem vertical 
% Os valores de comando na decolagem sejam iguais a zero
B.pSC.Ud(1) = 0; % Esquerda/Direita [-1,1] (-) Move Drone para Esquerda
B.pSC.Ud(2) = 0; % Frente/Tras [-1,1] (-) Avanï¿½a - Move frente para baixo
B.pSC.Ud(3) = 0; % Velocidade Vertical [-1,1] (+) Eleva o drone
B.pSC.Ud(4) = 0; %                 

countPrint = 0;
countWhile = 0;
countControl = 0;
countModel = 0;

% Inicializa o tempo de simulação

comandos = [];
poses = [];
velocidades = [];
tempo = [];
Bebop = [];
tz = 15;
elev = 0;

% Envia Comando de Decolagem para o Drone e aguarda estabilizaï¿½ï¿½o de
% altitude padrao (8s)
B.rTakeOff;
pause(4);

disp(" ");
disp("Final da Estabilizaï¿½ao .... Controlador ON...")
disp(" ");

%t  = tic; 
B.rGetSensorDataOpt
pPosAnt = B.pPos.X(6);
B.pPos.X(12) = (B.pPos.X(6) - pPosAnt)/To;
t  = tic; 
t_inc = tic;
%%
 try    
    while toc(t) < tmax
                      
       if toc(tc) > To            
            tc = tic;
                    
            countWhile = countWhile + 1; 

            %B.rGetSensorData;
            B.rGetSensorDataOpt;

            Vref = kA/2*(sin(kw*toc(t))+ sin(2*kw*toc(t)));
%             Vref = kA/(3 + 1 + .5)*(3*sin(kw*toc(t)) + 1*sin(3*kw*toc(t)) + .5*sin(5*kw*toc(t)));
%             Vref = kA/(3 + 1 + .5)*(3*sin(.5*kw*toc(t)) + 1*sin(5*kw*toc(t)) + .5*sin(7*kw*toc(t)));

                            
            B.pSC.Ud(1) = 0; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B.pSC.Ud(4) = 0; % Não Rotaciona 
            B.pSC.Ud(5) = 0; % Não Rotaciona
            B.pSC.Ud(6) = Vref; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
            
            B.pPos.X(12) = (B.pPos.X(6) - pPosAnt)/toc(t_inc);
            pPosAnt = B.pPos.X(6);
            
            comandos = [comandos; B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)];
            poses = [poses; B.pPos.X(1) B.pPos.X(2) B.pPos.X(3) B.pPos.X(6)];
            velocidades = [velocidades; B.pPos.X(7) B.pPos.X(8) B.pPos.X(9) B.pPos.X(12)];
            Bebop = [Bebop;B];
            tempo = [tempo toc(t)];            
            
            if btnEmergencia ~= 0 
                disp('Bebop Landing');
                B.rLand;  
                break;
            end
            
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)
            
            % Envia Comando para o Drone
            B.rCommand;
            
            countModel = countModel + 1;
            
            a(countModel,:) = (f1*[B.pSC.Ud(1) B.pSC.Ud(2) B.pSC.Ud(3) B.pSC.Ud(6)]' - f2*v(countModel,:)')';
            v(countModel+1,:) = v(countModel,:) + a(countModel,:)*toc(t_inc);
            t_inc = tic;
            
            drawnow;
        
       end
       
       % 
       if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
           disp('Bebop Landing through Emergency Command ');
           B.rCmdStop;
           B.rLand;
           break;
       end                     
    end
   
 catch Exception 
      B.rCmdStop;
      B.rLand;          
      disp("Pouso forcado via comando da estrutura Try Catch");
      disp(toStringJSON(Exception));
 end
 

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    B.rCmdStop;
    B.rLand
    pause(dLandMsg);
end

%%

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Comando de Pouso ao final do programa");
figure
hold on
plot(tempo,velocidades(:,4),'b')
plot(tempo,v(1:length(tempo),4),'r')
title('Velocidades')
xlabel('Tempo(s)');
ylabel('Vel');
