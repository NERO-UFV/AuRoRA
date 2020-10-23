% Initial Comands
clear variables; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Class
try
    % Load Classes
    RI = RosInterface; % 
    setenv('ROS_IP','192.168.0.158') % IP do computador que está rodando o código principal
    setenv('ROS_MASTER_URI','http://192.168.0.146:11311') % IP do computador que está rodando o mestre
    RI.rConnect('192.168.0.146'); % Conectar ao mestre
    
    B{1} = Bebop(1,'B1');
    
    % Joystick
    J = JoyControl;
    
    disp('  -------------------  Load Class Success  -------------------');
    
catch ME
    disp(' ');
    disp('  -------------------  Load Class Issues  -------------------');
    disp(' ');
    disp(' ');
    disp(ME);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

% Botão de Emergencia
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% detect rigid body ID from optitrack
idB{1} = getID(OPT,B{1},1);    % drone ID on optitrack

% Take Off
disp('Start Take Off Timming....');
B{1}.rTakeOff;
pause(3);
disp('Taking Off End Time....');


%% Creating the virtual robots

% Robot initial pose
rb = OPT.RigidBody;          % read optitrack data
B{1} = getOptData(rb(idB{1}),B{1});
B{1}.pPar.Ts = 0.030;

T_BEBOP = B{1}.pPar.Ts; %1/30

T = 30;   % [s]

% Data variables
kk = 1;
data = zeros(round(T/T_BEBOP),23); % Data matrix

t  = tic;

t_Bebop_1 = tic;

try
    while toc(t)< T

        % Laço de controle dos robôs
        if toc(t_Bebop_1) > T_BEBOP
            t_Bebop_1 = tic;
            rb = OPT.RigidBody; % read optitrack data
            B{1} = getOptData(rb(idB{1}),B{1});   % Adquirir dados dos sensores - ArDrone

            Vref = 0.5*sin(2*pi/5*toc(t));            
                            
            B{1}.pSC.Ud(1) = Vref; %Vref; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            B{1}.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            B{1}.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            B{1}.pSC.Ud(4) = 0; % Angulo do drone [-1,1] (+) rotaciona para Direita em torno do Eixo Z 
            B{1}.pSC.Ud(5) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda 
            B{1}.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
            
            B{1} = J.mControl(B{1});
            B{1}.rCommand;               % Enviar sinal de controle para o robô

            % Variable to feed plotResults function    
            data(kk,:) = [  B{1}.pPos.X'    B{1}.pSC.Ud'    B{1}.pSC.U'   toc(t)];

            kk = kk + 1;
            
        end
        
        drawnow;
        B{1}.pFlag.EmergencyStop = 0;

        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        if btnEmergencia ~= 0 || J.pFlag == 1 || ...
                B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1
            disp('Bebop Landing through Emergency Command ');
            B{1}.rCmdStop;
            B{1}.rLand;
            break;
        end
            
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{1}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    
end

%% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
    B{1}.rLand
    pause(1);
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");
