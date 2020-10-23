clear; close all; clc;
try
    fclose(instrfindall);
catch
end
%
% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.129');
    B = Bebop(1,'B');

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



%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);
% Beboop
disp('Start Take Off Timming....');
B.rTakeOff;
pause(5);
disp('Taking Off End Time....');

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 1/5; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 200; % tempo maximo do experimento

Xd(:,1) = [1 0 .5 0]'; % posicasao desejada x, y, z, psi

B.pPos.Xd(1:3) = Xd(1:3,1); % popula posicao desejada x, y, z
B.pPos.Xd(6) = Xd(4,1);     % popula posicao desejada psi

fprintf('\nStart..............\n\n');

t  = tic; % tempo corrido
t_control = tic;  % tempo de contragem do passo do controle

B.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ]'; % modelo lantado
B.pPar.Ts = 1/5; % popula variavel tempo de amostragem do Bebop

try
    while toc(t) < T_MAX % tempo corrido < tempo maximo
        
        if toc(t_control) > T_CONTROL % tempo contado do passo < tempo fixo do passo
            
            t_control = tic; % zera a contagem a cada loop

            B.rGetSensorData; %popula variavel de posiao e velocidade
                       
            B.cInverseDynamicController_Compensador; % controle posição e trajetória

%             B.pSC.Ud = [0 0 0 0 0 0]';
            B = J.mControl(B);                    % joystick command (priority)
            B.pSC.Ud % print / log da ação de controle desejada enviada ao controle
            B.rCommand; % função de envio da variavel Ud ao robo 
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    B.rCmdStop;
                    B.rLand;
                end
                break;
            end
            
            
        end
    end
catch ME % exception da try da while
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B.rLand
    
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;
    B.rLand
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

