%% Controle de Formação Baseado em Espaço Nulo
% ICUAS 2019
% Mauro Sérgio Mafra

%% Referência de Modelo Convencional (Marcos)
% Resetar 
clear all;   
close all;
warning off; 
clc;

%% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

try
    fclose(instrfindall);
end

%% Load Class
try     
    % Load Classes
    %setenv('ROS_IP','192.168.0.103')
    %setenv('ROS_MASTER_URI','http://192.168.0.103:11311');
    RI = RosInterface; 
    RI.rConnect('192.168.0.166');    
    P = RPioneer;    
    
    disp('### Load Class Success');
    
catch ME
    disp(' #### Load Class Issues ');
    disp('');
    disp(ME);
    return;
end



%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);


% Time variables initialization
timeout = 60;   % maximum simulation duration
sampleTime = 1/3 0;

% timeout = size(Qd,1)*time + 30;
t  = tic;
tc = tic;
tp = tic;
t1 = tic;        % pioneer cycle
t2 = tic;        % ardrone cycle
index = 1;

cont = 1;        % counter to change desired position through simulation
time = 60;       % time to change desired positions [s]
landTime = 3;   % 10 Seconds to Lend

pause(3);
disp('Proceeded to Loop Control!.....');

try 
    
    % Loop while error > erroMax
    while toc(t)< (time)
        
        if toc(tc) > sampleTime        
            tc = tic;                         
            
            % Test receive data
            P.rGetSensorDataOpt;
            P.pPos.X
            
            % Senoid Signal
            Vref = 0.2*cos(0.5*toc(t));  
                        
            P.pSC.Ud(1) = Vref; % v{k}(4)*ganho; % Frente/Tras [-1,1] (+) Avanï¿½a, Move frente para baixo
            P.pSC.Ud(2) = 0; % -v{k}(5)*ganho; % Esquerda/Direita [-1,1] (+) Move Drone para Esquerda                        
            P.pSC.Ud(3) = 0; % -v{k}(6)*ganho; % Velocidade Vertical [-1,1] (+) Eleva o drone
            P.pSC.Ud(4) = 0; % Não Rotaciona 
            P.pSC.Ud(5) = 0; % Não Rotaciona
            P.pSC.Ud(6) = 0; % Angulo do drone [-1,1] (+) rotaciona para esquerda em torno do Eixo Z 
                        
            % Test send commands
            P.rCommand;   
            drawnow;
            
        end
        
        if btnEmergencia ~= 0 
           disp('Pioneer Stopping');
           P.rCmdStop;  
           break;
        end          
        
    end
    
    % Send Command to Land
    P.rCmdStop;           
    
catch ME
    
    disp('');
    disp(ME);
    disp('');
    
    % Send Land Command
    P.rCmdStop;  
    
    % Fecha o cliente ROS
    RI.rDisconnect;
    %rosshutdown;
end



% Send 3 times Commands 1 second delay to Drone Land
for i=1:3
    P.rCmdStop;
    pause(1);
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;