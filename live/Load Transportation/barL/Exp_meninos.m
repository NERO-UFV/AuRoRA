clear; close all; clc;
try
    fclose(instrfindall);
catch
end
rosshutdown;

%
% % Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

%% Load Class
    % Load Classes
      % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    % Initiate classes
    P = Pioneer3DX(1);
    idP = getID(OPT,P); % ID do Bebop
    
    % Joystick
    J = JoyControl;
    
% Network
r = ROSNetwork;
r.InitROS('/master')
r.InitROS('/robot1','192.168.0.158')
P.pSC.Ud = [0 0]';
r.PublisherROS(r.node,'robot1/vel');

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
                         'String', 'land', ...
                         'Callback', 'btnEmergencia=1', ...
                         'Position', [50 50 400 300]);

%% Variable initialization
data = [];

% Time variables initialization
Xd = [0 0 0 0];
dXd = [0 0 0 0];

fprintf('\nStart..............\n\n');


% timers
T_exp = 60; % tempo de experimento
T_run = 1/30;       % Tempo de amostragem do experimento
t_run = tic;
t_exp = tic;


t  = tic;

pgains = [1.5 1 1.5 1];
P.pPar.ti = tic;
P.pPar.Ts = 1/30;



    while toc(t) < T_exp
        
        if toc(t_run) > T_run
            
            t_run = tic;
            t_atual = toc(t);

            
%% TAREFA

%Trajetória   

%% Ciclo 1

            

%% LEITURA DE POSIÇÃO E VELOCIDADE
       
% OPTITRACK
        rb = OPT.RigidBody;
        if rb(idP).isTracked
            P = getOptData(rb(idP),P);
        end
%             


%% CONTROLE DOS ROBÔS

% Pioneer
        P = fControladorCinematico(P,pgains);        
        P.pPar.ti = tic;
        
        
% Atribuindo comandos
        r.SendROS('robot1/vel',P.pSC.Ud);

            
%% EMERGÊNCIA
        drawnow
        if btnEmergencia == 1
            P.pFlag.EmergencyStop = 1;
        end
    
        if btnEmergencia ~= 0 || P.pFlag.EmergencyStop ~= 0 || A.pFlag.EmergencyStop ~= 0
            disp('Pioneer stopping by  ');

            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                P.pSC.Ud = [0 0]';
                r.SendROS('robot1/vel',P.pSC.Ud);
            end
            break;
        end   
            
        end
    end
%% EMERGÊNCIA FORA DO LAÇO    
    for i=1:nLandMsg
        P.pSC.Ud = [0 0]';
        r.SendROS('robot1/vel',P.pSC.Ud);

    end     

%% Send control signals
P.pSC.Ud = [0  ;  0];
for ii = 1:50
 r.SendROS('robot1/vel',P.pSC.Ud);
end
