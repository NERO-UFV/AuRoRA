clear; close all; clc;
try
    fclose(instrfindall);
catch
end
%
% % Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
%     setenv('ROS_IP','192.168.0.153')
    RI.rConnect('192.168.0.129');
    
    P = RPioneer(1,'RosAria');
    
    
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

%% Variable initialization

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 60;

fprintf('\nStart..............\n\n');

t  = tic;
t_control = tic;

% pgains = [0.35 0.35 0.8 0.8];
% Kp1 = diag([pgains(1), pgains(2)]);
% Kp2 = diag([pgains(3), pgains(4)]);

K = [1.7453 1];

T = 5;
w = 2*pi/T;

xd_hist = [];
x_hist = [];
t_hist = [];

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
            %% POSIÇÃO
            
            gammad = -90*pi/180;
            gammadp = 0;
            
            % Dados Odometria
            P.rGetSensorData;
            gammatil = gammad - P.pPos.X(6);
            
            if abs(gammatil) > pi
                gammatil = gammatil - 2*pi*sign(gammatil);
            end
            
%             K = [ cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
%                 sin(P.pPos.X(6)), +P.pPar.a*cos(P.pPos.X(6))];
%             P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%             P.pSC.Ud(1:2) = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));

            
            v = 0.1;
            w = gammadp + K(1)*tanh(K(2)*gammatil);
            P.pSC.Ud = [v w]';
            P = J.mControl(P);                    % joystick command (priority)
            P.pSC.Ud
            P.rCommand;

            drawnow
            if btnEmergencia ~= 0
                disp('Bebop Landing through Emergency Command ');
                
                % Send 3 times Commands 1 second delay to Drone Land
                
                break;
            end
            
            
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    P.rCmdStop;
    
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");
