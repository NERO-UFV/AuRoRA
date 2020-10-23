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
    
    setenv('ROS_MASTER_URI','http://192.168.0.104:11311')
    setenv('ROS_IP','192.168.0.105')
    RI = RosInterface;
    RI.rConnect('192.168.0.104');
    
    P = RPioneer(1,'RosAria',2);
%     P2 = Pioneer3DX(1);

    J = JoyControl
    
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
data = [];

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 30;

fprintf('\nStart..............\n\n');

t  = tic;
t_control = tic;
t_int = tic;
t_derivada = tic;

rX = 2;           % [m]
rY = 2;           % [m]
T = T_MAX;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;
u_anterior = 0;
w_anterior = 0;

%%% optitrack
idP = 1;
idT1 = 2;
OPT = OptiTrack;
OPT.Initialize;
rb = OPT.RigidBody;          % read optitrack data
P = getOptData(rb(idP),P);   % get pioneer data
% P2 = getOptData(rb(idT1),P2);   % get pioneer data


u_w_d_hist = [];
u_w_hist = [];
t_hist = [];
u_p_hist = [];
w_p_hist = [];

%try
while toc(t) < T_MAX
    
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        %% POSIÇÃO
        
        
        % Dados Odometria
%         P.rGetSensorData;
        rb = OPT.RigidBody; 
        P = getOptData(rb(idP),P);   % get pioneer data
%         P2 = getOptData(rb(idT1),P2);   % get pioneer data
        
        % Sinal excitacao
%         u_ref = 0.001*(5*sin(ww*t_atual) - 0.5*sin(2*ww*t_atual) - 0.01*sin(5*ww*t_atual) - 4*sin(0.2*ww*t_atual) + 2*sin(4*ww*t_atual));
%         w_ref = 0.1*(sin(ww*t_atual) - 1.5*sin(2*ww*t_atual) - 2*sin(0.8*ww*t_atual) + 0.15*sin(0.2*ww*t_atual)- 0.55*sin(10*ww*t_atual));
%       
        %%% teste 1 - 30s
%         u_ref = 0.001*(0.1*sin(ww*t_atual) + 0.5*sin(2*ww*t_atual) - 0.4*sin(5*ww*t_atual) + 4*sin(0.2*ww*t_atual) + 2*sin(0.5*ww*t_atual));
%         w_ref = 0.2*(cos(ww*t_atual) - 1.5*cos(2*ww*t_atual) - 2*sin(0.8*ww*t_atual) + 0.15*sin(0.2*ww*t_atual)- 0.55*sin(10*ww*t_atual));
%         
        %%% teste 2 - 40s
        u_ref = 0.4*(-0.1*sin(4*ww*t_atual) + 0.5*sin(0.1*ww*t_atual) - 0.4*sin(0.8*ww*t_atual) + 4*sin(8*ww*t_atual) + 2*sin(0.1*ww*t_atual));
        w_ref = 0.05*(1.5*cos(0.15*ww*t_atual) - 0.2*sin(-0.8*ww*t_atual) + 0.85*sin(0.2*ww*t_atual) + 0.85*sin(0.1*ww*t_atual));
%         
        U = [u_ref;w_ref];
                       
        u_robo = P.pPos.X(7);
        
        w_robo = P.pPos.X(12);
        
        u_p = (u_robo - u_anterior)/toc(t_derivada);
        w_p = (w_robo - w_anterior)/toc(t_derivada);
        t_derivada = tic;
        u_anterior = P.pPos.X(7);
        w_anterior = P.pPos.X(12);             
                            
        %%% ---------------------------------------------------------
               
%         P.pSC.Ud(1:2) = [0;0];
        P.pSC.Ud(1:2) = U;
        
        P = J.mControl(P);                    % joystick command (priority)
        P.pSC.Ud
        P.rCommand;
        
        u_w_d_hist = [u_w_d_hist [P.pSC.Ud(1) P.pSC.Ud(2)]'];
        u_w_hist = [u_w_hist [u_robo w_robo]'];
        t_hist = [t_hist toc(t)];
        u_p_hist = [u_p_hist u_p];
        w_p_hist = [w_p_hist w_p];
        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        drawnow
        if btnEmergencia ~= 0
            disp('Bebop Landing through Emergency Command ');
            
            % Send 3 times Commands 1 second delay to Drone Land
            
            break;
        end
        
        
    end
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");


%%
path = 'C:\Users\lai1u\Dropbox\AuRoRA 2018\ROS\Scripts\DiegoCBA\simulacoes\optitrack\identificacao\';
filename = ['Diego_identificacao' num2str(T) '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
save(fullname,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist')
