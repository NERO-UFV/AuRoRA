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
    RI.rConnect('192.168.0.103');
    P = RPioneer(1,'RosAria',1);

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
data = [];

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 45;

fprintf('\nStart..............\n\n');

t  = tic;
t_control = tic;
t_int = tic;
t_derivada = tic;

rX = 2;           % [m]
rY = 2;           % [m]
T = T_MAX/5;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;
u_anterior = 0;
w_anterior = 0;


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
        P.rGetSensorData;
        
        % Sinal excitacao
        u_ref = 0.1*(0.75*sin(ww*t_atual) + 0.02*sin(15*ww*t_atual) + 0.4*sin(2*ww*t_atual) + 3*sin(0.2*ww*t_atual) + 0.5*sin(0.5*ww*t_atual));
        w_ref = 0.6*(0.5*sin(ww*t_atual) - 1.1*sin(3*ww*t_atual) - 0.5*sin(0.2*ww*t_atual) + 0.75*sin(0.3*ww*t_atual));
        
        U = [u_ref;w_ref];
                       
        u_robo = P.pPos.X(7);
        
        w_robo = P.pPos.X(12);
        
        u_p = (u_robo - u_anterior)/toc(t_derivada);
        w_p = (w_robo - w_anterior)/toc(t_derivada);
        t_derivada = tic;
        u_anterior = P.pPos.X(7);
        w_anterior = P.pPos.X(12);             
                            
        %%% ---------------------------------------------------------
               
        P.pSC.Ud(1:2) = U;
        
        P = J.mControl(P);                    % joystick command (priority)
        P.rCommand;
        
        u_w_d_hist = [u_w_d_hist [u_ref w_ref]'];
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
path = [pwd '\ROS\Scripts\DiegoCBA\Identificacao Pioneer\'];
filename = ['Diego_validacao' num2str(T) '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
save(fullname,'t_hist','u_w_d_hist','u_w_hist','u_p_hist','w_p_hist')