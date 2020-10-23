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
    RI.rConnect('192.168.0.100');
    
    P = RPioneer(1,'RosAria',1);
    
    
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
T_MAX = 600;

fprintf('\nStart..............\n\n');

t  = tic;
t_control = tic;
t_der = tic;
t_integral = tic;

% pgains = [0.35 0.35 0.8 0.8];
% Kp1 = diag([pgains(1), pgains(2)]);
% Kp2 = diag([pgains(3), pgains(4)]);

K = [1.7453 0.3]; %ganhos Pioneer
Kth = 0.5; % ganho proporcional Theta_1

T = 5; % periodo trajetoria
w = 2*pi/T; % velocidade angular trajetoria

% A1 = 0.285; % dimensoses RMMA Diego - medidas fisicas
% A2 = 0.455; % dimensoses RMMA Diego - medidas fisicas

A1 = 0.285; % dimensoses RMMA Diego - medidas fisicas
A2 = 0.455; % dimensoses RMMA Diego - medidas fisicas
B1 = 0; % dimensoses RMMA Diego - medidas fisicas

xd_hist = [];
x_hist = [];
t_hist = [];
gammad_old = 0;
lastError = 0;
iError = 0;
output = 0;

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
            %% POSIÇÃO
            
            % Dados Odometria
            P.rGetSensorData;
            P.rGetPotentiometerData;
            
            th_ref = 15*pi/180; % valor de referencia para Theta_1
            
%             gammad = 0*pi/180;
%             gammadp = 0;
%             gamma_giro_t = 0.5*(abs(asin((2*A1*sin(th_ref))/((A2)+B1*cos(th_ref)))))*sign(-th_ref);
            gamma_giro_dianteiro = (abs(atan((A1*sin(th_ref))/((A2)+B1*cos(th_ref)))))*sign(-th_ref);
            gamma_giro_fixo = gamma_giro_dianteiro*180/pi
            thtil = th_ref - (P.pPot1.Data)*(pi/180);
            P.pPot1.Data;

%             gammad = gamma_giro - Kth*thtil;
          
             iError = iError + thtil * toc(t_integral);
             dError = (thtil - lastError)/ toc(t_integral);
% %             PID Function
             output = Kth*thtil + 0*iError + 0*dError;
             lastError = thtil;
             t_integral = tic;

            gammad = gamma_giro_dianteiro - output;
            gamma_desejado = gammad*180/pi
             
%           gammad = gamma_giro_d - Kth*thtil
%           gammad = gamma_giro - Kth*thtil + P.pPos.X(6);
            gammadp = (gammad-gammad_old)/toc(t_der);
            t_der = tic;
            gammad_old = gammad;
            
            gammatil = gammad - P.pPos.X(6);
            erro_gamma = gammatil*180/pi
            
            if abs(gammatil) > pi
                gammatil = gammatil - 2*pi*sign(gammatil);
            end
            
%             K = [ cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
%                 sin(P.pPos.X(6)), +P.pPar.a*cos(P.pPos.X(6))];
%             P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
%             P.pSC.Ud(1:2) = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));

            
            v = 0;
            w = gammadp + K(1)*tanh(K(2)*gammatil);
%             w = 0.05;
            P.pSC.Ud = [v w]';
            P = J.mControl(P);                    % joystick command (priority)
%             P.rCommand;

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
