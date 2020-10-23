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
%     B = Bebop(1,'B');
    
%     P = Pioneer3DX(1);  % Pioneer Instance
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
data = [];

% Time variables initialization
T_CONTROL = 1/10; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 30;

Xd_i = 1;
Xd(:,1) = [0.5 0 0 0]';

P.pPos.Xd(1:3) = Xd(1:3,Xd_i);
P.pPos.Xd(6) = Xd(4,Xd_i);

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_derivada = tic;

rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;

pgains = [0.35 0.35 0.8 0.8];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

k_u = 4;
k_w = 4;

%%% thetas primeira identificacao
% theta_1 = 0.23025;
% theta_2 = 0.22615;
% theta_3 = 0.00028953;
% theta_4 = 0.95282;
% theta_5 = 0.021357;
% theta_6 = 0.95282;

%%% thetas filipe martins
% theta_1 = 0.2604;
% theta_2 = 0.2509;
% theta_3 = -0.000499;
% theta_4 = 0.9965;
% theta_5 = 0.00263;
% theta_6 = 1.0768;

%%% thetas segunda identificacao
theta_1 = 0.22475;
theta_2 = 0.19672;
theta_3 = 0.0026587;
theta_4 = 0.97439;
theta_5 = 0.026025;
theta_6 = 0.93105;

u_ref_anterior = 0;
w_ref_anterior = 0;

U_hist = [];

Theta = [theta_1;theta_2;theta_3;theta_4;theta_5;theta_6];

xd_hist = [];
x_hist = [];
angle1_hist = [];
angle2_hist = [];
t_hist = [];

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            
            
            % Dados Odometria
            P.rGetSensorData;
            
            P.pPos.Xd([1 2]) = [rX*sin(ww*t_atual + phase)+P.pPar.a;-rY*cos(ww*t_atual + phase)+rY];
            P.pPos.Xd([7 8]) = [ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];
            
         %%%% -------------   compensador cinematico ---------------
           
          K = [cos(P.pPos.X(6)), -P.pPar.a*sin(P.pPos.X(6)); ...
               sin(P.pPos.X(6)), P.pPar.a*cos(P.pPos.X(6))];
                        
          P.pPos.Xtil = P.pPos.Xd - P.pPos.X;
            
          sinal_cinematico = K\(P.pPos.Xd(7:8) + Kp1*tanh(Kp2*P.pPos.Xtil(1:2)));
          
          u_ref = sinal_cinematico(1);
          w_ref = sinal_cinematico(2);
          
          %%%%%%%%%%% ------------------------------------------
          
          %%%% -------------   compensador dinâmico ---------------
                   
          A = [theta_1 0; 0 theta_2];
          
          u_p_ref = (u_ref - u_ref_anterior)/toc(t_derivada);
          w_p_ref = (w_ref - w_ref_anterior)/toc(t_derivada);
          t_derivada = tic;
          u_ref_anterior = u_ref;
          w_ref_anterior = w_ref;
          
          G = [0 0 -P.pPos.X(12)^2 P.pPos.X(7) 0 0; ...
               0 0 0 0 P.pPos.X(12)*P.pPos.X(7) P.pPos.X(12)];
          
          delta_1 = u_p_ref + k_u*(u_ref - P.pPos.X(7));
          delta_2 = w_p_ref + k_w*(w_ref - P.pPos.X(12));
          
          P.pSC.Ud(1:2) = A*[delta_1;delta_2] + G*Theta;
          
          %%%%%%%%%%% ------------------------------------------
          
%           P.pSC.Ud(1:2) = sinal_cinematico;

          P = J.mControl(P);                    % joystick command (priority)

          P.rCommand;
          
          xd_hist = [xd_hist P.pPos.Xd(1:2)];
          x_hist = [x_hist P.pPos.X(1:2)];
          t_hist = [t_hist toc(t)];
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
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

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(x_hist(1,:),x_hist(2,:));
legend('desejado','real')
xlabel('X [m]');
ylabel('Y [m]');

figure();
hold on;
grid on;
plot(t_hist(1,:),xd_hist(1,:) - x_hist(1,:));
plot(t_hist(1,:),xd_hist(2,:) - x_hist(2,:));
legend('erro x','erro y')
xlabel('t');
ylabel('erro');

