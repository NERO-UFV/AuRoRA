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
    P = RPioneer(1,'RosAria',2);
    
    % Joystick
    J = JoyControl;
    
    % Create OptiTrack object and initialize
    %     OPT = OptiTrack;
    %     OPT.Initialize;
    %     idB = getID(OPT,B); % ID do Bebop
    
    
    
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
T_MAX = 60;

fprintf('\nStart..............\n\n');

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
t_int = tic;
t_derivada = tic;

rX = 1.5;           % [m]
rY = 1.5;           % [m]
T = T_MAX;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;
L0 = 0.3;
L1 = 0.455;
L10 = 0.28;
L2 = 0.455;
P.pPar.a = 0.19;
Xda = 0;

pgains = [0.3 0.3 0.5 0.5];
Kp1 = diag([pgains(1), pgains(2)]);
Kp2 = diag([pgains(3), pgains(4)]);

Kp1 = 0.6;
Kp2 = 0.5;

xd_hist = [];
x_hist = [];
angle1_hist = [];
angle2_hist = [];
t_hist = [];
t1_hist = [];
vel_hist = [];
t0_c_hist = [];
t1_c_hist = [];
t0_ct_hist = [];
pose_final_trailer_hist = [];
centro_robo_hist = [];
control_hist = [];
theta_eq_hist = [];
controlT_hist = [];
w0_c_hist = [];
x_atras_hist = [];
centro_trailer_hist = [];
theta1_giro = 25;
a = 0.15;
b = 0.2;
alpha = pi;
% z = 0.4;

%try
while toc(t) < T_MAX
    
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        %% POSIÇÃO
        
        
        % Dados Odometria
        P.rGetSensorData;
        P.rGetPotentiometerData;
        
        P.pPos.Xd([1 2]) = [-rX*sin(ww*t_atual + phase) - (L0+L1+L10+L2+b);-rY*cos(ww*t_atual + phase)+rY];
        P.pPos.Xd([7 8]) = [-ww*rX*cos(ww*t_atual + phase);ww*rY*sin(ww*t_atual + phase)];
        
        
        x_t1 = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);
        y_t1 = P.pPos.Xc(2) - L0*sin(P.pPos.X(6)) - (L1)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
        
        x_t2 = x_t1 - L10*cos(P.pPos.X(6) - P.pPot1.Data*pi/180) - (L2+b)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180 - P.pPot2.Data*pi/180);
        y_t2 = y_t1 - L10*sin(P.pPos.X(6) - P.pPot1.Data*pi/180) - (L2+b)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180 - P.pPot2.Data*pi/180);
        
        pose_final_trailer = [x_t2; y_t2];
        
        centro_robo = [P.pPos.Xc(1); P.pPos.Xc(2)];
        
        
        P.pPos.Xtil = P.pPos.Xd(1:2) - pose_final_trailer;
        
        %%% ---------------------------------------------------------
        
        %%% ----- Trailer como Pioneer - modelo direto e controle para obter v1 e w1
        %%% neccesarios para seguimento de trajetória;
        
        angulo_trailer_mundo_1 = P.pPos.Xc(6) - (P.pPot1.Data*pi/180);
        angulo_trailer_mundo_2 = angulo_trailer_mundo_1 - (P.pPot2.Data*pi/180);
        
        K_inv = [ cos(angulo_trailer_mundo_2), sin(angulo_trailer_mundo_2), b*sin(alpha); ...
            -sin(angulo_trailer_mundo_2), cos(angulo_trailer_mundo_2), -b*cos(alpha); ...
            0, 0, 1];
        
        vx = (P.pPos.Xd(7) + Kp1*tanh(Kp2*P.pPos.Xtil(1)));
        vy = (P.pPos.Xd(8) + Kp1*tanh(Kp2*P.pPos.Xtil(2)));
        v_fi = (-vx/(b*cos(alpha)))*sin(angulo_trailer_mundo_2) +  (vy/(b*cos(alpha)))*cos(angulo_trailer_mundo_2);
        
        sinal_controle_trailer = K_inv*[vx; ...
            vy;...
            v_fi];
        
        v2 = sinal_controle_trailer(1);
        w2 = sinal_controle_trailer(3);
        
        v1 = v2*cos(P.pPot2.Data*pi/180) + (w2)*(L2)*sin(P.pPot2.Data*pi/180);
        w1 = v2*sin(P.pPot2.Data*pi/180)/L10 - (w2)*(L2)*cos(P.pPot2.Data*pi/180)/L10;
        
        v0_c = v1*cos(P.pPot1.Data*pi/180) + (w1)*(L1)*sin(P.pPot1.Data*pi/180);
        w0_c = v1*sin(P.pPot1.Data*pi/180)/L0 - (w1)*(L1)*cos(P.pPot1.Data*pi/180)/L0;
        
        P.pSC.Ud(1:2) = [v0_c; w0_c];
        
        P = J.mControl(P);                    % joystick command (priority)
        P.rCommand;
        
        xd_hist = [xd_hist P.pPos.Xd(1:2)];
        x_hist = [x_hist P.pPos.X(1:2)];
        %           angle1_hist = [angle1_hist P.pPot1.Data];
        %           angle2_hist = [angle2_hist P.pPot2.Data];
        t_hist = [t_hist toc(t)];
        %           t1_hist = [t1_hist t1];
        %           vel_hist = [vel_hist vel];
        %           control_hist = [control_hist P.pSC.Ud(1:2)];
        %           t0_c_hist = [t0_c_hist t0_c];
        pose_final_trailer_hist = [pose_final_trailer_hist pose_final_trailer];
        centro_robo_hist = [centro_robo_hist centro_robo];
        %           theta_eq_hist = [theta_eq_hist theta_eq];
        controlT_hist = [controlT_hist [v1 w1]'];
        w0_c_hist = [w0_c_hist [-v1*sin(P.pPot1.Data*pi/180)/L0  -(-w1)*(L1)*cos(P.pPot1.Data*pi/180)/L0]' ];
        %           centro_trailer_hist = [centro_trailer_hist pose_centro_trailer];
        %           x_atras_hist = [x_atras_hist X_atras];
        
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

figure();
hold on;
grid on;
plot(t_hist(1,:), controlT_hist(1,:));
plot(t_hist(1,:), controlT_hist(2,:));
legend('sinal controle v1', 'sinal controle w1');
title('sinais controle');
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(t_hist(1,:), w0_c_hist(1,:));
plot(t_hist(1,:), w0_c_hist(2,:));
legend('w0_t', 'w0_r');
title('W0 sinal controle');
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(pose_final_trailer_hist(1,:),pose_final_trailer_hist(2,:));
% plot(centro_robo_hist(1,:),centro_robo_hist(2,:));
legend('Desejado','final Trailer', 'Centro robo');
xlabel('X');
ylabel('Y');

figure();
hold on;
grid on;
plot(t_hist(1,:), xd_hist(1,:) - pose_final_trailer_hist(1,:));
plot(t_hist(1,:), xd_hist(2,:) - pose_final_trailer_hist(2,:));
legend('erro trailer x', 'erro trailer y');
title('erro posições');
xlabel('t');
ylabel('sinal');
%%
path = [pwd '\ROS\Scripts\DiegoCBA\'];
filename = ['Diego_re_circulo_2Trailers_video_T' num2str(T) '_Data_' datestr(now,30) '.mat'];
fullname = [path filename];
% save(fullname,'t_hist','controlT_hist','w0_c_hist','xd_hist','pose_final_trailer_hist','centro_robo_hist','pose_final_trailer_hist')
