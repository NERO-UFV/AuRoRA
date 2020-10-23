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
T_MAX = 150;

fprintf('\nStart..............\n\n');

t  = tic;
t_control = tic;
t_int = tic;

rX = 1;           % [m]
rY = 1;           % [m]
T = T_MAX/2;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]
ww = 1*w;
phase = 0;
cont = 0;
L0 = 0.3;
L1 = 0.455;
P.pPar.a = 0.19;

kx = 3;
ky = 3;
kt = 6;

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
pose_centro_trailer_hist = [];
centro_robo_hist = [];
control_hist = [];

%try
% figure();

P.pPos.Xd(1) = -0.755;
while toc(t) < T_MAX
    
    if toc(t_control) > T_CONTROL
        
        t_control = tic;
        t_atual = toc(t);
        %% POSIÇÃO
        
        
        % Dados Odometria
        P.rGetSensorData;
        P.rGetPotentiometerData;
        
        r = 1.5;
        vd = 2*pi*r/T;
        wd = vd/r;

        P.pPos.Xd(1) = P.pPos.Xd(1) + P.pPos.Xd(7)*toc(t_int);
        P.pPos.Xd(2) = P.pPos.Xd(2) + P.pPos.Xd(8)*toc(t_int);
        P.pPos.Xd(6) = P.pPos.Xd(6) + P.pPos.Xd(12)*toc(t_int);
        
        t_int = tic;
        
        P.pPos.Xd(7) = vd*cos(P.pPos.Xd(6));
        P.pPos.Xd(8) = vd*sin(P.pPos.Xd(6));
        P.pPos.Xd(12) = wd;

 
        
        %%% -- Pose ponto final do Trailer (assumi sinal negativo para theta1 - potenciomero em relação
        %%% ao mundo (análise de giro do pioneer somente));
        
        
        %           y_t = P.pPos.Xc(2) + L0*sin(P.pPos.X(6)) + (L1-P.pPar.a)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
        %           x_t = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1-P.pPar.a)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);
%         x_t = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);
%         y_t = P.pPos.Xc(2) + L0*sin(P.pPos.X(6)) + (L1)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
%         P.pPos.X(6) = pi/2;
        
        x_t = P.pPos.Xc(1) - L0*cos(P.pPos.X(6)) - (L1)*cos(P.pPos.X(6) - P.pPot1.Data*pi/180);
        y_t = P.pPos.Xc(2) - L0*sin(P.pPos.X(6)) - (L1)*sin(P.pPos.X(6) - P.pPot1.Data*pi/180);
        pose_centro_trailer = [x_t; y_t];
        
        %%% -------------------------------------------------------------
        
        centro_robo = [P.pPos.Xc(1); P.pPos.Xc(2)];
        angulo_trailer_mundo = P.pPos.X(6) - P.pPot1.Data*pi/180;
%         angulo_trailer_mundo*180/pi;
        
        %%% ---------- erro em relacao ao ponto final do trailer
        
        P.pPos.Xtil(1:2) = P.pPos.Xd(1:2) - [x_t; y_t];
        P.pPos.Xtil(6) = P.pPos.Xd(6) - angulo_trailer_mundo;
        
        %%% ---------------------------------------------------------
        
        %%% ----- Trailer como Pioneer - modelo direto e controle para obter v1 e w1
        %%% neccesarios para seguimento de trajetória;

        K = [ cos(P.pPos.Xc(6)) sin(P.pPos.Xc(6))    0; ...
              -sin(P.pPos.Xc(6)) cos(P.pPos.Xc(6))   0; ...
                     0                   0           1];
                 
        XtilCor = K*P.pPos.Xtil([1 2 6]);
        
        sinal_controle_trailer(1) = vd*cos(XtilCor(3)) + kx*XtilCor(1);
        sinal_controle_trailer(2) = wd + vd*(ky*XtilCor(2) + kt*sin(XtilCor(3)));
        
        
        %%% -----Modelo inverso - transforma sinais de velocidades do
        %%% trailer para o Pioneer
        
        v1 = sinal_controle_trailer(1);
        w1 = sinal_controle_trailer(2);
%         v0_c = v1*cos(P.pPot1.Data*pi/180) - (P.pPos.X(12)-w1)*(L1-P.pPar.a)*sin(P.pPot1.Data*pi/180);
%         w0_c = -(v1*sin(P.pPot1.Data*pi/180) + (P.pPos.X(12)-w1)*(L1-P.pPar.a)*cos(P.pPot1.Data*pi/180))/(L0);

%           v0_c = v1*cos(P.pPot1.Data*pi/180) - (P.pPos.X(12)-w1)*(L1)*sin(P.pPot1.Data*pi/180);
%           w0_c = -(v1*sin(P.pPot1.Data*pi/180) + (P.pPos.X(12)-w1)*(L1)*cos(P.pPot1.Data*pi/180))/(L0);

           v0_c = v1*cos(P.pPot1.Data*pi/180) + (w1)*(L1)*sin(P.pPot1.Data*pi/180);
           w0_c = -v1*sin(P.pPot1.Data*pi/180)/L0 + (w1)*(L1)*cos(P.pPot1.Data*pi/180)/L0;
          
%           v0_c = v1*cos(P.pPot1.Data*pi/180) + (w1)*(L1)*sin(P.pPot1.Data*pi/180);
%           w0_c = v1*sin(P.pPot1.Data*pi/180)/L1 - (w1)*(L1)*cos(P.pPot1.Data*pi/180);
        
        P.pSC.Ud(1:2) = [v0_c; w0_c];
        %%%% ---- sinal de controle original
        
        %           P.pSC.Ud(1:2) = [v0_c; w0_c];
        %
        %%%---------
        
        P = J.mControl(P);                    % joystick command (priority)
        %               disp('angle 1')
        %               P.pPot1.Data
        %               disp('angle 2')
        %               P.pPot2.Data
        
        %             B.pSC.Ud
        P.rCommand;
        
        xd_hist = [xd_hist P.pPos.Xd(1:2)];
        x_hist = [x_hist P.pPos.X(1:2)];
        angle1_hist = [angle1_hist P.pPot1.Data];
        angle2_hist = [angle2_hist P.pPot2.Data];
        t_hist = [t_hist toc(t)];
        control_hist = [control_hist P.pSC.Ud(1:2)];
        pose_centro_trailer_hist = [pose_centro_trailer_hist pose_centro_trailer];
        centro_robo_hist = [centro_robo_hist centro_robo];
        

%         plot(xd_hist(1,:),xd_hist(2,:));
%         hold on;
%         grid on;
%         plot(pose_centro_trailer_hist(1,:),pose_centro_trailer_hist(2,:));
%         plot(centro_robo_hist(1,:),centro_robo_hist(2,:));
%         legend('Desejado','Trailer', 'Centro robo');
%         xlabel('X');
%         ylabel('Y');
%         axis([-3 1 -0.5 3.5])
%         hold off;
        
        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        drawnow
        if btnEmergencia ~= 0
            disp('Bebop Landing through Emergency Command ');
            
            % Send 3 times Commands 1 second delay to Drone Land
            
            break;
        end
        
        
    end
end
%catch ME

%     disp('Bebop Landing through Try/Catch Loop Command');
%     P.rCmdStop;
%
% end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

figure();
hold on;
grid on;
plot(t_hist(1,:), control_hist(1,:));
plot(t_hist(1,:), control_hist(2,:));
legend('sinal controle v', 'sinal controle w');
title('sinais controle');
xlabel('t');
ylabel('sinal');

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(pose_centro_trailer_hist(1,:),pose_centro_trailer_hist(2,:));
plot(centro_robo_hist(1,:),centro_robo_hist(2,:));
legend('Desejado','Trailer', 'Centro robo');
xlabel('X');
ylabel('Y');

figure();
hold on;
grid on;
plot(t_hist(1,:), xd_hist(1,:) - pose_centro_trailer_hist(1,:));
plot(t_hist(1,:), xd_hist(2,:) - pose_centro_trailer_hist(2,:));
legend('erro trailer x', 'erro trailer y');
title('erro posições');
xlabel('t');
ylabel('sinal');