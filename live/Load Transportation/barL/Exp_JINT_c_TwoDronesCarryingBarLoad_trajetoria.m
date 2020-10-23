
clear; close all; clc;
try
    fclose(instrfindall);
catch
end


try
    rosshutdown;
end

%
% % Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

%% Load Class
% Load Classes
A = ArDrone(1);
A.pPar.Ts = 1/30;
P.pPar.Ts = 1/30;
A.pPar.Model_simp = [ 14.72 0.2766 6.233 0.53 2.6504 2.576 .3788 1.5216 ]';
pgains = .3*[1.5 1 1.5 1];


% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Initiate classes
P = Pioneer3DX(1);
A = ArDrone(1);
idP = getID(OPT,P); % ID do Bebop
idA = getID(OPT,ArDrone);


% Joystick
J = JoyControl;

% % Network
% r = ROSNetwork;
% r.InitROS('/master')
% r.InitROS('/robot1','192.168.0.158')
% P.pSC.Ud = [0 0]';
% r.PublisherROS(r.node,'robot1/vel');
setenv('ROS_IP','192.168.0.158')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.148:11311/')  %Ip do master (raspberry)
rosinit        
[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist');

cmd_vel.Linear.X = 0;
cmd_vel.Angular.Z = 0.0;
send(pub,cmd_vel);

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

pause(3)
%% Formation Variables Initialization
barL.pPos.X = zeros(6,1);
barL.pPos.Xd = zeros(6,1);
barL.pPos.Xr = zeros(6,1);
barL.pPos.dXr = zeros(6,1);
barL.pPos.dXd = zeros(6,1);
barL.pPos.Qd = zeros(6,1);
barL.pPos.dQd = zeros(6,1);
barL.pPos.Q = zeros(6,1);
barL.pPos.Qtil = zeros(6,1);
barL.pPos.alpha = 0;
barL.pPos.beta = 0;
barL.pPos.rho = 0;


%% Parâmetros da Carga barL
% Comprimento dos cabos
barL.pPar.l1 = 1;
barL.pPar.l2 = 1;

% Comprimento da barL
barL.pPar.L = 1.5;

barL.pPar.K1 = 1*diag([1 1 1 1 1 3]);    % kinematic control gain  - controls amplitude
barL.pPar.K2 = 1*diag([1 1 1 1 1 1]);        % kinematic control gain - control saturation

% ArDrone
A.pPar.ip = '192.168.1.30';
A.rConnect;
A.rTakeOff;

pause(5);
disp('Taking Off End Time....');


% timers
T_exp = 120; % tempo de experimento
T_run = 1/30; % período de amostragem do experimento
t_run = tic;
t_total = tic; % tempo que o experimento está rodando (running)
t_plot = tic; %
T_plot = .3; % período de plotagem
t_task = tic;
T_task = 13;
i_task = 0;
t_exp = tic;
t  = tic;
A.pSC.Kinematics_control = 1;

rX = 1;           % [m]
rY = 1;           % [m]
T = 20;             % [s]
w = 2*pi/T;         % [rad/s]


while toc(t) < T_exp
    
    if toc(t_run) > T_run
        
        t_run = tic;
        t_atual = toc(t);
        %% LEITURA: Obtendo dados dos sensores
        
        %OPTITRACK
        rb = OPT.RigidBody;
        if rb(idP).isTracked
            P = getOptData(rb(idP),P);
        end
        %
        if rb(idA).isTracked
            A = getOptData(rb(idA),A);
%             A.pPos.X
        end
        
        
        %% TAREFA: Posição desejada nas variáveis generalizadas Q = (xc,yc,zc,alpha,gamma,L)
        % Trajetória do Pioneer
                        Xd = [rX*sin(w*t_atual);
                            rY*cos(0.5*w*t_atual);
                            0;
                            0];
        
        
                        dXd = [w*rX*cos(w*t_atual);
                            -0.5*w*rY*sin(0.5*w*t_atual);
                            0;
                            0];
        
%         % Posição do Pioneer
%         Xd = [0;
%             0;
%             0;
%             0];
%         
%         
%         dXd = [0;
%             0;
%             0;
%             0];
        
        barL.pPos.Qd = [Xd(1) Xd(2) Xd(3) deg2rad(0) deg2rad(90) barL.pPar.L]';
        barL.pPos.dQd = [dXd(1) dXd(2) dXd(3) 0 0 0]';
        
        %% FORMAÇÃO
        barL.pPos.alpha = atan2(A.pPos.X(2) - P.pPos.X(2),A.pPos.X(1) - P.pPos.X(1));
        barL.pPos.beta = atan2(A.pPos.X(3) - P.pPos.X(3),norm(A.pPos.X(1:2) - P.pPos.X(1:2)));
        barL.pPos.rho = norm(A.pPos.X(1:3) - P.pPos.X(1:3));
        
        barL.pPos.Q = [P.pPos.X(1:3); barL.pPos.alpha; barL.pPos.beta; barL.pPos.rho];
        barL.pPos.Qtil = barL.pPos.Qd - barL.pPos.Q;
        
        %% CONTROLE CINEMÁTICO DA FORMAÇÃO (VERIFICAR SE pSC.Kinematics_control = 1)
        barL.pPos.Qr = barL.pPos.dQd + barL.pPar.K1*tanh(barL.pPar.K2*barL.pPos.Qtil);
        
        J_inv = [1 0 0           0                       0                          0          ;...
                 0 1 0           0                       0                          0          ;...
                 0 0 1           0                       0                          0          ;...
                 1 0 0 -barL.pPos.rho*sin(barL.pPos.alpha)*cos(barL.pPos.beta) -barL.pPos.rho*cos(barL.pPos.alpha)*sin(barL.pPos.beta) cos(barL.pPos.alpha)*cos(barL.pPos.beta);...
                 0 1 0 barL.pPos.rho*cos(barL.pPos.alpha)*cos(barL.pPos.beta)  -barL.pPos.rho*sin(barL.pPos.alpha)*sin(barL.pPos.beta) sin(barL.pPos.alpha)*cos(barL.pPos.beta);...
                 0 0 1                             0                                            barL.pPos.rho*cos(barL.pPos.beta)                 sin(barL.pPos.beta)           ];
        
        
        barL.pPos.Xr = J_inv*barL.pPos.Qr;
        
        
        P.pPos.Xd([1 2 7 8]) = [barL.pPos.Qd(1:2) barL.pPos.Xr(1:2)];
        A.pPos.Xr([7 8 9]) = barL.pPos.Xr(4:6);
        
        A.pPos.Xd(6) = 0;   %yaw desejado
        A.pPos.Xd(12) = 0;  %vyaw desejado
        
        
        
        %% CONTROLE DINÂMICO DOS ROBÔS
        A = cInverseDynamicController_Compensador_ArDrone(A);
        P = fControladorCinematico(P,pgains);
        

        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel);

        
        %         B = J.mControl(B);                    % joystick command (priority)
        %         B.rCommand;
        A = J.mControl(A);                       % joystick command (priority)
        A.rSendControlSignals;
        
        %% DATA
        
        % Variable to feed plotResults function
        data = [  data  ; P.pPos.Xd(1:3)'     P.pPos.X(1:3)' ...
            A.pPos.Xd(1:3)'     A.pPos.X(1:3)' ...
            toc(t)];
        
        % %         %   1 -- 3      4 -- 6
        % %         B{1}.pPos.Xd'  B{1}.pPos.X'
        % %
        % %         %   7 -- 9     10 -- 12        13 -- 18
        % %         B{2}.pPos.Xd'  B{2}.pPos.X' barL.pPos.X_load'
        % %
        % %         %  19 -- 26             27 -- 34          35
        % %      B{1}.pPar.Model_simp  B{2}.pPar.Model_simp  toc(t) ];
        
        
    end
end


%% Plot results
B1_Xtil = data(:,1:3) - data(:,4:6);
B2_Xtil = data(:,7:9) - data(:,10:12);
Load_Xtil = [data(:,35:37)-data(:,13:15) data(:,38)-data(:,18) data(:,39)-data(:,16)];


%
figure();
hold on;
grid on;
plot(data(:,end),B1_Xtil(:,1));
plot(data(:,end),B1_Xtil(:,2));
plot(data(:,end),B1_Xtil(:,3));
title('Q1 position error');
legend('$\tilde{x}$','$\tilde{y}$','$\tilde{z}$','interpreter','latex');
xlabel('Time [s]');
ylabel('Error [m]');

figure();
hold on;
grid on;
plot(data(:,end),B2_Xtil(:,1));
plot(data(:,end),B2_Xtil(:,2));
plot(data(:,end),B2_Xtil(:,3));
title('Q2 position error');
legend('$\tilde{x}$','$\tilde{y}$','$\tilde{z}$','interpreter','latex');
xlabel('Time [s]');
ylabel('Error [m]');

figure();
hold on;
grid on;
plot(data(:,end),Load_Xtil(:,1));
plot(data(:,end),Load_Xtil(:,2));
plot(data(:,end),Load_Xtil(:,3));
title('Load Position error');
legend('$\tilde{x}$','$\tilde{y}$','$\tilde{z}$','interpreter','latex');
xlabel('Time [s]');
ylabel('Error [m]');

figure();
hold on;
grid on;
plot(data(:,end),Load_Xtil(:,4));
plot(data(:,end),Load_Xtil(:,5));
title('Load orientation error');
legend('$\alpha$','$\beta$','interpreter','latex');
xlabel('Time [s]');
ylabel('Error [°]');


% %
% % figure();
% % hold on;
% % grid on;
% % plot(data(:,1),data(:,2));
% % plot(data(:,13),data(:,14));
% % title('XY');
% % xlabel('X [m]');
% % ylabel('Y [m]');
% %
% % figure();
% % subplot(411)
% % hold on;
% % grid on;
% % plot(data(:,end),data(:,19));
% % plot(data(:,end),data(:,7));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dX', 'dXd');
% %
% % subplot(412)
% % hold on;
% % grid on;
% % plot(data(:,end),data(:,20));
% % plot(data(:,end),data(:,8));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dY', 'dYd');
% %
% % subplot(413)
% % hold on;
% % grid on;
% % plot(data(:,end),data(:,21));
% % plot(data(:,end),data(:,9));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [m/s]');
% % legend('dZ', 'dZd');
% %
% % subplot(414)
% % hold on;
% % grid on;
% % plot(data(:,end),data(:,24));
% % plot(data(:,end),data(:,12));
% % xlabel('Tempo[s]');
% % ylabel('Velocidade [rad/s]');
% % legend('phi', 'dphi');
% %
% %
figure();
sgtitle('Q1')
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,4));
plot(data(:,end),data(:,1));
legend('$x$','$x_d$','interpreter','latex');
xlabel('Time [s]');
ylabel('Position [m]');


subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,5));
plot(data(:,end),data(:,2));
legend('$y$','$y_d$','interpreter','latex');
xlabel('Time [s]');
ylabel('Position [m]');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,6));
plot(data(:,end),data(:,3));
legend('$z$','$z_d$','interpreter','latex');
xlabel('Time [s]');
ylabel('Position [m]');


figure();
sgtitle('Q2')
subplot(311)
hold on;
grid on;
plot(data(:,end),data(:,10));
plot(data(:,end),data(:,7));
legend('$x$','$x_d$','interpreter','latex');
xlabel('Time [s]');
ylabel('Position [m]');

subplot(312)
hold on;
grid on;
plot(data(:,end),data(:,11));
plot(data(:,end),data(:,8));
legend('$y$','$y_d$','interpreter','latex');
xlabel('Time [s]');
ylabel('Position [m]');

subplot(313)
hold on;
grid on;
plot(data(:,end),data(:,12));
plot(data(:,end),data(:,9));
legend('$z$','$z_d$','interpreter','latex');
xlabel('Time [s]');
ylabel('Position [m]');


figure();
subplot(421)
grid on;
plot(data(:,end),data(:,19));
xlabel('Time [s]');
ylabel('$K_1$','interpreter','latex');
legend(['$K_1 = $' num2str(data(end,19))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,end),data(:,20));
xlabel('Time [s]');
ylabel('$K_2$','interpreter','latex');
legend(['$K_2 = $' num2str(data(end,20))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,end),data(:,21));
xlabel('Time [s]');
ylabel('$K_3$','interpreter','latex');
legend(['$K_3 = $' num2str(data(end,21))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,end),data(:,22));
xlabel('Time [s]');
ylabel('$K_4$','interpreter','latex');
legend(['$K_4 = $' num2str(data(end,22))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,end),data(:,23));
xlabel('Time [s]');
ylabel('$K_5$','interpreter','latex');
legend(['$K_5 = $' num2str(data(end,23))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,end),data(:,24));
xlabel('Time [s]');
ylabel('$K_6$','interpreter','latex');
legend(['$K_6 = $' num2str(data(end,24))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,end),data(:,25));
xlabel('Time [s]');
ylabel('$K_7$','interpreter','latex');
legend(['$K_7 = $' num2str(data(end,25))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,end),data(:,26));
xlabel('Time [s]');
ylabel('$K_8$','interpreter','latex');
legend(['$K_8 = $' num2str(data(end,26))],'interpreter','latex')

figure();
subplot(421)
grid on;
plot(data(:,end),data(:,27));
xlabel('Time [s]');
ylabel('$K_1$','interpreter','latex');
legend(['$K_1 = $' num2str(data(end,27))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,end),data(:,28));
xlabel('Time [s]');
ylabel('$K_2$','interpreter','latex');
legend(['$K_2 = $' num2str(data(end,28))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,end),data(:,29));
xlabel('Time [s]');
ylabel('$K_3$','interpreter','latex');
legend(['$K_3 = $' num2str(data(end,29))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,end),data(:,30));
xlabel('Time [s]');
ylabel('$K_4$','interpreter','latex');
legend(['$K_4 = $' num2str(data(end,30))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,end),data(:,31));
xlabel('Time [s]');
ylabel('$K_5$','interpreter','latex');
legend(['$K_5 = $' num2str(data(end,31))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,end),data(:,32));
xlabel('Time [s]');
ylabel('$K_6$','interpreter','latex');
legend(['$K_6 = $' num2str(data(end,32))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,end),data(:,33));
xlabel('Time [s]');
ylabel('$K_7$','interpreter','latex');
legend(['$K_7 = $' num2str(data(end,33))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,end),data(:,34));
xlabel('Time [s]');
ylabel('$K_8$','interpreter','latex');
legend(['$K_8 = $' num2str(data(end,34))],'interpreter','latex')




