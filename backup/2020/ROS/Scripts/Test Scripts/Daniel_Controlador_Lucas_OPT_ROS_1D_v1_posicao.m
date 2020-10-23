clear; close all; clc;
try
    fclose(instrfindall);
catch
end
%
% % Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
% addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.102');
    B = Bebop(1,'B');
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
    % Joystick
    J = JoyControl;
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    idB = getID(OPT,B); % ID do Bebop
    

    
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
% Beboop
disp('Start Take Off Timming....');
B.rTakeOff;
pause(5);
disp('Taking Off End Time....');

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 1/30; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 120;

Xd_i = 1;
Xd(:,1) = [0 0 .5 0]';
Xd(:,2) = [.5 .5 1.5 0]';
Xd(:,3) = [-.5 .5 1.5 0]';
Xd(:,4) = [-.5 -.5 1.5 0]';
Xd(:,5) = [.5 -.5 1.5 0]';
Xd(:,6) = [0 0 1.5 0]';


B.pPos.Xd(1:3) = Xd(1:3,Xd_i);
B.pPos.Xd(6) = Xd(4,Xd_i);

T_xd = 4;
t_xd = tic;


fprintf('\nStart..............\n\n');


t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

B.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ]';
B.pPar.Ts = 1/30;


try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            

         
            if toc(t_xd) > T_xd && Xd_i < 6
                t_xd = tic;
                Xd_i = Xd_i + 1;
                B.pPos.Xd(1:3) = Xd(1:3,Xd_i);
                B.pPos.Xd(6) = Xd(4,Xd_i);
            end
            
            % Dados do OPT
            rb = OPT.RigidBody;
            if rb(idB).isTracked
                B = getOptData(rb(idB),B);
            end
            
            B.cInverseDynamicController_Compensador;

            
            %% Save data
            
            % Variable to feed plotResults function
            data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                      t_atual B.pPar.Model_simp'];
            
            % %         %   1 -- 12      13 -- 24     25 -- 29          30 -- 34
            % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
            % %
            % %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60
            % %             B.pPos.Xd'   B.pPos.X'    B.pSC.Ud'         B.pSC.U'
            % %
            % %         %   61 -- 66     67 -- 72       73 -- 78       79
            % %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
            
            
            % Beboop
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)
            B.pSC.Ud
            B.rCommand;
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            drawnow
            if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    B.rCmdStop;
                    B.rLand;
                end
                break;
            end
            
            
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B.rLand
    
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B.rCmdStop;
    B.rLand
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%% Plot results
Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,35),Xtil(:,1));
plot(data(:,35),Xtil(:,2));
plot(data(:,35),Xtil(:,3));
plot(data(:,35),Xtil(:,6));
title('Erro de Posição');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

figure();
subplot(411)
hold on;
grid on;
plot(data(:,35),data(:,7));
plot(data(:,35),data(:,19));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('dX', 'dXd');

subplot(412)
hold on;
grid on;
plot(data(:,35),data(:,8));
plot(data(:,35),data(:,20));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('dY', 'dYd');

subplot(413)
hold on;
grid on;
plot(data(:,35),data(:,9));
plot(data(:,35),data(:,21));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('dZ', 'dZd');

subplot(414)
hold on;
grid on;
plot(data(:,35),data(:,18));
plot(data(:,35),data(:,24));
xlabel('Tempo[s]');
ylabel('Velocidade [rad/s]');
legend('phi', 'dphi');

subplot(421)
grid on;
plot(data(:,35),data(:,36));
xlabel('Tempo[s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,36))],'interpreter','latex')

subplot(422)
grid on;
plot(data(:,35),data(:,37));
xlabel('Tempo[s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,37))],'interpreter','latex')

subplot(423)
grid on;
plot(data(:,35),data(:,38));
xlabel('Tempo[s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,38))],'interpreter','latex')

subplot(424)
grid on;
plot(data(:,35),data(:,39));
xlabel('Tempo[s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,39))],'interpreter','latex')

subplot(425)
grid on;
plot(data(:,35),data(:,40));
xlabel('Tempo[s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,40))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,35),data(:,41));
xlabel('Tempo[s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,41))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,35),data(:,42));
xlabel('Tempo[s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,42))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,35),data(:,43));
xlabel('Tempo[s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,43))],'interpreter','latex')
