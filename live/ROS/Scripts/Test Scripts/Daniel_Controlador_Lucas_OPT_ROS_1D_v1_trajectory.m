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
    setenv('ROS_IP','192.168.0.103')
    RI.rConnect('192.168.0.105');
    B = Bebop(1,'B1');
    
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
pause(1);
disp('Taking Off End Time....');

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 1/30; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 120;

rX = 1.5;           % [m]
rY = 1;           % [m]
T = 40;             % [s]
Tf = T*2;            % [s]
w = 2*pi/T;         % [rad/s]

Xd = [0 0 0 0];
dXd = [0 0 0 0];

fprintf('\nStart..............\n\n');

t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

B.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295  ]';
B.pPar.Ts = 1/30;


try
    while toc(t) < T_MAX
              
        if toc(t_control) > T_CONTROL

            t_control = tic;
            t_atual = toc(t);

                Xd = [rX*sin(w*t_atual);
                    rY*cos(w*t_atual);
                    1.5 - 0.5*cos(2*w*t_atual);
                    0];
                
                xdp_old = dXd;
                
                dXd = [w*rX*cos(w*t_atual);
                    -w*rY*sin(w*t_atual);
                    .5*w*2*sin(2*w*t_atual);
                    0];
%                 Xd = [0;0;1;0];
%                 dXd = [0;0;0;0];
                          
            % Bebop
%             B.rGetSensorDataOpt;
            rb = OPT.RigidBody;
            if rb(idB).isTracked
                B = getOptData(rb(idB),B);
%                 B.pPos.X
            end
            
            % Encontrando velocidade angular
            
            B.pPos.Xd(1:3) = Xd(1:3);
            B.pPos.Xd(6) = Xd(4);
            
            B.pPos.Xd(7:9) = dXd(1:3);
            B.pPos.Xd(12) = dXd(4);
            
%            if size(data,1) > 90 && B.pSC.Control_flag == 0
%                 B.pSC.Control_flag = 1;
%            end            
           
            B.cInverseDynamicController_Compensador;
            

            
            %% Save data
            
            % Variable to feed plotResults function
            data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                      t_atual B.pPar.Model_simp'];
            
            % %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28
            % %             P.pPos.Xd'   P.pPos.X'    P.pSC.Ud(1:2)'    P.pSC.U(1:2)'
            % %
            % %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60
            % %             B.pPos.Xd'   B.pPos.X'    B.pSC.Ud'         B.pSC.U'
            % %
            % %         %   61 -- 66     67 -- 72       73 -- 78       79
            % %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    t_atual  ];
            
            
            % Beboop
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)
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
axis([0 60 -0.1 .1])
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
plot(data(:,35),data(:,19));
plot(data(:,35),data(:,7));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('dX', 'dXd');

subplot(412)
hold on;
grid on;
plot(data(:,35),data(:,20));
plot(data(:,35),data(:,8));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('dY', 'dYd');

subplot(413)
hold on;
grid on;
plot(data(:,35),data(:,21));
plot(data(:,35),data(:,9));
xlabel('Tempo[s]');
ylabel('Velocidade [m/s]');
legend('dZ', 'dZd');

subplot(414)
hold on;
grid on;
plot(data(:,35),data(:,24));
plot(data(:,35),data(:,12));
xlabel('Tempo[s]');
ylabel('Velocidade [rad/s]');
legend('phi', 'dphi');


figure();
subplot(311)
hold on;
grid on;
plot(data(:,35),data(:,13));
plot(data(:,35),data(:,1));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,35),data(:,14));
plot(data(:,35),data(:,2));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,35),data(:,15));
plot(data(:,35),data(:,3));
xlabel('Tempo[s]');
ylabel('Erro posição [m]');
legend('Z', 'Zd');

figure();
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
