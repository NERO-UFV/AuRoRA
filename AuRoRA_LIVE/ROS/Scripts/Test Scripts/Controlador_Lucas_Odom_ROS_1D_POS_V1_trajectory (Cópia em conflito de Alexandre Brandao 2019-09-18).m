%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear; close all; clc;
try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Load Classes

%% Load Class
try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.166');
    B = Bebop(1,'B');
        
    % Joystick
    J = JoyControl;

    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
       
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
pause(3);
disp('Taking Off End Time....');

%% Variable initialization
data = [];
B.rGetSensorDataLocal;

% Time variables initialization
T_CONTROL = 0.2; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 120;

T = 30;
w = 2*pi/T;

fprintf('\nStart..............\n\n');

% Parametros dos Controladores 
model = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ];
%          X     Y     Z    Psi
gains = [  1     1     1     1 ...
           1.5   1.5   3     1 ...
           1     1     1     1 ...
           1     1     1     1];

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

try    
    while toc(t) < T_MAX

        if toc(t_control) > T_CONTROL             

            B.pPar.Ts = toc(t_incB);        
            t_incB = tic;
            t_control = tic;

            % Trajectory Planner        
            Xd = [0 0 1.5 0]';
            dXd = [0 0 0 0]';
            ddXd = [0 0 0 0]';

            % Ardrone
            B.rGetSensorDataLocal;

            B.pPos.Xd(1:3) = Xd(1:3);
            B.pPos.Xd(6) = Xd(4);

            B.pPos.Xd(7:9) = dXd(1:3);
            B.pPos.Xd(12) = dXd(4);

            B.pPos.dXd(7:9) = ddXd(1:3);
            B.pPos.dXd(12) = ddXd(4);

            B.cInverseDynamicController(model,gains);
            
            %disp(strcat("PSI",toStringJSON(B.pPos.X(6))));
            %disp(strcat("PSI_Ponto",toStringJSON(B.pPos.X(12))));
            %disp(strcat("PSI_Ud",toStringJSON(B.pSC.Ud(6))));

            %% Save data

            % Variable to feed plotResults function
             data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                toc(t)];

            % Zerando sinais de controle
            %B.pSC.Ud = [0 0 0 0 0 0]';
            %B.pSC.Ud(6) = 0;

            % Beboop
            % Joystick Command Priority
            B = J.mControl(B);                    % joystick command (priority)   
            B.rCommand;

            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');
                B.rCmdStop;
                B.rLand;
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

% figure();
% subplot(311)
% hold on;
% grid on;
% plot(data(:,35),data(:,7));
% plot(data(:,35),data(:,19));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dX', 'dXd');
% 
% subplot(312)
% hold on;
% grid on;
% plot(data(:,35),data(:,8));
% plot(data(:,35),data(:,20));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dY', 'dYd');
% 
% subplot(313)
% hold on;
% grid on;
% plot(data(:,35),data(:,9));
% plot(data(:,35),data(:,21));
% xlabel('Tempo[s]');
% ylabel('Velocidade [m/s]');
% legend('dZ', 'dZd');

% figure
% hold on
% plot(tempo,poses(:,1))
% plot(tempo,poses(:,2))
% title('Posicoes')
% legend('Pos X','Pos Y','Pos Z', 'Ori Z');
% xlabel('Tempo(s)');
% ylabel('Pos');
% hold off

