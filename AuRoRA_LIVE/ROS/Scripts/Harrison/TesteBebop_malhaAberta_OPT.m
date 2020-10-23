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
    RI.rConnect('192.168.0.129');
    B = Bebop(1,'B');
    
    %P = Pioneer3DX(1);  % Pioneer Instance
    
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
% Beboop
disp('Start Take Off Timming....');
B.rTakeOff;
pause(5);
disp('Taking Off End Time....');


% Time variables initialization
T_CONTROL = 1/5; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 36;

fprintf('\nStart..............\n\n');

t  = tic;
t_control = tic;

xd_hist = [];
x_hist = [];
t_hist = [];
quat_hist = [];

try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
            t_atual = toc(t);
%% POSIÇÃO            
            
            % Dados do OPT
            
            B.rGetSensorData;
            quat = [B.pOdom.Pose.Pose.Orientation.W
            B.pOdom.Pose.Pose.Orientation.X
            B.pOdom.Pose.Pose.Orientation.Y
            B.pOdom.Pose.Pose.Orientation.Z;];

            % Joystick Command Priority
            B.pSC.Ud = [0 0 0 0 0 0.6]';
%             B.pSC.Ud = [0 0 0 0 0 0]';
            B = J.mControl(B);                    % joystick command (priority)
            B.pSC.Ud
            B.rCommand;

            x_hist = [x_hist B.pPos.X];
            t_hist = [t_hist toc(t)];
            quat_hist = [quat_hist quat];
            
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
% 
% figure();
% hold on;
% grid on;
% % plot(xd_hist(1,:),xd_hist(2,:));
% plot(x_hist(1,:),x_hist(2,:));
% title('XY');
% xlabel('X [m]');
% ylabel('Y [m]');
% 
% figure();
% hold on;
% grid on;
% % plot(t_hist,xd_hist(6,:));
% plot(t_hist,x_hist(6,:));
% title('XY');
% xlabel('X [m]');
% ylabel('Y [m]');

save('Harrison_quat.mat','xd_hist','x_hist','t_hist','quat_hist')

filename = ['Harrison_quat_' datestr(now,30) '.mat'];
save(filename,'x_hist','t_hist','quat_hist')
