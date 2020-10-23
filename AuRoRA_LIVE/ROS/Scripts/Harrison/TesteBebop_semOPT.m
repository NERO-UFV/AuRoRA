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

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 1/30; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 20;

T_xd = 4;
t_xd = tic;

T = T_MAX;
w = 2*pi/T;
r = 0.5;


fprintf('\nStart..............\n\n');


t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;

B.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ]';
B.pPar.Ts = 1/30;

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
            
            B.rGetSensorDataLocal;
            quat = [B.pOdom.Pose.Pose.Orientation.W
            B.pOdom.Pose.Pose.Orientation.X
            B.pOdom.Pose.Pose.Orientation.Y
            B.pOdom.Pose.Pose.Orientation.Z;];
%             
%             B.pPos.Xd(1:3) = [r*cos(w*toc(t)); r*sin(w*toc(t)); 1];
%             B.pPos.Xd(7:9) = [-r*w*sin(w*toc(t)); r*w*cos(w*toc(t)); 0];
            B.pPos.Xd(1:3) = [0.1*toc(t); 0; 1];
            B.pPos.Xd(7:9) = [0.1;0; 0];
            B.pPos.Xd(6) = w*toc(t);
            B.pPos.Xd(12) = w;
            
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
%             B.pSC.Ud = [0 0 0 0 0 0]';
            B = J.mControl(B);                    % joystick command (priority)
%             B.pPos.X
            B.rCommand;
            xd_hist = [xd_hist B.pPos.Xd];
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

figure();
hold on;
grid on;
plot(xd_hist(1,:),xd_hist(2,:));
plot(x_hist(1,:),x_hist(2,:));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

figure();
hold on;
grid on;
plot(t_hist,xd_hist(6,:));
plot(t_hist,x_hist(6,:));
title('XY');
xlabel('X [m]');
ylabel('Y [m]');

save('Harrison_quat.mat','xd_hist','x_hist','t_hist','quat_hist')