%% 3D Line Formation Pioneer-Drone
% Pioneer is the reference of the formation
% The formation variables are:
% Q = [ xf yf zf rho alfa beta ]
% Initial Comands

clear; close all; clc;
% try
%     fclose(instrfindall);
% catch
% end
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
    RI.rConnect('192.168.0.144');
    B{1} = Bebop(1,'B1');
    B{2} = Bebop(2,'B2');
    
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
B{1}.rTakeOff;
B{2}.rTakeOff;
% pause(3);
disp('Taking Off End Time....');

%% Variable initialization
data = [];

% Time variables initialization
T_CONTROL = 0.2; % 200 ms de Amostragem | 5 Hz de Frequência
T_MAX = 40;

% T = 20;
% w = 2*pi/T;
% dXd_max = diag([w w 0.0 0.0]);

% dXd = [0 0 0 0];
% xp = [0 0 0 0];

rX = .5;           % [m]
rY = 1;           % [m]
T = 20;             % [s]
Tf = T*2;            % [s]

w = 2*pi/T;         % [rad/s] frequência Drone 1

Xd = [0 0 0 0]';
dXd = [0 0 0 0]';
ddXd = [0 0 0 0]';


fprintf('\nStart..............\n\n');

pause(5);

t_Formation = tic;      % Formation cycle
t_integF = tic;
t_incB = tic;
t  = tic;
t_control = tic;
B{1}.pPar.ti = tic;
B{2}.pPar.ti = tic;


try
    while toc(t) < T_MAX
        
        if toc(t_control) > T_CONTROL
            
            t_control = tic;
%             t_traj = toc(t);
%             a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
%             tp = a*Tf;
            
            % Trajectory Planner
            %             Xd = [cos(w*toc(t)) sin(w*toc(t)) 1.5 0]';
            %             dXd = [-w*sin(w*toc(t)) w*cos(w*toc(t)) 0 0]';
            %             ddXd = [-w^2*cos(w*toc(t)) -w^2*sin(w*toc(t)) 0 0]';
            
            %             Xd = [0 0 1 0]';
            %             dXd = [0 0 0 0]';
            %             ddXd = [0 0 0 0]';
            %
            
            %% Trajetória 1
            Xd = [rX*sin(w*toc(t));
                rY*cos(0.5*w*toc(t));
                1.25 + 0.5*sin(w*toc(t));
                pi/6*sin(w*toc(t))];
            
            
            xdp_old = dXd;
            
            dXd = [w*rX*cos(w*toc(t));
                -0.5*w*rY*sin(0.5*w*toc(t));
                w*0.5*cos(w*toc(t));
                w*pi/6*cos(w*toc(t))];
            
            ddXd = [ (dXd(1) - xdp_old(1))/toc(t_incB) ...
                (dXd(2) - xdp_old(2))/toc(t_incB) ...
                -w^2*0.5*sin(w*toc(t)) ...
                0]';

            
% %             Xd = [ rX*sin(w*tp) ...
% %                 rY*sin(2*w*tp) ...
% %                 1.25 + 0.5*sin(w*tp) ...
% %                 0]';
% %             
% %             xdp_old = dXd;
% %             dXd = [  w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rX*cos(w*tp) ...
% %                 2*w*6*((t_traj/Tf)-(t_traj/Tf)^2)*rY*cos(2*w*tp)...
% %                 w*0.5*cos(w*tp) ...
% %                 0]';
% %             
% %             ddXd = [ (dXd(1) - xdp_old(1))/toc(t_incB) ...
% %                 (dXd(2) - xdp_old(2))/toc(t_incB) ...
% %                 -w^2*0.5*sin(w*tp) ...
% %                 0]';
            
            
            % Ardrone
            B{1}.rGetSensorDataOpt;
            B{2}.rGetSensorDataOpt;

            
            % Encontrando velocidade angular
            B{1}.pPos.X(12) = (B{1}.pPos.X(6) - B{1}.pPos.Xa(6))/toc(t_incB);
            B{2}.pPos.X(12) = (B{2}.pPos.X(6) - B{2}.pPos.Xa(6))/toc(t_incB);
            t_incB = tic;
            
            % Bebop 1
            B{1}.pPos.Xd(1:3) = Xd(1:3) - [.5 .5 0]';
            B{1}.pPos.Xd(6) = Xd(4);
            
            B{1}.pPos.Xd(7:9) = dXd(1:3);
            B{1}.pPos.Xd(12) = dXd(4);
            
            B{1}.pPos.dXd(7:9) = ddXd(1:3);
            B{1}.pPos.dXd(12) = ddXd(4);
            
            %Bebop 2
            B{2}.pPos.Xd(1:3) = Xd(1:3) + [.5 .5 0]';
            B{2}.pPos.Xd(6) = Xd(4);
            
            B{2}.pPos.Xd(7:9) = dXd(1:3);
            B{2}.pPos.Xd(12) = dXd(4);
            
            B{2}.pPos.dXd(7:9) = ddXd(1:3);
            B{2}.pPos.dXd(12) = ddXd(4);
            
            
        %% Control
        
            B{1}.cInverseDynamicController;
            B{2}.cInverseDynamicController;
%             B.cInverseDynamicController;
            B{1}.pPar.ti = tic;
            B{2}.pPar.ti = tic;
            
            %% Save data
            
%             Variable to feed plotResults function
            data = [  data  ; B{1}.pPos.Xd'     B{1}.pPos.X'        B{2}.pPos.Xd'     B{2}.pPos.X' ...
                toc(t)];
            
            %         %   1 -- 12         13 -- 24     25 -- 36       37 -- 48
            %          B{1}.pPos.Xd'    B{1}.pPos.X'  B{2}.pPos.Xd'  B{2}.pPos.X'
            %
            %         %   29 -- 40     41 -- 52     53 -- 56          57 -- 60
            %             B.pPos.Xd'   B.pPos.X'    B.pSC.Ud'         B.pSC.U'
            %
            %         %   61 -- 66     67 -- 72       73 -- 78       79
            %             LF.pPos.Qd'  LF.pPos.Qtil'  LF.pPos.Xd'    toc(t)  ];
            
            
            % Beboop
            % Joystick Command Priority
            B{1} = J.mControl(B{1});                    % joystick command (priority)
            B{1}.rCommand;
            B{2} = J.mControl(B{2});                    % joystick command (priority)
            B{2}.rCommand;
            
            
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B{1}.pFlag.EmergencyStop ~= 0 || B{1}.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    B{1}.rCmdStop;
                    B{1}.rLand;
                end
                break;
            end
            
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B{2}.pFlag.EmergencyStop ~= 0 || B{2}.pFlag.isTracked ~= 1
                disp('Bebop Landing through Emergency Command ');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    disp("End Land Command");
                    B{2}.rCmdStop;
                    B{2}.rLand;
                end
                break;
            end
            
        end
    end
catch ME
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{1}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B{1}.rLand
    
    disp('Bebop Landing through Try/Catch Loop Command');
    B{2}.rCmdStop;
    disp('');
    disp(ME);
    disp('');
    B{2}.rLand
    
end

% Send 3 times Commands 1 second delay to Drone Land
for i=1:nLandMsg
    disp("End Land Command");
    B{1}.rCmdStop;
    B{1}.rLand
    
    disp("End Land Command");
    B{2}.rCmdStop;
    B{2}.rLand
end


% Close ROS Interface
RI.rDisconnect;
rosshutdown;

disp("Ros Shutdown completed...");

%% Plot results
B{1}.pPos.Xtil = data(:,1:12) - data(:,13:24);
B{2}.pPos.Xtil = data(:,25:36) - data(:,37:48);

subplot(211);
hold on;
grid on;
plot(data(:,49),B{1}.pPos.Xtil(:,1));
plot(data(:,49),B{1}.pPos.Xtil(:,2));
plot(data(:,49),B{1}.pPos.Xtil(:,3));
plot(data(:,49),B{1}.pPos.Xtil(:,6));
title('Erro de Posição Q1');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

subplot(212);
hold on;
grid on;
plot(data(:,49),B{2}.pPos.Xtil(:,1));
plot(data(:,49),B{2}.pPos.Xtil(:,2));
plot(data(:,49),B{2}.pPos.Xtil(:,3));
plot(data(:,49),B{2}.pPos.Xtil(:,6));
title('Erro de Posição Q2');
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Tempo[s]');
ylabel('Erro [m]');

% figure();
% hold on;
% grid on;
% plot(data(:,1),data(:,2));
% plot(data(:,13),data(:,14));
% title('XY');
% xlabel('X [m]');
% ylabel('Y [m]');

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
% 
% title('Posicoes')
% legend('Pos X','Pos Y','Pos Z', 'Ori Z');
% xlabel('Tempo(s)');
% ylabel('Pos');
% hold off
