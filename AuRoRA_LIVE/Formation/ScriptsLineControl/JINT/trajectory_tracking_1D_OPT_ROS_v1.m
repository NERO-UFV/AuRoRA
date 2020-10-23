%% Trajectory Controller for Parrot Bebop 2

% Valentim Ernandes Neto
% Date: jun-20-2019

% Work developed to Journal of Intelligent & Robotic Systems (JINT), with 
% a special section on Unmanned Systems. 

% ISSN: 0921-0296 (Print) 1573-0409 (Online)
% Publisher: Springer Netherlands
% Impact Factor: 2.020

% ----------------------------------------------------------------------- %

% Initial Comands
clear; 
close all; 
clc;

try
    fclose(instrfindall);
catch
end

% Look for root folder
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
addpath(genpath(pwd));

rmpath('Experimento_Pioneer_Laser_Opt'); % There is a conflict with the 
                                         % files in this folder, you need 
                                         % to remove it.

%% Load Class
try
    
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.144');
    B = Bebop(2,'B2'); 
    
    % Joystick
    J = JoyControl;
    
    % Create OptiTrack object and initialize
    OPT = OptiTrack;
    OPT.Initialize;
    
    fprintf('\n\n------------------------   Load Class Success   ------------------------\n');
    
catch EM
    
    fprintf('\n\n-------------------------   Load Class Issue   -------------------------\n');
    disp(EM);
    
    RI.rDisconnect;
    rosshutdown;
    return;
    
end

%% Emergency settings

% Emmergency button
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'Emergency!', ...
    'Callback', 'btnEmergencia = 1;', ...
    'Position', [50 50 400 300]);

nLandMsg = 3; % Number of redundant landing messages.

%% Variable initialization

% Bebop estimated model
B.pPar.Model_simp = [  0.8417   ,  0.18227  ,    0.8354   ,  0.17095  ...
                       3.9660   ,  4.00100  ,    9.8524   ,  4.7295   ]';

% Take off Bebop
fprintf('\n\n (1) Start take off timming....');
B.rTakeOff;
pause(5);
fprintf('\n\n (2) Take off ends....');

% Time variables 
Tcontrol = 0.030; % 200 ms de Amostragem | 5 Hz de Frequência
Tsim = 50; % Simulation time

% Data variables
data = zeros(round(Tsim/Tcontrol),100); % Data matrix
data = [];

% Trajectory variables

%       (1) 'Lemniscate - Horizontal'
%       (2) 'Lemniscate - Horizontal with double parameterization in time'
%       (3) 'Lemniscate - Inclined'
%       (4) 'Lemniscate - Vertical ( XZ plan | Y = cte )'
%       (5) 'Lemniscate - Vertical ( YZ plan | X = cte )'

type = 'Lemniscate - Vertical ( XZ plan | Y = cte )';
rX = 1;    % X-axis radius [m]
rY = 1;    % Y-axis radius [m]
rZ = 1.5;   % Z-axis radius [m]
T = 15;     % Period [s]
TF = T*2;   % Final time for two turns [s]
w = 2*pi/T; % Angular frequency[rad/s]

fprintf('\n\n (3) Starting timers....');

pause(5);

% Timers start
t  = tic;               % Simulation loop timer
t_control = tic;        % General control loop timer
B.pPar.ti = tic;        % Bebop control loop timer
dt_psi = tic;

fprintf('\n\n (4) Simulation running....\n');
fprintf('\n         ...  ...  ...  ...  ...  ...  ...  ...  ...\n\n');

try
    while toc(t) < Tsim
        
        if toc(t_control) > Tcontrol
            
            TrajConst = [  rX   rY   rZ  ...
                           T    TF   w   ];
            
            TimeVars = [  toc(t_control)    toc(t) ];
                       
            t_control = tic; % Reset general control loop timer
            
            % Trajectory Planner
            [ Xd , dXd , ddXd ] = funcTrajectoryPlanner(TimeVars,TrajConst,type);
              
            % Getting Bebop position data from Optitrack
            B.rGetSensorDataOpt;
            
            % Updating robot reference variables.
            B.pPos.Xd(1:3) = Xd(1:3);
            B.pPos.Xd(6) = Xd(4);
            
            B.pPos.Xd(7:9) = dXd(1:3);
            B.pPos.Xd(12) = dXd(4);
            
            B.pPos.dXd(7:9) = ddXd(1:3);
            B.pPos.dXd(12) = ddXd(4);
            
            % Using the robot trajectory controller.
%             B.cInverseDynamicController_Compensador;
            B.cInverseDynamicController_Adaptativo;
            B.pPar.ti = tic;
            
            %% Save data
            
            % Variable to feed plotResults function
            data = [  data  ; B.pPos.Xd'     B.pPos.X'        B.pSC.Ud'         B.pSC.U' ...
                      toc(t) B.pPar.Model_simp'];
            
            % %         %   1 -- 12      13 -- 24     25 -- 26          27 -- 28
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
            B.rCommand;
            
            % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
            if btnEmergencia ~= 0 || B.pFlag.EmergencyStop ~= 0 || B.pFlag.isTracked ~= 1
                fprintf('\n----------------------------   Emergency   -----------------------------\n');
                fprintf('\n   ----> Bebop Landing through Emergency Command\n');

                % Send 3 times Commands 1 second delay to Drone Land
                for i=1:nLandMsg
                    fprintf(['\n   ----> Land Command (' num2str(i) ')\n']);
                    B.rCmdStop;
                    B.rLand;
                end
                break;
            end
            
            
        end
    end
catch EM
    
    fprintf('\n----------------------------   Code Error   ----------------------------\n');
    fprintf('\n   ----> Bebop Landing through Try/Catch Loop Command\n');
    B.rCmdStop;
    disp('');
    disp(EM);
    disp('');
    B.rLand
    
end

% Send 3 times Commands 1 second delay to Drone Land
fprintf('\n-----------------------------   The End   ------------------------------\n');
for i=1:nLandMsg
    
    fprintf(['\n   ----> Land Command (' num2str(i) ')\n']);
    B.rCmdStop;
    B.rLand
    
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

fprintf('\n   ----> Shutdown completed...\n\n');

%% Plot results
Xtil = data(:,1:12) - data(:,13:24);

figure();
hold on;
grid on;
plot(data(:,35),Xtil(:,1));
plot(data(:,35),Xtil(:,2));
plot(data(:,35),Xtil(:,3));
plot(data(:,35),Xtil(:,6));
legend('Pos X','Pos Y','Pos Z', 'Ori Z');
xlabel('Time [s]');
ylabel('Pose error [m] | [rad]');

figure();
hold on;
grid on;
plot(data(:,1),data(:,2));
plot(data(:,13),data(:,14));
xlabel('X-axis [m]');
ylabel('Y-axis [m]');

figure();
subplot(411)
hold on;
grid on;
plot(data(:,35),data(:,19));
plot(data(:,35),data(:,7));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dX', 'dXd');

subplot(412)
hold on;
grid on;
plot(data(:,35),data(:,20));
plot(data(:,35),data(:,8));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dY', 'dYd');

subplot(413)
hold on;
grid on;
plot(data(:,35),data(:,21));
plot(data(:,35),data(:,9));
xlabel('Time [s]');
ylabel('Velocity [m/s]');
legend('dZ', 'dZd');

subplot(414)
hold on;
grid on;
plot(data(:,35),data(:,24));
plot(data(:,35),data(:,12));
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
legend('dphi', 'dPhid');


figure();
subplot(311)
hold on;
grid on;
plot(data(:,35),data(:,13));
plot(data(:,35),data(:,1));
xlabel('Time [s]');
ylabel('Xtilde [m]');
legend('X', 'Xd');

subplot(312)
hold on;
grid on;
plot(data(:,35),data(:,14));
plot(data(:,35),data(:,2));
xlabel('Time [s]');
ylabel('Ytilde [m]');
legend('Y', 'Yd');

subplot(313)
hold on;
grid on;
plot(data(:,35),data(:,15));
plot(data(:,35),data(:,3));
axis([0,70,1,2])
xlabel('Time [s]');
ylabel('Ztilde [m]');
legend('Z', 'Zd');

subplot(421)
hold on;
grid on;
plot(data(:,35),data(:,36));
xlabel('Time [s]');
ylabel('$K_1$','interpreter','latex')
legend(['$K_1 = $' num2str(data(end,36))],'interpreter','latex')

subplot(422)
hold on;
grid on;
plot(data(:,35),data(:,37));
xlabel('Time [s]');
ylabel('$K_2$','interpreter','latex')
legend(['$K_2 = $' num2str(data(end,37))],'interpreter','latex')

subplot(423)
hold on;
grid on;
plot(data(:,35),data(:,38));
xlabel('Time [s]');
ylabel('$K_3$','interpreter','latex')
legend(['$K_3 = $' num2str(data(end,38))],'interpreter','latex')

subplot(424)
hold on;
grid on;
plot(data(:,35),data(:,39));
xlabel('Time [s]');
ylabel('$K_4$','interpreter','latex')
legend(['$K_4 = $' num2str(data(end,39))],'interpreter','latex')

subplot(425)
hold on;
grid on;
plot(data(:,35),data(:,40));
xlabel('Time [s]');
ylabel('$K_5$','interpreter','latex')
legend(['$K_5 = $' num2str(data(end,40))],'interpreter','latex')

subplot(426)
hold on;
grid on;
plot(data(:,35),data(:,41));
xlabel('Time [s]');
ylabel('$K_6$','interpreter','latex')
legend(['$K_6 = $' num2str(data(end,41))],'interpreter','latex')

subplot(427)
hold on;
grid on;
plot(data(:,35),data(:,42));
xlabel('Time [s]');
ylabel('$K_7$','interpreter','latex')
legend(['$K_7 = $' num2str(data(end,42))],'interpreter','latex')

subplot(428)
hold on;
grid on;
plot(data(:,35),data(:,43));
xlabel('Time [s]');
ylabel('$K_8$','interpreter','latex')
legend(['$K_8 = $' num2str(data(end,43))],'interpreter','latex')
