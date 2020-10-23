%% 3D Line Formation Pioneer-Drone
% Position task using a 3D line virtual structure to control the formation
%
%  In this script, the ArDrone is commanded to land during the
%  experiment. Later, its take off again to continue the task until the
%  end.
%
% I.   Pioneer is the reference of the formation
% II.  The formation variables are:
%      Q = [xf yf zf rhof alfaf betaf]
%  *the formation variables are specified in formation class file (LineFormation3D.m)

% Close communication with raspberry (for pioneer)
clear; close all; clc;
try
    fclose(instrfindall);
catch
end

try
    % Load Classes
    RI = RosInterface;
    RI.rConnect('192.168.0.146');
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

%% Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
% addpath(genpath(pwd));

%% Load Classes
% Robot
P = Pioneer3DX(1);
% pgains = [1.5 1 1.5 1]; % controller gains
% pgains = [0.3 0.3 0.45 0.45]; % controller gains
pgains = [0.1 0.1 1];         % gains (for extended controller)
P.pPar.a = 0.15;                 % control point (for extended controller)

% Drone
B = Bebop(1,'B1');
B.pPar.Model_simp = [ 0.8417 0.18227 0.8354 0.17095 3.966 4.001 9.8524 4.7295 ]';
B.pPar.Ts = 1/30;

% 3D Formation
LF = LineFormation3D;
% Trajectory formation gains
LF.pPar.K1 = 1*diag([0.2 0.2 0.2 3 1 1.8]);        % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([1 1 1 1 1 1]);        % kinematic control gain - control saturation

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network (Pioneer-Raspberry)
% Setup Ros Master, Ros Matlab environment variables
% Tutorial para rodar o pioneer com raspberry (2020)
% 1. Conectar o raspberry ao serial do pioneer (e obviamente alimentá-lo corretamente);
% 2. No PC, abrir o terminal PuTTY;
% 3. No terminal, login e senha: ubuntu
% 4. Digitar o comando: "rosrun rosaria RosAria 192.168.0.148"  e pressionar ENTER
% 5. No programa, utilizar as linhas de código adiante para conectar ao pioneer

setenv('ROS_IP','192.168.0.158')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.146:11311/')  %Ip do master (raspberry)
% rosinit                                                 %iniciar a comunicação

[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist');

%% Open file to save data
NomeArq = datestr(now,30);
% cd('DataFiles')
% cd('Log_Optitrack')
Arq = fopen(['FL3d_TrajectoryUtimateBebop' NomeArq '.txt'],'w');
% cd(PastaAtual)
dados = [];


%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);          % pioneer ID on optitrack
idB = getID(OPT,B);          % drone ID on optitrack

% Variables to calculate velocities inside optitrack function
P.pPar.ti = tic;             % pioneer

rb = OPT.RigidBody;          % read optitrack data
B = getOptData(rb(idB),B);   % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

B.rTakeOff;
pause(3);                     % time for drone stabilization
disp('READY?? GO! GO! GO!');

%% Variable initialization
% Saves data to plot
data = [];
% Desired positions vector [xf yf zf rho alpha beta]

Qd = [  0   0     0   1.5    0     pi/2];      % funcionou legal (marcos 13/01/2020)

cont = 1;     % counter to change desired position through simulation
time = 10;    % time to change desired positions [s]

% First desired position
LF.pPos.Qd = Qd(1,:)';
% Robots desired pose
LF.mInvTrans;

%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;

%% Simulation
% Time variables initialization
rX = 0.85; % [m]
rY = 1.5; % [m]
rho = 1.5;
T = 25;   % [s]
nVoltas = 2;
Tf = T*nVoltas;
w = 2*pi/T; % [rad/s]

Xd = zeros(3,1);
Xda = zeros(3,1);
dXd = zeros(3,1);
XTs = 1/30;

WantedFlag = 0;

tsim    = size(Qd,1)*(time);
t       = tic;
tc      = tic;
tp      = tic;
B.pSC.Kinematics_control = 1;

%%%%%%%%%%%%%%%%%%%%%% Botão de Emergencia %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nLandMsg = 3;
btnEmergencia = 0;
ButtonHandle = uicontrol('Style', 'PushButton', ...
    'String', 'land', ...
    'Callback', 'btnEmergencia=1', ...
    'Position', [50 50 400 300]);



while toc(t)< Tf
    
    %% Desired positions
    if toc(tc) > 1/30
        tc = tic;
        %% Trajectory - 8'
        
        t_traj = toc(t);
        a = 3*(t_traj/Tf)^2 - 2*(t_traj/Tf)^3;
        tp = a*Tf;
        
        Xda = Xd;
        Xd(1) = rX*sin(2*w*tp);                   % xF
        Xd(2) = rY*sin(w*tp);                 % yF
        Xd(3) = 0.00;                           % zF
        dXd = (Xd - Xda)/XTs;
        
        
        if toc(t) < 13
            
            LF.pPos.Qd = [ Xd(1)  Xd(2) Xd(3)  1.5    (3*pi/4+P.pPos.X(6))     pi/3]';      
            LF.pPos.dQd = [ dXd(1)  dXd(2) dXd(3)  0    0     0]';      
        
            % LAND
        elseif toc(t) < 30
            
            LF.pPos.Qd = [ Xd(1)  Xd(2) Xd(3)  0.75    0     pi/2]';      
            LF.pPos.dQd = [ dXd(1)  dXd(2) dXd(3)  0    0     0]';      
            
        elseif toc(t) < 46
            
            LF.pPos.Qd = [ Xd(1)  Xd(2) Xd(3)  1.5  0     pi/2]';      
            LF.pPos.dQd = [ dXd(1)  dXd(2) dXd(3)  0    0     0]';     
        
            % FINAL LAND
        else
            
            LF.pPos.Qd = [ Xd(1)  Xd(2) Xd(3)  0.75    0     pi/2]';      
            LF.pPos.dQd = [ dXd(1)  dXd(2) dXd(3)  0    0     0]';      
            
        end
        
        
        %% Acquire sensors data
        % Get optitrack data
        rb = OPT.RigidBody;             % read optitrack
        % Ardrone
        B  = getOptData(rb(idB),B);
        % Pioneer
        P = getOptData(rb(idP),P);
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); B.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired positions ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position / tem que integrar o dXr da formação para obter o Xd do robô
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
        
        % Drone
        B.pPos.Xr(7:9) = LF.pPos.dXr(4:6);
        B.pPos.Xda(6) = B.pPos.Xd(6);
        B.pPos.Xd(6) = P.pPos.X(6);   %yaw desejado
        
        if abs(B.pPos.Xd(6)) > pi
            if B.pPos.Xd(6) > 0
                B.pPos.Xd(6) = B.pPos.Xd(6) - 2*pi;
            else
                B.pPos.Xd(6) = B.pPos.Xd(6) + 2*pi;
            end
        end
        
        B.pPos.Xd(12) = (B.pPos.Xd(6) - B.pPos.Xda(6))/(1/30);  %vyaw desejado

        
        % Dynamic compensation ................................................
        % Drone
        B.cInverseDynamicController_Compensador;
        B = J.mControl(B);
        % Pioneer
        %         P = fKinematicControllerExtended(P);        % new controller (by timotiu 2020)
        P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020)
        
        %% Save data (.txt file)
        % Data file structure
        % (1:12)      (13:24)    (25:26)   (27:28)  ...
        % P.pPos.Xd   P.pPos.X   P.pSC.Ud  P.pSC.U  ...
        %
        % (29:40)     (41:52)    (53:58)   (59:62)  ...
        % B.pPos.Xd   B.pPos.X   B.pSC.Ud  B.pSC.U  ...
        %
        % (63:68)     (69:74)    (75)
        % LF.pPos.Qd  LF.pPos.Q   Simulation time
        
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
        
        dados(end+1,:) =  [P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            B.pPos.Xd' B.pPos.X' B.pSC.Ud' B.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
        
        %% Send control signals
        % to Pioneer (by raspberry)
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel)
        
        % to bebop
        %         B.rCommand;
        
        %% Land and take off
        if toc(t) > 33.5 && toc(t) < 35 && WantedFlag == 1        % Taking off
            for i=1:nLandMsg
                B.rTakeOff;                   % takeoff again
            end
            %                 disp('LOOP TAKEOFF');
            B.pFlag.EmergencyStop = 0;
            WantedFlag = 0;
            B.rCommand;
            
        elseif toc(t) > 16.5 && toc(t) < 18   % Landing
            %                 disp('LOOP LAND');
            for i=1:nLandMsg
                B.rCmdStop;
                B.rLand;
                send(B.pubBrLand,B.pStdMsgEmpty);
            end
            B.pFlag.EmergencyStop = 0;
            WantedFlag = 1;
        else
            B.rCommand;
        end
        
        

        % If push Emergency or ROS Emergency Stop On or Not Rigid Body tracked Stop loop
        drawnow
        if btnEmergencia ~= 0 || (WantedFlag == 0 && B.pFlag.EmergencyStop ~= 0) || B.pFlag.isTracked ~= 1
            disp('Bebop Landing through Emergency Command ');
            
            % Send 3 times Commands 1 second delay to Drone Land
            for i=1:nLandMsg
                disp("End Land Command");
                B.rCmdStop;
                B.rLand;
                P.pSC.Ud = [0;0];
                cmd_vel.Linear.X = P.pSC.Ud(1);
                cmd_vel.Angular.Z = P.pSC.Ud(2);
                send(pub,cmd_vel)
                
            end
            break;
        end
        
        
    end
    
    
end


%% Close files
fclose(Arq);
%%  Stop robot
% Send control signals
P.pSC.Ud = [0;0];
cmd_vel.Linear.X = P.pSC.Ud(1);
cmd_vel.Angular.Z = P.pSC.Ud(2);
send(pub,cmd_vel)

% Land drone
for i=1:5
    disp("Stop Command");
    B.rCmdStop;
    B.rLand;
    P.pSC.Ud = [0;0];
    cmd_vel.Linear.X = P.pSC.Ud(1);
    cmd_vel.Angular.Z = P.pSC.Ud(2);
    send(pub,cmd_vel)
end

% Close ROS Interface
RI.rDisconnect;
rosshutdown;

%% Plot results

figure
subplot(2,1,1)
plot(dados(:,end),dados(:,25))
grid on
subplot(2,1,2)
plot(dados(:,end),dados(:,26))
grid on

%%


figure
subplot(6,1,1)
plot(dados(:,end),(dados(:,60+(1))-dados(:,66+(1))))
grid on
title('X')
subplot(6,1,2)
plot(dados(:,end),(dados(:,60+(2))-dados(:,66+(2))))
grid on
title('Y')
subplot(6,1,3)
plot(dados(:,end),(dados(:,60+(3))-dados(:,66+(3))))
grid on
title('Z')
subplot(6,1,4)
plot(dados(:,end),(dados(:,60+(4))-dados(:,66+(4))))
grid on
title('rho')
subplot(6,1,5)
plot(dados(:,end),(dados(:,60+(5))-dados(:,66+(5))))
grid on
title('alpha')
subplot(6,1,6)
plot(dados(:,end),(dados(:,60+(6))-dados(:,66+(6))))
grid on
title('beta')

figure
subplot(6,1,1)
plot(dados(:,end),dados(:,60+(1)))
hold on
plot(dados(:,end),dados(:,66+(1)))
title('X')
subplot(6,1,2)
plot(dados(:,end),dados(:,60+(2)))
hold on
plot(dados(:,end),dados(:,66+(2)))
title('Y')
subplot(6,1,3)
plot(dados(:,end),dados(:,60+(3)))
hold on
plot(dados(:,end),dados(:,66+(3)))
title('Z')
subplot(6,1,4)
plot(dados(:,end),dados(:,60+(4)))
hold on
plot(dados(:,end),dados(:,66+(4)))
title('rho')
subplot(6,1,5)
plot(dados(:,end),dados(:,60+(5)))
hold on
plot(dados(:,end),dados(:,66+(5)))
title('alpha')
subplot(6,1,6)
plot(dados(:,end),dados(:,60+(6)))
hold on
plot(dados(:,end),dados(:,66+(6)))
title('beta')

% figure
% subplot(3,1,1)
% plot(dados(:,end),dados(:,28+(1)))
% hold on
% plot(dados(:,end),dados(:,40+(1)))
% title('X')
% subplot(3,1,2)
% plot(dados(:,end),dados(:,28+(2)))
% hold on
% plot(dados(:,end),dados(:,40+(2)))
% title('Y')
% subplot(3,1,3)
% plot(dados(:,end),dados(:,28+(3)))
% hold on
% plot(dados(:,end),dados(:,40+(3)))
% title('Z')
%
% figure
% subplot(3,1,1)
% plot(dados(:,end),dados(:,40+(7)))
% title('Velocidade X')
% subplot(3,1,2)
% plot(dados(:,end),dados(:,40+(8)))
% title('Velocidade Y')
% subplot(3,1,3)
% plot(dados(:,end),dados(:,40+(9)))
% title('Velocidade Z')


% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
