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
try
    rosshutdown
end

clear; close all; clc;


try
    fclose(instrfindall);
catch
end

%% Look for root folder
% PastaAtual = pwd;
% PastaRaiz = 'AuRoRA 2018';
% cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)));
% addpath(genpath(pwd));

%% Load Classes
% Robots
% Pioneer
P = Pioneer3DX(1);
% pgains = 0.2*[1.5 1 1.5 1];   % trajectory controller gains 
% pgains = [0.3 0.3 0.45 0.45]; % position controller gains 
pgains = [0.1 0.1 1];         % gains (for extended controller)
P.pPar.a = 0.15;                 % control point (for extended controller) 
% P.pPar.alpha = 0;           % control point angle (for extended controller)

% Drone
A = ArDrone(30);
A.pPar.Ts = 1/30;             % refresh rate

% 3D Formation 
LF = LineFormation3D;
% Trajectory formation gains
LF.pPar.K1 = 1*diag([0.7 0.7 0.7 3 1 1.1]);        % kinematic control gain  - controls amplitude
% LF.pPar.K1 = 1*diag([1 1 1 3 1 1.2]);        % kinematic control gain  - controls amplitude
LF.pPar.K2 = 1*diag([1 1 1 1 1 1]);              % kinematic control gain - control saturation

% Joystick
J = JoyControl;

% Create OptiTrack object and initialize
OPT = OptiTrack;
OPT.Initialize;

% Network (Pioneer-Raspberry)
% Setup Ros Master, Ros Matlab environment variables 
% Tutorial Simplificado para rodar o pioneer com raspberry (2020)
% 1. Conectar o raspberry ao serial do pioneer (e obviamente alimentá-lo corretamente);
% 2. No PC, abrir o terminal PuTTY; 
% 3. No terminal, login e senha: ubuntu
% 4. Digitar o comando: "rosrun rosaria RosAria 192.168.0.148"  e pressionar ENTER
% 5. No programa, utilizar as linhas de código adiante para conectar ao pioneer

setenv('ROS_IP','192.168.0.158')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.158')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.148:11311/')  %Ip do master (raspberry)
rosinit                                                 %iniciar a comunicação

[pub,cmd_vel] = rospublisher('/RosAria/cmd_vel','geometry_msgs/Twist');

%% Open file to save data
NomeArq = datestr(now,30);
% cd('DataFiles')
% cd('Log_Optitrack')
Arq = fopen(['FL3d_TrajectoryUtimateExpNewControllers' NomeArq '.txt'],'w');
% cd(PastaAtual)
dados = [];

%% Robot/Simulator conection
% A.pPar.ip = '192.168.1.50';      % ar.drone IP (use ipconfig to verify)
% A.pPar.ip = '192.168.1.30';    % ar.drone IP
A.pPar.ip = '192.168.1.40';    % ar.drone IP
A.rConnect;

%% Robots initial pose
% detect rigid body ID from optitrack
idP = getID(OPT,P);          % pioneer ID on optitrack
idA = getID(OPT,A);          % drone ID on optitrack

% Variables to calculate velocities inside optitrack function
P.pPar.ti = tic;             % pioneer
A.pPar.ti = tic;             % drone

rb = OPT.RigidBody;          % read optitrack data
A = getOptData(rb(idA),A);   % get ardrone data
P = getOptData(rb(idP),P);   % get pioneer data

A.rTakeOff;
pause(4);                     % time for drone stabilization
disp('READY?? GO! GO! GO!');

%% Variable initialization
% Saves data to plot
data = [];

%% Trajectory variables
a  = 1;             % x distance
b  = 1.4; %1;       % y distance
w  = 0.25; % .25 ; .3           % angular velocity for pioneer

% First desired position
% Desired formation [xf yf zf rhof alfaf betaf]
% Qd = [1 1 0 1.5 0 pi/2]';
% Qd = [1 1 0 1.5 pi/2 pi/3]';
Qd = [1 1 0 1.5 pi pi/3]';

LF.pPos.Qd = Qd;                % initial desired position

% Robots desired pose
LF.mInvTrans;
%% Formation initial error
% Formation initial pose
LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];

% Formation initial pose
LF.mDirTrans;

% Formation Error
LF.mFormationError;

%% Simulation
% Time variables initialization
nlap    = 3;             % number of laps 
tsim    = 2*pi*nlap/w;   % trajectory duration time

t       = tic;
tc      = tic;
tp      = tic;

while toc(t)< tsim && (J.pFlag==0)
    
    %% Desired positions
    if toc(tc) > 1/30
        
        tc = tic;
        
        %% Formation changes
        % Changes the desired variables 
        if toc(t) > 2*tsim/nlap 
            LF.pPos.Qd(4) = 1.5;     % rho
            Qd(5) = pi;          % alpha angle                       
            LF.pPos.Qd(6) = Qd(6); % beta angle            
        elseif toc(t) > tsim/nlap      
            LF.pPos.Qd(4) = 1; %0.8     % rho
            Qd(5) = pi;             % alpha angle                       
            LF.pPos.Qd(6) = pi/2;  % beta angle            
        end
        
        % Landing and taking off
        if toc(t) > 2*tsim/nlap
            if A.pCom.cStatus(32)==0      % flying state flag (1=flying/0=landed)
                A.rTakeOff;               % takeoff again 
            end
        elseif toc(t) > tsim/nlap + 8     % landing time + extra time to be safe lol
           if A.pCom.cStatus(32)==1       % flying state flag (1=flying/0=landed)
                A.rLand;
           end
        end
        

         %% Trajectory
        % Elipse
        ta = toc(t);
        % Positions
        LF.pPos.Qd(1)  = a*cos(w*ta);          % x position
        LF.pPos.Qd(2)  = b*sin(w*ta);          % y position
        LF.pPos.Qd(5)  = Qd(5) + P.pPos.X(6);  % alpha angle
        % Quadrant correction
        if abs(LF.pPos.Qd(5)) > pi
            if LF.pPos.Qd(5) > 0
                LF.pPos.Qd(5) = LF.pPos.Qd(5) - 2*pi;
            else
                LF.pPos.Qd(5) = LF.pPos.Qd(5) + 2*pi;
            end
        end
        
        % Velocities
        LF.pPos.dQd(1) = -a*w*sin(w*ta);  % x velocity
        LF.pPos.dQd(2) = b*w*cos(w*ta);   % y velocity
        LF.pPos.dQd(5) = P.pPos.X(12);    %0;               % alpha velocity
        
        %% Acquire sensors data        
        % Get optitrack data        
        rb = OPT.RigidBody;             % read optitrack
        
        % Ardrone
        A  = getOptData(rb(idA),A);      
        % Pioneer
        P = getOptData(rb(idP),P);
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired positions ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position / tem que integrar o dXr da formação para obter o Xd do robô
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
        
        % Drone
%         A.pPos.Xd(6) = atan2(A.pPos.Xd(2)-A.pPos.X(2),A.pPos.Xd(1)-A.pPos.Xd(1));   % desired Yaw following trajectory
%         A.pPos.Xd(12) = (A.pPos.Xd(12)-A.pPos.Xda(12))/toc(A.pPar.ti);           % desired dYaw 
%         A.pPos.Xda(12) = A.pPos.Xd(12);
        A.pPos.Xd(6) = 0; %P.pPos.X(6);   %desired Yaw 
        A.pPos.Xd(12) = 0; %P.pPos.X(12);  %desired dYaw 
        
        % Dynamic compensation ................................................
        % Drone 
        A.pSC.Kinematics_control = 1; % flag to avoid second kinematic control (inside controller)
        A.pPos.Xr(7:9) = LF.pPos.dXr(4:6); % passando a referencia para o drone 
        A = cInverseDynamicController_Compensador_ArDrone(A);       
        A = J.mControl(A);                          % Joystick control for desperate times  
        
        % Pioneer
%         pgains = [0 0 0];         % gains (for extended controller)
        P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020)
        
        %% Save data (.txt file)
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
       dados(end+1,:) =  [P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
       
        %% Send control signals 
        % to Pioneer (by raspberry)
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel)
%         
        % to Ardrone 
        A.rSendControlSignals;
        
    end
    
    % Exit loop if stop button is pressed
    if J.pFlag == 1
       break 
    end
    
    
end

%% Loop: Final Landing
tsim = toc(t);
tland = 4;
% tland = 0;

while (toc(t) < tsim+tland) && (J.pFlag==0)
    if toc(tc) > 1/30
        tc = tic;
        %% Position
        
        ta = toc(t);
        % Positions
        LF.pPos.Qd(1)  = LF.pPos.Q(1);         % x position
        LF.pPos.Qd(2)  = LF.pPos.Q(2);         % y position
        LF.pPos.Qd(4)  = Qd(4,end) - (Qd(4,end)-0.5)*(ta-tsim)/(tland);    % rho (distance)
        LF.pPos.Qd(6)  = pi/2;                 % beta (vertical angle)
        % Velocities
        LF.pPos.dQd(1)  = 0;                   % x velocities
        LF.pPos.dQd(2)  = 0;                   % y velocities
        LF.pPos.dQd(3)  = 0;                   % z velocities
        LF.pPos.dQd(4)  = 0;                   % rho velocities
        LF.pPos.dQd(5)  = 0;                   % alpha velocities
        LF.pPos.dQd(6)  = 0;                   % beta velocities
        
        %% Acquire sensors data        
        % Get optitrack data        
        rb = OPT.RigidBody;             % read optitrack       
        A  = getOptData(rb(idA),A);     % Ardrone     
        P = getOptData(rb(idP),P);      % Pioneer
        
        %% Control
        % Formation Members Position
        LF.pPos.X = [P.pPos.X(1:3); A.pPos.X(1:3)];
        
        % Formation Controller
        LF.mFormationControl;
        
        % Desired positions ...........................................
        LF.mInvTrans;
        % Pioneer
        P.pPos.Xd(1:3) = LF.pPos.Xd(1:3);        % desired position / tem que integrar o dXr da formação para obter o Xd do robô
        P.pPos.Xd(7:9) = LF.pPos.dXr(1:3);       % desired velocities
            
        % Dynamic compensation ................................................
%         A.pPos.Xd(6) = atan2(LF.pPos.Qd(2),LF.pPos.Qd(1));   % desired Yaw following trajectory
%         A.pPos.Xd(12) = (A.pPos.Xd(12)-A.pPos.Xda(12))/toc(A.pPar.ti);           % desired dYaw 
%         A.pPos.Xda(12) = A.pPos.Xd(12);
% 
%         A.pPos.Xd(6) =  P.pPos.X(6);   %desired Yaw 
%         A.pPos.Xd(12) = P.pPos.X(12);  %desired dYaw 
         A.pPos.Xd(6) = 0; %P.pPos.X(6);   %desired Yaw 
         A.pPos.Xd(12) = 0; %P.pPos.X(12);  %desired dYaw 
        
        A.pSC.Kinematics_control = 1;         % flag to avoid kinematic controller
        A.pPos.Xr(7:9) = LF.pPos.dXr(4:6);    % passando a referencia para o drone 
        A = cInverseDynamicController_Compensador_ArDrone(A);
        
        A = J.mControl(A);                        % Joystick control for desperate times  
        % Pioneer
        pgains = [0 0 0];         % gains (for extended controller)
        P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020)
%         P = fKinematicControllerExtended(P,pgains);        % new controller (by timotiu 2020) 
        %% Save data (.txt file)
        % Data file structure
        % (1:12)      (13:24)    (25:26)   (27:28)  ...     
        % P.pPos.Xd   P.pPos.X   P.pSC.Ud  P.pSC.U  ...
        %
        % (29:40)     (41:52)    (53:56)   (57:60)  ...     
        % A.pPos.Xd   A.pPos.X   A.pSC.Ud  A.pSC.U  ...
        %
        % (61:66)     (67:72)    (73)
        % LF.pPos.Qd  A.pPos.Q   Simulation time
       
        fprintf(Arq,'%6.6f\t',[P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)]);
        fprintf(Arq,'\n\r');
       dados(end+1,:) =  [P.pPos.Xd' P.pPos.X' P.pSC.Ud(1:2)' P.pSC.U(1:2)' ...
            A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' LF.pPos.Qd' LF.pPos.Q' toc(t)];
        
        
        %% Send control signals to robots
        % Pioneer
        P.pSC.Ud = [0; 0];                 % stop Pioneer
        cmd_vel.Linear.X = P.pSC.Ud(1);
        cmd_vel.Angular.Z = P.pSC.Ud(2);
        send(pub,cmd_vel)
        % Drone
        A.rSendControlSignals;
    end
    

end

Xtil = dados(:,28+(1:12)) - dados(:,40+(1:12)); 
figure;
plot(dados(:,end), dados(:,34),dados(:,end), dados(:,46))

%% Close data file
fclose(Arq);
%%  Stop robot
% Pioneer
P.pSC.Ud = [0;0];         % define zero control signals
cmd_vel.Linear.X = P.pSC.Ud(1);
cmd_vel.Angular.Z = P.pSC.Ud(2);
send(pub,cmd_vel)

% Land drone
if A.pFlag.Connected == 1
%     A.rEmergency;       % Stops motors
    A.rLand;
end

%% Plots (feito por João, tenho minhas dúvidas se está certo...)
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

figure
subplot(3,1,1)
plot(dados(:,end),dados(:,28+(1)))
hold on
plot(dados(:,end),dados(:,40+(1)))
title('X')
subplot(3,1,2)
plot(dados(:,end),dados(:,28+(2)))
hold on
plot(dados(:,end),dados(:,40+(2)))
title('Y')
subplot(3,1,3)
plot(dados(:,end),dados(:,28+(3)))
hold on
plot(dados(:,end),dados(:,40+(3)))
title('Z')

figure
subplot(3,1,1)
plot(dados(:,end),dados(:,40+(7)))
title('Velocidade X')
subplot(3,1,2)
plot(dados(:,end),dados(:,40+(8)))
title('Velocidade Y')
subplot(3,1,3)
plot(dados(:,end),dados(:,40+(9)))
title('Velocidade Z')

% End of code xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
