%% RECEBE COMANDOS PELA REDE E ENVIA DADOS DOS SENSORES
% Este programa apenas recebe comandos enviados de outro pc e os repassa ao
% pioneer. Além disso, envia para rede dados dos sensores do robô (u e w)

clear
close all
clc
% Fecha todas possíveis conexões abertas
try
    fclose(instrfindall);
catch
end

%% Rotina para buscar pasta raiz
PastaAtual = pwd;
PastaRaiz = 'AuRoRA 2018';
cd(PastaAtual(1:(strfind(PastaAtual,PastaRaiz)+numel(PastaRaiz)-1)))
addpath(genpath(pwd))

%% Classes initialization
% Robot
P = Pioneer3DX;   % the ID must be diferent of the pioneer in the server machine to avoid conflict on NetDataShare

% Setup variaveis de ambiente Ros Master, Ros Matlab

setenv('ROS_IP','192.168.0.106')                        %Ip do desktop
setenv('ROS_HOSTNAME','192.168.0.106')                  %Host name = ip
setenv('ROS_MASTER_URI','http://192.168.0.148:11311/')  %Ip do master (raspberry)
rosinit                                                 %iniciar a comunicação

%% Connect with robot or simulator MobileSim
P.rConnect;

%% Define robot initial position
[pub, pose] = rospublisher('/matlab/pose','nav_msgs/Odometry');

pose.Pose.Pose.Position.X = 0;
pose.Pose.Pose.Position.Y = 0;
pose.Pose.Pose.Orientation.Z = 0;
send(pub, pose)


sub = rossubscriber('/matlab/cmd_vel','geometry_msgs/Twist');

% P.rSetPose();             % set initial position

%% Variables initialization
% Time variables
tsim = 120;   % maximum simulation time
t    = tic;   % simulation current time
tc   = tic;   % sample time to send data to robot
tr   = tic;   % timer to reset pioneer pose
% Simulation data
data = [];    % save simulation data

%% Simulation
while true % toc(t) < tsim
    
    if toc(tc) > 0.030
        tc = tic;
        
        % Read sensors data
        P.rGetSensorData;
        
        % Send data to network
        pose.Pose.Pose.Position.X = P.pPos.Xc(1);
        pose.Pose.Pose.Position.Y = P.pPos.Xc(2);
        pose.Pose.Pose.Orientation.Z = P.pPos.X(6);
        send(pub, pose)
        
        % Read Network
        cmd_vel = sub.LatestMessage;
        
        
        % Assign variables
        P.pSC.Ud  = [cmd_vel.Linear.X cmd_vel.Angular.Z];       % control signal
            
        
        % Send commands to pioneer
        P.rSendControlSignals;

     end
    % Reset pioneer position periodically
    
end

%%  Stop
% Zera velocidades do robô
P.pSC.Ud = [0 ; 0];
P.rSendControlSignals;


